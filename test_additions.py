"""Synthetic image / fake telemetry tests; these never connect to a drone."""
import asyncio
import base64
import contextlib
import csv
import importlib.util
import io
import math
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, patch
import numpy as np
import camera_sensor as camera
import flight_csv as logger


class VisionTests(unittest.TestCase):
    def test_red_and_non_red(self):
        red = np.full((100, 100, 3), (0, 0, 255), np.uint8)
        self.assertEqual(camera.detect_red(red), (True, 1.0))
        for color in ((255, 0, 0), (0, 255, 0), (0, 0, 0), (255, 255, 255)):
            self.assertFalse(camera.detect_red(np.full((100,100,3), color, np.uint8))[0])

    def test_rgb_decode_with_padding(self):
        payload = bytes([255,0,0, 0,255,0, 99,99])
        message = dict(width=2, height=1, step=8, pixelFormatType='RGB_INT8',
                       data=base64.b64encode(payload).decode())
        image = camera.decode_image(message)
        self.assertEqual(image.tolist(), [[[0,0,255], [0,255,0]]])
        message['data'] = base64.b64encode(b'bad').decode()
        with self.assertRaises(ValueError): camera.decode_image(message)

    def test_rgba_numeric_format(self):
        message = dict(width=1,height=1,step=4,pixel_format_type=4,
                       data=base64.b64encode(bytes([255,0,0,255])).decode())
        self.assertEqual(camera.decode_image(message).tolist(), [[[0,0,255]]])
        message['pixel_format_type'] = 13
        with self.assertRaises(ValueError): camera.decode_image(message)


class CameraTests(unittest.IsolatedAsyncioTestCase):
    async def test_separate_topics(self):
        output = '\n'.join('/world/default/model/'+model+'/link/camera/sensor/camera/image'
                           for model in camera.MODELS.values())
        with patch.object(camera, 'TOPICS', {14540:'',14541:''}), \
             patch.object(camera, 'gz_command', new=AsyncMock(return_value=output)):
            first = await camera.find_camera_topic(14540)
            second = await camera.find_camera_topic(14541)
            self.assertNotEqual(first,second)

    async def test_missing_camera_is_error(self):
        with patch.object(camera,'TOPICS',{14540:'',14541:''}), \
             patch.object(camera,'gz_command',new=AsyncMock(return_value='/camera')):
            with self.assertRaises(RuntimeError): await camera.find_camera_topic(14540)

    async def test_timeout_reaps_gz_child(self):
        proc = NS(returncode=None)
        async def communicate(): await asyncio.Event().wait()
        def kill(): proc.returncode=-9
        proc.communicate=communicate
        proc.kill=kill
        proc.wait=AsyncMock()
        with patch.object(asyncio,'create_subprocess_exec',new=AsyncMock(return_value=proc)):
            with self.assertRaises(asyncio.TimeoutError):
                await camera.gz_command('topic','-l',timeout=0.01)
        proc.wait.assert_awaited_once()


async def repeating(value):
    while True:
        yield value
        await asyncio.sleep(0.05)


class CsvTests(unittest.IsolatedAsyncioTestCase):
    async def test_logging_and_summary(self):
        pv = NS(position=NS(north_m=100,east_m=50,down_m=5),
                velocity=NS(north_m_s=0,east_m_s=0,down_m_s=0))
        tele = NS(position_velocity_ned=lambda:repeating(pv),
                  battery=lambda:repeating(NS(remaining_percent=0.8,voltage_v=15.2)),
                  attitude_euler=lambda:repeating(NS(roll_deg=1,pitch_deg=2,yaw_deg=3)),
                  set_rate_position_velocity_ned=AsyncMock(),set_rate_battery=AsyncMock(),
                  set_rate_attitude_euler=AsyncMock())
        with tempfile.TemporaryDirectory() as directory:
            state=await logger.start_logging(NS(telemetry=tele),'Drone 1',directory,time.monotonic())
            state['armed_at']=time.monotonic()
            state['takeoff_at']=state['armed_at']
            state['phase']='search'
            state['waypoint']='B'
            logger.log_detection(state,dict(camera_status='error',red_detected=None,
                red_fraction=None,image_sim_time_s=None,camera_topic='',snapshot='',camera_error='timeout'))
            await asyncio.sleep(0.25)
            state['landed_at']=time.monotonic()
            with contextlib.redirect_stdout(io.StringIO()):
                await logger.stop_logging(state,'completed')
            rows=list(csv.DictReader(io.StringIO((Path(directory)/'drone_1_flight.csv').read_text())))
            self.assertGreaterEqual(len(rows),2)
            last=rows[-1]
            self.assertEqual(float(last['north_m']),0)
            self.assertEqual(float(last['height_m']),0)
            self.assertEqual(float(last['battery_fraction']),0.8)
            detections=list(csv.DictReader(io.StringIO((Path(directory)/'drone_1_detections.csv').read_text())))
            self.assertEqual(detections[0]['red_detected'],'')
            self.assertEqual(detections[0]['camera_status'],'error')
            summary=list(csv.DictReader(io.StringIO((Path(directory)/'drone_1_summary.csv').read_text())))[0]
            self.assertEqual(summary['landing_confirmed'],'1')
            self.assertGreater(float(summary['mission_duration_s']),0)
            self.assertTrue(all(t.done() for t in state['tasks']))

    async def test_stale_telemetry_is_blank(self):
        state=dict(start=time.monotonic(),phase='travel',waypoint='B',distance=0,armed_at=None,
                   position=(time.monotonic()-3,{'north_m':5}))
        row=logger.row_now(state)
        self.assertIsNone(row['north_m'])
        self.assertGreaterEqual(row['position_age_s'],3)


def load_mission_with_fake_sdk():
    fake=types.ModuleType('mavsdk_grpc')
    fake.System=lambda **kw:None
    fake.__file__='/not-used/__init__.py'
    action=types.ModuleType('mavsdk_grpc.action')
    action.OrbitYawBehavior=NS(HOLD_FRONT_TO_CIRCLE_CENTER=2)
    spec=importlib.util.spec_from_file_location('mission_for_test',Path(__file__).with_name('simple.py'))
    mission=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,{'mavsdk_grpc':fake,'mavsdk_grpc.action':action}):
        spec.loader.exec_module(mission)
    return mission


async def once(value): yield value


class MissionTests(unittest.IsolatedAsyncioTestCase):
    async def test_new_route_camera_checks_and_preserved_orbit(self):
        m=load_mission_with_fake_sdk()
        for sign in (1,-1):
            target=[40.,-83.,100.]
            async def go(lat,lon,alt,yaw): target[:]=[lat,lon,alt]
            drone=NS(connect=AsyncMock(),core=NS(connection_state=lambda:once(NS(is_connected=True))),
                telemetry=NS(health=lambda:once(NS(is_global_position_ok=True,is_home_position_ok=True)),
                    armed=lambda:once(False),in_air=lambda:once(False),
                    position=lambda:once(NS(latitude_deg=target[0],longitude_deg=target[1],absolute_altitude_m=target[2])),
                    position_velocity_ned=lambda:once(NS(position=NS(north_m=0,east_m=0,down_m=-10),
                        velocity=NS(north_m_s=0,east_m_s=0,down_m_s=0)))),
                action=NS(set_takeoff_altitude=AsyncMock(),arm=AsyncMock(),takeoff=AsyncMock(),
                    goto_location=AsyncMock(side_effect=go),set_current_speed=AsyncMock(),
                    do_orbit=AsyncMock(),land=AsyncMock()))
            state={}
            m.RUN_FOLDER=Path('/not-used')
            m.RUN_STARTED=0
            result=dict(camera_status='ok',red_detected=1,red_fraction=0.02)
            with patch.object(m,'System',return_value=drone), \
                 patch.object(m,'start_logging',new=AsyncMock(return_value=state)), \
                 patch.object(m,'stop_logging',new=AsyncMock()) as stop, \
                 patch.object(m,'sense_camera',new=AsyncMock(return_value=result)) as sense, \
                 patch.object(m,'log_detection') as detection, \
                 patch.object(m.asyncio,'sleep',new_callable=AsyncMock) as sleep, \
                 contextlib.redirect_stdout(io.StringIO()):
                await m.main('Drone',14540,50051,sign)
            self.assertEqual(sense.await_count,27)
            self.assertEqual(detection.call_count,27)
            self.assertEqual(drone.action.goto_location.await_count,28) # includes final home
            self.assertEqual(drone.action.set_current_speed.await_count,27)
            orbit=drone.action.do_orbit.await_args.args
            self.assertEqual(orbit[:2],(5*sign,2))
            self.assertEqual(orbit[3:5],m.meters_to_coords(40,-83,0,5*sign))
            self.assertIn(math.pi*5/2,[c.args[0] for c in sleep.await_args_list])
            self.assertEqual(stop.await_args.args[1],'completed')
            self.assertEqual(drone.action.land.await_count,1)
            # All travel points remain on the correct side of each local origin.
            for call in drone.action.goto_location.await_args_list:
                self.assertGreaterEqual(sign*(call.args[1]+83),-1e-10)


if __name__ == '__main__': unittest.main()
