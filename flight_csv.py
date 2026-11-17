"""Background telemetry and CSV writing; no flight commands or navigation logic."""
import asyncio
import csv
import math
import time
from datetime import datetime, timezone
from pathlib import Path

INTERVAL = 0.2  # Log latest telemetry at about 5 rows/second (wall time).
FIELDS = ['utc_time', 'elapsed_s', 'mission_elapsed_s', 'phase', 'waypoint',
          'north_m', 'east_m', 'height_m', 'velocity_north_m_s', 'velocity_east_m_s',
          'velocity_down_m_s', 'speed_m_s', 'sampled_distance_m',
          'battery_fraction', 'voltage_v', 'roll_deg', 'pitch_deg', 'yaw_deg',
          'position_age_s', 'battery_age_s', 'attitude_age_s',
          'camera_status', 'red_detected', 'red_fraction', 'camera_age_s',
          'image_sim_time_s', 'camera_error']
DETECTION_FIELDS = ['elapsed_s', 'waypoint', 'north_m', 'east_m', 'height_m',
                    'camera_status', 'red_detected', 'red_fraction',
                    'image_sim_time_s', 'camera_topic', 'snapshot', 'camera_error']


def finite(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def row_now(state):
    """Leave missing/stale telemetry empty; never substitute fabricated zeroes."""
    now = time.monotonic()
    row = {key: None for key in FIELDS}
    row.update(utc_time=datetime.now(timezone.utc).isoformat(),
               elapsed_s=round(now-state['start'], 3), phase=state['phase'],
               waypoint=state['waypoint'], sampled_distance_m=state['distance'])
    if state['armed_at'] is not None:
        row['mission_elapsed_s'] = round(now-state['armed_at'], 3)
    for group, max_age in [('position', 2), ('battery', 5), ('attitude', 2)]:
        value = state.get(group)
        if value:
            row[group+'_age_s'] = round(now-value[0], 3)
            if now-value[0] <= max_age:
                row.update(value[1])
    camera = state.get('camera')
    if camera:
        row['camera_age_s'] = round(now-camera[0], 3)
        # Keep camera results only on search rows. Other phases have no new image.
        if state['phase'] == 'search':
            for key in ('camera_status', 'red_detected', 'red_fraction',
                        'image_sim_time_s', 'camera_error'):
                row[key] = camera[1].get(key)
    return row


async def collect_position(drone, state):
    try:
        async for pv in drone.telemetry.position_velocity_ned():
            p, v = pv.position, pv.velocity
            xyz = (p.north_m, p.east_m, p.down_m)
            velocity = (v.north_m_s, v.east_m_s, v.down_m_s)
            if not all(finite(x) is not None for x in xyz+velocity):
                continue
            if state['origin'] is None:
                state['origin'] = xyz  # Ground position, before arming.
                state['ready'].set()
            n, e, d = state['origin']
            if state['armed_at'] is not None:
                if state['previous'] is not None:
                    state['distance'] += math.dist(state['previous'], xyz)
                state['previous'] = xyz
            state['position'] = (time.monotonic(), {
                'north_m': p.north_m-n, 'east_m': p.east_m-e, 'height_m': d-p.down_m,
                'velocity_north_m_s': v.north_m_s, 'velocity_east_m_s': v.east_m_s,
                'velocity_down_m_s': v.down_m_s,
                'speed_m_s': math.sqrt(sum(x*x for x in velocity))})
    except Exception as error:
        state['errors'].append(f'position stream: {error}')


async def collect_other(stream, state, key, convert):
    try:
        async for value in stream:
            state[key] = (time.monotonic(), {k: finite(v) for k, v in convert(value).items()})
    except Exception as error:
        state['errors'].append(f'{key} stream: {error}')
    finally:
        await stream.aclose()


async def write_rows(state):
    while True:
        state['writer'].writerow(row_now(state))
        state['file'].flush()
        state['rows'] += 1
        await asyncio.sleep(INTERVAL)


async def start_logging(drone, name, folder, started):
    """Open files and start collectors; return only after a ground position arrives."""
    folder = Path(folder)
    slug = name.lower().replace(' ', '_')
    handle = (folder/f'{slug}_flight.csv').open('w', newline='')
    detections = (folder/f'{slug}_detections.csv').open('w', newline='')
    writer = csv.DictWriter(handle, FIELDS)
    writer.writeheader()
    detection_writer = csv.DictWriter(detections, DETECTION_FIELDS)
    detection_writer.writeheader()
    state = dict(start=started, phase='ready', waypoint='', origin=None, previous=None,
                 distance=0.0, armed_at=None, takeoff_at=None, landed_at=None,
                 ready=asyncio.Event(), rows=0, detection_count=0, errors=[],
                 file=handle, writer=writer, detections=detections,
                 detection_writer=detection_writer, tasks=[], folder=folder, slug=slug)
    try:
        # Unsupported rate requests are recorded, but do not fabricate telemetry.
        for method, rate in ((drone.telemetry.set_rate_position_velocity_ned, 5),
                             (drone.telemetry.set_rate_battery, 1),
                             (drone.telemetry.set_rate_attitude_euler, 5)):
            try:
                await asyncio.wait_for(method(rate), 5)
            except Exception as error:
                state['errors'].append(f'rate request: {error}')
        state['tasks'] = [
            asyncio.create_task(collect_position(drone, state)),
            asyncio.create_task(collect_other(drone.telemetry.battery(), state, 'battery',
                lambda b: {'battery_fraction': b.remaining_percent, 'voltage_v': b.voltage_v})),
            asyncio.create_task(collect_other(drone.telemetry.attitude_euler(), state, 'attitude',
                lambda a: {'roll_deg': a.roll_deg, 'pitch_deg': a.pitch_deg, 'yaw_deg': a.yaw_deg})),
            asyncio.create_task(write_rows(state))]
        await asyncio.wait_for(state['ready'].wait(), 15)
        return state
    except BaseException:
        for task in state['tasks']:
            task.cancel()
        await asyncio.gather(*state['tasks'], return_exceptions=True)
        handle.close()
        detections.close()
        raise


def log_detection(state, result):
    """One row per attempted camera reading; errors stay distinct from negatives."""
    state['camera'] = (time.monotonic(), result)
    position = row_now(state)
    row = {key: position.get(key) for key in DETECTION_FIELDS}
    row.update(result)
    state['detection_writer'].writerow(row)
    state['detections'].flush()
    if result.get('red_detected') == 1:
        state['detection_count'] += 1


async def stop_logging(state, outcome):
    """Close streams/files even after errors; save a separate one-row summary."""
    if state is None:
        return
    for task in state['tasks']:
        task.cancel()
    results = await asyncio.gather(*state['tasks'], return_exceptions=True)
    state['errors'].extend(str(r) for r in results
                           if isinstance(r, Exception) and not isinstance(r, asyncio.CancelledError))
    try:
        state['writer'].writerow(row_now(state))
        state['rows'] += 1
    finally:
        state['file'].close()
        state['detections'].close()
    end = time.monotonic()
    landed, armed, takeoff = state['landed_at'], state['armed_at'], state['takeoff_at']
    summary = {'outcome': outcome, 'landing_confirmed': int(landed is not None),
               'run_elapsed_s': round(end-state['start'], 3),
               'mission_duration_s': round(landed-armed, 3) if landed and armed else None,
               'takeoff_to_disarm_s': round(landed-takeoff, 3) if landed and takeoff else None,
               'sampled_distance_m': round(state['distance'], 3),
               'telemetry_rows': state['rows'], 'positive_camera_checks': state['detection_count'],
               'logging_errors': '; '.join(state['errors'])}
    path = state['folder']/f'{state["slug"]}_summary.csv'
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, summary.keys())
        writer.writeheader()
        writer.writerow(summary)
    print(f'CSV saved: {state["folder"]} ({state["slug"]})', flush=True)
    print(f'Mission duration (arm attempt to disarm): {summary["mission_duration_s"]} s', flush=True)
