"""Read ONE real Gazebo image at a search stop; detect a red area locally.

No ROS, cloud, LLM, or webcam. Run `python camera_sensor.py` before flight.
This is a color threshold, not object recognition or obstacle avoidance.
"""
import asyncio
import base64
import json
import os
from pathlib import Path
import cv2
import numpy as np

MODELS = {14540: 'x500_mono_cam_down_0', 14541: 'x500_mono_cam_down_1'}
# Set an exact topic here only if your Gazebo model uses a different image topic.
TOPICS = {14540: '', 14541: ''}
MIN_RED_FRACTION = 0.005   # At least 0.5% of the image must be red.
CAMERA_TIMEOUT = 8


async def gz_command(*args, timeout=CAMERA_TIMEOUT):
    """Bound command duration and terminate its child if canceled/timed out."""
    env = dict(os.environ, GZ_IP='127.0.0.1')
    proc = await asyncio.create_subprocess_exec(
        'gz', *args, stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE, env=env)
    try:
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout)
    finally:
        if proc.returncode is None:
            proc.kill()
            await proc.wait()
    if proc.returncode:
        raise RuntimeError(stderr.decode(errors='replace')[-600:] or 'gz command failed')
    return stdout.decode()


async def find_camera_topic(udp_port):
    if TOPICS[udp_port]:
        return TOPICS[udp_port]
    topics = (await gz_command('topic', '-l')).splitlines()
    candidates = [t.strip() for t in topics
                  if f'/model/{MODELS[udp_port]}/' in t
                  and t.strip().endswith('/image')]
    if len(candidates) != 1:
        raise RuntimeError(f'Expected one image topic for {MODELS[udp_port]}; found {candidates}. '
                           'Start the camera drones, then inspect: GZ_IP=127.0.0.1 gz topic -l')
    TOPICS[udp_port] = candidates[0]
    return candidates[0]


def decode_image(message):
    """Gazebo JSON stores raw pixel bytes in base64, not a compressed JPEG."""
    width, height = int(message['width']), int(message['height'])
    fmt = message.get('pixelFormatType', message.get('pixel_format_type'))
    formats = {3: 'RGB_INT8', 8: 'BGR_INT8', 4: 'RGBA_INT8', 5: 'BGRA_INT8'}
    fmt = formats.get(fmt, fmt)
    if fmt not in ('RGB_INT8', 'BGR_INT8', 'RGBA_INT8', 'BGRA_INT8'):
        raise ValueError(f'Unsupported camera format: {fmt}; use an RGB camera')
    channels = 4 if 'A_' in fmt else 3
    step = int(message.get('step', width * channels))
    if not 0 < width <= 4096 or not 0 < height <= 4096 or step < width * channels:
        raise ValueError('Invalid image dimensions/stride')
    data = base64.b64decode(message['data'], validate=True)
    if len(data) != height * step:
        raise ValueError('Image data length does not match its dimensions')
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :width * channels].reshape(height, width, channels)
    if fmt.startswith('RGB'):
        return pixels[:, :, :3][:, :, ::-1].copy()  # OpenCV expects BGR.
    return pixels[:, :, :3].copy()


def detect_red(image):
    """Detect saturated red pixels, covering both ends of the HSV hue range."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
    high = cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    fraction = cv2.countNonZero(mask) / mask.size
    return fraction >= MIN_RED_FRACTION, float(fraction)


async def sense_camera(udp_port, save_path=None):
    """Return a detection only after receiving and decoding an actual image."""
    topic = await find_camera_topic(udp_port)
    output = await gz_command('topic', '-e', '-t', topic, '-n', '1', '--json-output')
    message = json.loads(output)
    image = decode_image(message)
    detected, fraction = detect_red(image)
    stamp = message.get('header', {}).get('stamp', {})
    sim_time = (float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) / 1e9
                if stamp else None)
    snapshot = ''
    if save_path is not None and detected:
        path = Path(save_path)
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f'Could not save camera image to {path}')
        snapshot = path.name
    return {'camera_status': 'ok', 'red_detected': int(detected),
            'red_fraction': fraction, 'image_sim_time_s': sim_time,
            'camera_topic': topic, 'snapshot': snapshot, 'camera_error': ''}


async def check_cameras():
    # Check both cameras before either drone is armed. No flight commands here.
    for port in MODELS:
        result = await sense_camera(port)
        print(f'Camera {port} OK: {result["camera_topic"]}', flush=True)


if __name__ == '__main__':
    asyncio.run(check_cameras())
