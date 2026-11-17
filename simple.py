import math, time, asyncio
# ADDED: only used to start both local servers before Python opens gRPC channels.
import os, sys, socket, subprocess
from pathlib import Path
from datetime import datetime
from camera_sensor import check_cameras, sense_camera
from flight_csv import start_logging, stop_logging, log_detection
import mavsdk_grpc
from mavsdk_grpc import System
from mavsdk_grpc.action import OrbitYawBehavior

# waypoints - Name, Meters north of start, Meters east of start
waypoints = [
    ("A", 0, 0),
    ("B", 10, 0),
    ("C", 10, 3),
    ("D", 0, 3),
    ("E", 0, 6),
    ("F", 10, 6),
    ("G", 10, 10),
    ("H", 0, 10)  # Keep the original orbit entry point: north=0, east=10.
]

ALT = 10.0

# ADDED: settings for the extra stops along your original straight lines.
SEARCH_SPEED = 0.5       # Requested straight-line speed, meters per second.
SEARCH_STEP = 2.0        # At most two meters between search stops.
SEARCH_PAUSE = 3.0       # Minimum hover time while checking the camera.
INSPECT_PAUSE = 2.0      # Extra hover if a red area is detected; then continue.
RUN_FOLDER = None       # Created once when the program starts.
RUN_STARTED = None
EARTH_RADIUS = 6378137.0  # earth's radius in meters


# turns a starting coordinate and number of meters north and east into a second set of coordinates
def meters_to_coords(lat, lon, meters_north, meters_east):
    lat_rad = math.radians(lat)
    dLat = meters_north / EARTH_RADIUS
    dLon = meters_east / (EARTH_RADIUS * math.cos(lat_rad))
    return lat + math.degrees(dLat), lon + math.degrees(dLon)


# turns two sets of coordinates into the distance in meters
def coords_to_meters(lat1, lat2, lon1, lon2, alt1, alt2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    lon1_rad = math.radians(lon1)
    lon2_rad = math.radians(lon2)

    # haversine formula to calculate distance horizontally
    dLat = lat2_rad - lat1_rad
    dLon = lon2_rad - lon1_rad

    a = (math.sin(dLat / 2) ** 2
         + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dLon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # distance horizontally
    horizontal_distance = EARTH_RADIUS * c

    # distance vertically
    vertical_distance = alt2 - alt1

    return math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

def find_circle_through_two_points(p1, p2, diameter):
    # Calculate the midpoint between the two points
    mid_x = (p1[0] + p2[0]) / 2
    mid_y = (p1[1] + p2[1]) / 2

    # Calculate the distance between the two points
    dist = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    # Check if the distance is greater than the diameter
    if dist > diameter:
        raise ValueError("The distance between the points is greater than the diameter.")

    # Calculate the radius of the circle
    radius = diameter / 2

    # Calculate the distance from the midpoint to the center of the circle
    h = math.sqrt(radius ** 2 - (dist / 2) ** 2)

    # Calculate the direction vector from p1 to p2
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    # Normalize the direction vector
    length = math.sqrt(dx ** 2 + dy ** 2)
    dx /= length
    dy /= length

    # Calculate the perpendicular vector
    perp_dx = -dy
    perp_dy = dx

    # Calculate the center of the circle
    center_x = mid_x + h * perp_dx
    center_y = mid_y + h * perp_dy

    return (center_x, center_y), radius

# ADDED arguments: run your same routine once for each drone.
# east_sign=1 uses your route; east_sign=-1 mirrors it east/west.
async def main(name, udp_port, grpc_port, east_sign):
    drone = System(mavsdk_server_address="127.0.0.1", port=grpc_port)
    arm_attempted = False
    log_state = None
    outcome = "incomplete"

    try:
        print(f"[{name}] Connecting to drone")
        await asyncio.wait_for(drone.connect(system_address=f"udpin://127.0.0.1:{udp_port}"), 45)

        # if connection state changes, print connected and break loop
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("drone connected")
                break
            await asyncio.sleep(1)

        print("Waiting for position checks")

        # Do global and home position checks
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Position checks passed!")
                break

        # ADDED: telemetry logging runs in the background; your waypoint loop
        # still reads position and uses the same goto_location calculations.
        async for armed in drone.telemetry.armed():
            if armed:
                raise RuntimeError("Start with this drone landed and disarmed")
            break
        async for flying in drone.telemetry.in_air():
            if flying:
                raise RuntimeError("Start with this drone landed and disarmed")
            break
        log_state = await start_logging(drone, name, RUN_FOLDER, RUN_STARTED)

        await drone.action.set_takeoff_altitude(ALT)

        print("Arming...")
        arm_attempted = True  # Record before the command in case its reply is lost.
        log_state["armed_at"] = time.monotonic()
        log_state["phase"] = "takeoff"
        await drone.action.arm()

        print(f"Taking off to {ALT} meters...")
        log_state["takeoff_at"] = time.monotonic()
        await drone.action.takeoff()

        await asyncio.sleep(12)  # wait for the drone to reach takeoff altitude

        print("Takeoff command sent.")

        # Reference point for ALL waypoints: latitude, longitude, absolute altitude
        startCoords = []

        async for pos in drone.telemetry.position():
            startCoords = [
                pos.latitude_deg,
                pos.longitude_deg,
                pos.absolute_altitude_m
            ]
            break

        # ADDED: create small waypoints ON each of your existing straight lines.
        # Your original for-loop below still uses goto_location and the same
        # meters_to_coords / coords_to_meters calculations.
        search_waypoints = []
        previous_north, previous_east = 0.0, 0.0

        for point_name, north_m, east_m in waypoints:
            east_m = east_m * east_sign
            line_distance = math.hypot(north_m - previous_north, east_m - previous_east)
            steps = max(1, math.ceil(line_distance / SEARCH_STEP))

            for step in range(1, steps + 1):
                part = step / steps
                small_north = previous_north + (north_m - previous_north) * part
                small_east = previous_east + (east_m - previous_east) * part
                stop_name = point_name if step == steps else f"{point_name} search {step}"
                search_waypoints.append((stop_name, small_north, small_east))

            previous_north, previous_east = north_m, east_m

        for waypoint in search_waypoints:
            waypoint_name, north_m, east_m = waypoint
            log_state["waypoint"] = waypoint_name
            log_state["phase"] = "travel"
            log_state["camera"] = None

            # FIX: offsets are measured from the START point, not the drone's current position
            newCoords = meters_to_coords(
                startCoords[0],
                startCoords[1],
                north_m,
                east_m
            )

            print(f"[{name}] Flying to waypoint {waypoint_name}")

            await drone.action.goto_location(
                newCoords[0],
                newCoords[1],
                startCoords[2],  # hold the altitude captured after takeoff
                0
            )

            # ADDED: request the slower speed after entering goto mode.
            # An unsupported/rejected speed command stops this drone's mission.
            await drone.action.set_current_speed(SEARCH_SPEED)

            while True:
                async for pos in drone.telemetry.position():
                    distance = coords_to_meters(
                        pos.latitude_deg,
                        newCoords[0],
                        pos.longitude_deg,
                        newCoords[1],
                        pos.absolute_altitude_m,
                        startCoords[2]
                    )
                    break

                if distance < 1.0:  # within 1 meter
                    # ADDED: wait for low speed so the search pause is a stop,
                    # rather than announcing arrival while still flying past.
                    async for local_pos in drone.telemetry.position_velocity_ned():
                        velocity = local_pos.velocity
                        speed = math.sqrt(velocity.north_m_s ** 2
                                          + velocity.east_m_s ** 2
                                          + velocity.down_m_s ** 2)
                        print(
                            f"[{name}] Current Axis Coordinates:",
                            local_pos.position.north_m,
                            local_pos.position.east_m,
                            local_pos.position.down_m
                        )
                        break
                    if speed < 0.3:
                        print(f"[{name}] Waypoint reached! Searching for {SEARCH_PAUSE} seconds...")
                        # ADDED: read a fresh Gazebo camera image during the pause.
                        # A red area causes a short extra inspection hover, then
                        # the same original waypoint loop continues.
                        log_state["phase"] = "search"
                        snapshot = RUN_FOLDER / f"{name.replace(' ', '_')}_{time.time_ns()}.png"
                        camera_result, _ = await asyncio.gather(
                            sense_camera(udp_port, snapshot),
                            asyncio.sleep(SEARCH_PAUSE), return_exceptions=True)
                        if isinstance(camera_result, asyncio.CancelledError):
                            raise camera_result
                        if isinstance(camera_result, Exception):
                            camera_result = {"camera_status": "error", "red_detected": None,
                                             "red_fraction": None, "image_sim_time_s": None,
                                             "camera_topic": "", "snapshot": "",
                                             "camera_error": str(camera_result)}
                            print(f"[{name}] Camera unavailable: {camera_result['camera_error']}")
                        log_detection(log_state, camera_result)
                        if camera_result.get("red_detected") == 1:
                            print(f"[{name}] Red area detected; inspecting for {INSPECT_PAUSE} seconds.")
                            await asyncio.sleep(INSPECT_PAUSE)
                        break

                await asyncio.sleep(0.5)


        log_state["phase"] = "orbit"
        log_state["waypoint"] = "orbit return"
        center_lat, center_lon = meters_to_coords(startCoords[0], startCoords[1], 0, 5 * east_sign)

        await drone.action.do_orbit(
            5.0 * east_sign,                              # ADDED: signed radius mirrors direction
            2.0,                                          # velocity_ms
            OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER, # yaw_behavior
            center_lat,
            center_lon,
            startCoords[2]                                # absolute_altitude_m
        )

        # Your original timed return is kept. This estimates a half lap;
        # orbit entry/acceleration mean it is not a measured exact semicircle.
        await asyncio.sleep(math.pi * 5.0 / 2.0)  # half lap, about 7.9 s

        log_state["phase"] = "return"
        log_state["waypoint"] = "Home"
        await drone.action.goto_location(startCoords[0], startCoords[1], startCoords[2], 0)
        await asyncio.sleep(3)
        await asyncio.sleep(3)
        outcome = "route_completed"

    except Exception as e:
        outcome = f"error: {e}"
        print(f"[{name}] Drone operation failed: {e}")
        raise

    finally:
        try:
            # ADDED cleanup: each routine lands only the drone it tried to arm.
            # Keep the servers alive until PX4 confirms landing and disarming.
            if arm_attempted:
                if log_state is not None:
                    log_state["phase"] = "landing"
                print(f"[{name}] Landing...")
                await asyncio.wait_for(drone.action.land(), 15)
    
                async def wait_landed():
                    async for in_air in drone.telemetry.in_air():
                        if not in_air:
                            break
                    async for armed in drone.telemetry.armed():
                        if not armed:
                            return
    
                await asyncio.wait_for(wait_landed(), 150)
                if log_state is not None:
                    log_state["landed_at"] = time.monotonic()
                    log_state["phase"] = "landed"
                if outcome == "route_completed":
                    outcome = "completed"
                print(f"[{name}] Landed and disarmed.")
        finally:
            # Always close CSV files, including after an error or first Ctrl+C.
            await stop_logging(log_state, outcome)

    return


# ADDED: run the SAME main routine twice. Each drone has its own startCoords.
async def run_both():
    results = await asyncio.gather(
        main("Drone 1", 14540, 50051, 1),
        main("Drone 2", 14541, 50052, -1),
        return_exceptions=True
    )
    # Let both routines finish their landing before reporting any failure.
    errors = [result for result in results if isinstance(result, BaseException)]
    if errors:
        raise RuntimeError("One or both drone routines failed: " + "; ".join(map(str, errors)))


if __name__ == "__main__":
    # ADDED connection setup for your Mac: start BOTH servers before creating
    # Python gRPC connections. This retains the fix that worked in run_local.py.
    # This file flies immediately, just like your original simple.py.
    servers = []
    try:
        # No arming takes place until both real Gazebo cameras return an image.
        if SEARCH_STEP <= 0 or SEARCH_SPEED <= 0 or SEARCH_PAUSE < 0:
            raise ValueError("Use positive search speed/spacing and a nonnegative pause")
        RUN_STARTED = time.monotonic()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        RUN_FOLDER = Path(__file__).resolve().parent / "logs" / stamp
        RUN_FOLDER.mkdir(parents=True)
        print(f"Results folder: {RUN_FOLDER}", flush=True)
        asyncio.run(check_cameras())

        binary = Path(mavsdk_grpc.__file__).parent / "bin" / "mavsdk_server"
        if not binary.is_file():
            raise RuntimeError(f"MAVSDK server is missing: {binary}")

        # Check actual TCP listeners, not TIME_WAIT from a previous run.
        for udp, grpc in ((14540, 50051), (14541, 50052)):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("127.0.0.1", udp))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.settimeout(1)
                try:
                    probe.connect(("127.0.0.1", grpc))
                except ConnectionRefusedError:
                    pass
                else:
                    raise RuntimeError(f"TCP {grpc} is already in use; close other mission scripts")

        for key in ("NO_PROXY", "no_proxy", "no_grpc_proxy"):
            os.environ[key] = os.environ.get(key, "") + ",localhost,127.0.0.1,::1"

        for udp, grpc in ((14540, 50051), (14541, 50052)):
            servers.append(subprocess.Popen(
                [str(binary), "-p", str(grpc), "--sysid", "245", "--compid", "190",
                 f"udpin://127.0.0.1:{udp}"],
                start_new_session=True  # First Ctrl+C lets Python finish LAND.
            ))

        asyncio.run(run_both())
    except KeyboardInterrupt:
        print("Execution stopped by user. Check both landing messages.")
        sys.exit(130)
    except Exception as e:
        print(f"Stopped: {e}")
        print("If landing was not confirmed, use QGroundControl LAND or commander land in each PX4 shell.")
        sys.exit(1)
    finally:
        # Stop only the servers this file started. PX4 and Gazebo stay running.
        for process in servers:
            if process.poll() is None:
                process.terminate()
        for process in servers:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
