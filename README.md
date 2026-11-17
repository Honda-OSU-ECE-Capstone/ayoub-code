# Simple two-drone camera search + CSV

Your procedural `main` routine, coordinate calculations, `goto_location` loop, 10-meter altitude and timed orbit return remain. Both drones execute that routine. New code is marked ADDED; the camera and CSV helpers live in separate files so the flight loop stays readable.

## Added behavior

- A -> B -> C -> D -> E -> F -> G -> H follows a back-and-forth path. North/east coordinates are `(0,0), (10,0), (10,3), (0,3), (0,6), (10,6), (10,10), (0,10)`.
- Drone 2 mirrors east offsets and launches six meters west. Its planned route stays west of Drone 1's planned route. This is not active collision avoidance.
- Each straight leg has steps of at most 2 meters, a requested speed of 0.5 m/s and a minimum 3-second search hover. Arrival also waits for speed below 0.3 m/s.
- At each stop, a real image from that drone's Gazebo downward-facing camera is checked for a saturated red area. A positive result saves the image, adds a 2-second inspection hover, and then continues your route.
- This is a simple color detector, not AI object recognition. Any sufficiently large red area may trigger it. It does not estimate an object's location or detect/avoid obstacles.
- Camera failures after takeoff are printed and recorded as `error` with blank detection values. The route continues. Both cameras must pass an image check before arming.
- The return remains your timed orbit (radius 5 meters, speed 2 m/s, estimated half lap) followed by `goto_location` home and six seconds of waiting. It does not measure an exact semicircle or confirm arrival at home before sending LAND.

## Run on your Mac in VS Code

1. Extract `simple_sensing.zip`. Open the extracted `simple_sensing` folder with File -> Open Folder. Open all three VS Code terminals inside this folder.
2. Stop previous simulations only after their drones have landed. These scripts launch camera-equipped drones; the old plain x500 models do not provide the required images.
3. In a project terminal, install once:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

4. Terminal 1:

```bash
bash start_drone_1.sh
```

Wait for Gazebo and `x500_mono_cam_down_0`. Leave the terminal running. The script uses your existing `$HOME/PX4-Autopilot` checkout and its `.venv` if present. Use the existing working PX4 build environment if a build dependency is missing.

5. Terminal 2:

```bash
bash start_drone_2.sh
```

Wait for `x500_mono_cam_down_1`. Leave both terminals running. If a previous instance retained the wrong model, restart from a fresh default world with these scripts. Confirm two separated camera drones.

6. Terminal 3:

```bash
source .venv/bin/activate
python add_targets.py
python camera_sensor.py
```

Add the targets once per new world. The camera check must print TWO `Camera ... OK` messages. It never arms. If it fails, do not run the mission; send the error and the output of `GZ_IP=127.0.0.1 gz topic -l`.

The built-in `gz topic` command reads the images; ROS and Gazebo Python bindings are not required. Both Gazebo processes and the reader use `GZ_IP=127.0.0.1`. Gazebo must be running/unpaused, and its camera renderer must work. Official PX4 multi-vehicle Gazebo support is Linux; these Mac scripts follow the launch configuration that worked in your prior session, but the camera flight has not been tested on your Mac.

7. Fly in Terminal 3:

```bash
python simple.py
```

This starts flight directly, as your original script did. It checks both cameras first, starts both MAVSDK servers, then runs both drone routines. Do not run `main.py`, `run_local.py`, or another mission script alongside it.

Wait until both drones report landed and disarmed before closing PX4 or starting another run. First Ctrl+C enters landing cleanup; keep Gazebo running while it finishes. This is a simulation project, not a validated physical-drone controller.

## CSV results

Each run creates a NEW timestamped folder beside `simple.py`:

```
logs/20261117_123456_123456/
    drone_1_flight.csv
    drone_1_detections.csv
    drone_1_summary.csv
    drone_2_flight.csv
    drone_2_detections.csv
    drone_2_summary.csv
    Drone_1_...png
```

The exact folder path is printed when you run the code. Open any CSV in Excel.

### Flight CSV: about 5 rows per second

- `elapsed_s`: common wall-clock seconds since this run started. Use this as graph X axis.
- `mission_elapsed_s`: seconds since that drone's arm attempt.
- `north_m`, `east_m`, `height_m`: NED-derived position relative to that drone's ground launch point. Height is positive upward. These are separate local frames, not common world coordinates.
- `velocity_north_m_s`, `velocity_east_m_s`, `velocity_down_m_s`, `speed_m_s`: telemetry velocities and 3D speed.
- `battery_fraction`: fraction 0..1, not 0..100; `voltage_v`: battery voltage. Gazebo battery behavior depends on its simulated battery setup; it is not a physical energy measurement.
- `roll_deg`, `pitch_deg`, `yaw_deg`: attitude.
- `phase`, `waypoint`: labels for takeoff, travel, search, orbit, return, landing and landed.
- `position_age_s`, `battery_age_s`, `attitude_age_s`: time since last received sample. The logger records latest available values; columns are not guaranteed synchronized. Old/missing values are left empty, never replaced with zero.
- Camera columns show the last search result during the search phase. Use the detections CSV for one row per camera check.

### Detection CSV: one row per attempted search image

`red_detected=1` means a red area met the configured threshold; `0` means a valid image did not meet it. A blank detection with `camera_status=error` means sensing failed. `red_fraction` is the fraction of pixels classified red (not model confidence). `image_sim_time_s` is the Gazebo image timestamp when present. Each positive check can save a PNG; multiple detections may be the same object, not unique objects found.

### Summary CSV: one row per drone

- `mission_duration_s`: arm attempt to confirmed disarming, only populated if landing was confirmed.
- `takeoff_to_disarm_s`: takeoff command to confirmed disarming.
- `run_elapsed_s`: includes setup/waiting and cleanup up to that drone's logger closing.
- `sampled_distance_m`: sum of successive valid local-position displacements after arming, including vertical motion and measurement noise; only an approximation of traveled distance.
- `outcome`, `landing_confirmed`, `positive_camera_checks`, `logging_errors`.

All durations except image timestamps use wall-clock time. If Gazebo runs slowly, these differ from simulation seconds. No motor RPM/current/power is recorded by this version.

## Simple graphs in Excel

Use Insert -> Scatter (XY), not a category chart:

| Graph | X column | Y column |
|---|---|---|
| Speed over time | elapsed_s | speed_m_s |
| Altitude over time | elapsed_s | height_m |
| Battery over time | elapsed_s | battery_fraction |
| Top-down path | east_m | north_m |
| Red detection per stop | elapsed_s (detections file) | red_detected |

Plot the two flight files as separate series. Local paths overlap at their own origins; to compare their planned default-world placement, offset Drone 2 east by -6 meters and label that assumption.

## Settings

In `simple.py`: edit `waypoints`, `SEARCH_SPEED`, `SEARCH_STEP`, `SEARCH_PAUSE`, and `INSPECT_PAUSE`. Keep the final waypoint `(0,10)` while retaining the original orbit center `(0,5)`, and keep east offsets nonnegative for the mirrored separation.

In `camera_sensor.py`: `MIN_RED_FRACTION=0.005` means 0.5% of pixels. `MODELS` identifies each camera drone. `TOPICS` can override auto-discovery with exact per-drone topic strings.

## Verification

Run `python -m unittest -v test_additions.py` for software tests. Tests use synthetic images and fake telemetry, not a real Gazebo flight. No physical drone, Mac rendering pipeline, or end-to-end camera flight was tested here.

Primary references checked while implementing:
- Camera-equipped PX4 vehicles: https://docs.px4.io/main/en/sim_gazebo_gz/vehicles
- Gazebo Transport 13 CLI: https://github.com/gazebosim/gz-transport/blob/gz-transport13/src/cmd/topic_main.cc
- Gazebo image layout: https://github.com/gazebosim/gz-msgs/blob/gz-msgs10/proto/gz/msgs/image.proto
- PX4 downward camera model: https://github.com/PX4/PX4-gazebo-models/blob/main/models/x500_mono_cam_down/model.sdf
