# Foxglove debug GUI (`scripts/robot_viz.py`)

A Mac-side bridge: subscribes to the robot's ROS topics over SSH tunnels (same TCPROS approach as
`robot_watch.py`), converts them to Foxglove's ROS-style schemas, and serves them on a Foxglove WebSocket
server. Foxglove plays the role RViz would in a normal ROS setup.

```sh
.venv/bin/pip install foxglove-sdk                         # once
.venv/bin/python scripts/robot_viz.py                      # view only
.venv/bin/python scripts/robot_viz.py --drive              # + drive from Foxglove's Teleop panel (driving.md)
.venv/bin/python scripts/robot_viz.py --record run.mcap    # also record (MCAP = the rosbag2 format)
.venv/bin/python scripts/robot_viz.py --no-camera --cam-size 1280x960 --port 8765
```

## Connecting

Foxglove desktop app → **Open connection → Foxglove WebSocket → `ws://localhost:8765`**.
Not "Remote Access" (paid cloud feature) and not "ROS 1" (the robot's nodes only listen on localhost).

Panel settings live in the left sidebar's **Panel** tab with the panel selected — the **Topics** tab is just
a data browser. A topic row that's greyed out is hidden: click the eye icon at the right of the row.

Suggested layout:

| Panel | Setup |
|---|---|
| **3D** | Display frame `base_link` (robot-centred) or `map`; enable `/lidar`, `/robot/body`, `/robot/path`. `/lidar`: point size 4–5, Color mode *Color map*, Color by `range`; optional Decay time ~10 s in `map` to build a rough map while driving. Transforms → hide labels/axes to declutter. Leave `/robot/body` → Color empty (it would override the red/green states). |
| **Image** | topic `/camera` (see `camera.md`) |
| **Teleop** | only with `--drive`; settings in `driving.md` |
| **Plot** / **Raw Messages** | JSON topics don't appear in the 3D panel — plot fields, e.g. `/imu.yaw_deg`, `/odom/wheels.left_mmps`, `/robot/pose.diff_mm`, `/robot/state.battery`; `/drive/status` |
| **Service Call** | `/drive/estop`, `/drive/clear_estop` with request `{}` (with `--drive`) |

Save the layout from the layout dropdown (top right): Save as… / Export… to a JSON file.

## Topics served

| Topic | Foxglove schema | ROS equivalent | Content | Notes |
|---|---|---|---|---|
| `/lidar` | `PointCloud` | `sensor_msgs/PointCloud2` | current scan; x, y, z(=0), range in metres; frame `lidar` | `lidar.md` |
| `/tf` | `FrameTransforms` | `tf2_msgs/TFMessage` | `map → base_link`, `base_link → lidar` | |
| `/robot/path` | `PosesInFrame` | `nav_msgs/Path` | firmware's corrected pose trail, last ~600 poses in `map` | `imu_odometry.md` |
| `/robot/path_predict` | `PosesInFrame` | `nav_msgs/Path` | firmware's dead-reckoning trail (give it another colour) | `imu_odometry.md` |
| `/robot/pose` | JSON | — | corrected + dead-reckoning pose (mm/rad) and their difference `diff_mm` / `diff_theta`; 20 Hz | `imu_odometry.md` |
| `/imu` | JSON | — | raw `gyros[]` (`[0]` = integrated yaw rad), `yaw_deg`, derived `yaw_rate_dps`; ~42 Hz | `imu_odometry.md` |
| `/odom/wheels` | JSON | — | cumulative `left_mm`/`right_mm`, per-wheel `*_mmps`; ~42 Hz | `imu_odometry.md` |
| `/camera` | `CompressedVideo` (h264) | — | robot's hardware H.264, frame `camera`, ~30 fps | `camera.md` |
| `/robot/body` | `SceneUpdate` | `visualization_msgs/MarkerArray` | robot outline, heading arrow, bumpers, cliff sensors, wheel drops, LDS turret, IR values — red when triggered; frame `base_link`, 10 Hz | `sensors.md` |
| `/robot/state` | JSON | — | battery, charging, hazard, bump / cliff / wheel-drop, dirtbox / carpet / waterbox, motor currents (mA), IR ranges; 10 Hz | `sensors.md` |
| `/drive/status` | JSON | — | e-stop, hazard, commanded and wheel speeds (`--drive` only) | `driving.md` |
| `/cmd_vel` (from Foxglove) | `geometry_msgs/Twist` JSON | `geometry_msgs/Twist` | Teleop panel → bridge (`--drive` only) | `driving.md` |

## Frames

```
map ──(corrected pose from /prediction/PredictPose, mm→m, 20 Hz)──▶ base_link ──(identity, offset unknown)──▶ lidar
```

- `map` — the firmware's own world frame.
- `base_link` — robot body: **+x forward, +y left, θ CCW-positive** (REP-103), verified (`lidar.md`).
- `lidar` — same orientation as `base_link`; mounting offset not measured yet.

## Implementation notes

- **Timestamps:** robot stamps are seconds since boot; the bridge maps them onto the Mac clock with one
  fixed offset (from the first message), preserving relative timing. Camera frames are stamped on arrival
  at the Mac (~100 ms behind) — fine for viewing, not for precise lidar/camera fusion.
- **Bandwidth:** `/robot/Robot` is ~50 Hz × ~20 KB (≈ 1 MB/s, it embeds the lidar scan). The bridge
  subscribes once and shares it between the display (throttled to 10 Hz) and the `--drive` safety checks.
  Lidar ~115 KB/s; camera ~150 KB/s at 640x480.
- **Reconnects:** each robot topic runs in its own thread and resubscribes after 2 s on failure; the
  camera pipeline restarts likewise. SSH tunnels are created one at a time under a lock and rebuilt if a
  connection through them fails (concurrent creation used to leave a random topic stuck on
  "Connection refused").
- **Shutdown** (Ctrl-C or SIGTERM): wheel zeros + unregister (`--drive`) first, then closes the MCAP, stops
  the server, kills SSH tunnels, under a 3 s hard deadline; a second Ctrl-C exits immediately. The camera's
  robot-side watchdog frees `/dev/video0` within ~1 s. `kill -USR1 <pid>` dumps all thread stacks.
- **Lidar stopping:** the firmware sometimes stops the LDS (cause unknown — idle timer?). `/lidar` then goes
  silent; publishing `lds/ControlLds` `type: 0` (START) on `/lds/ControlLds` restarts it within ~2 s.
