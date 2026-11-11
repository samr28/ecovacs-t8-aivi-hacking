# Lidar (LDS)

## Live view in Foxglove

```sh
.venv/bin/pip install foxglove-sdk            # once
.venv/bin/python scripts/robot_viz.py         # bridge: robot topics → Foxglove WebSocket :8765
.venv/bin/python scripts/robot_viz.py --record run.mcap   # also record (MCAP = rosbag2 format)
```

In the Foxglove desktop app: **Open connection → Foxglove WebSocket → `ws://localhost:8765`** (not
"Remote Access" — that's the paid cloud feature; not "ROS 1" — the robot's nodes are localhost-only).
Add a **3D** panel, then in its *Panel* settings (not the *Topics* data-browser tab):

- Frame → Display frame: `base_link` (robot-centred) or `map` (world)
- Topics → `/lidar`: make visible, Point size 4–5, Color mode *Color map*, Color by `range`
- Optional: `/lidar` Decay time ~10 s with Display frame `map` → scans accumulate into a rough map while
  driving with `scripts/robot_teleop.py`; `/robot/path` shows the trail
- Transforms → turn labels off to declutter

Bridge output:

| Topic | Foxglove schema | ROS equivalent | Content |
|---|---|---|---|
| `/lidar` | `foxglove.PointCloud` | `sensor_msgs/PointCloud2` | current scan, metres, fields x, y, z(=0), range; frame `lidar` |
| `/tf` | `foxglove.FrameTransforms` | `tf2_msgs/TFMessage` | `map → base_link` (robot pose), `base_link → lidar` (identity) |
| `/robot/path` | `foxglove.PosesInFrame` | `nav_msgs/Path` | last ~600 poses (~2 min) in `map` |

Why a point cloud and not `LaserScan`: LaserScan needs evenly spaced beams (`angle_min + i·angle_increment`);
the robot's points have irregular angles, so they're sent exactly as measured.

Robot timestamps are seconds since boot; the bridge maps them onto the Mac clock with one fixed offset
(from the first message), preserving relative timing.

## Robot topic: `/lds/Lds`

```
lds/Lds   md5 ba4104feb5e50b9c15d3e29666224d02     publisher /node (also subscribed by /node)
Header header                      # frame_id is empty; stamp = seconds since boot
LdsPoint[] points                  # float32 x, y, rho, theta, power
prediction/PredictPose ldsPose     # Pose predictPose (dead-reckoning), Pose pose (corrected) — each Header + x, y, theta
```

Measured (robot idle off-dock):

- **5.5 Hz**, ~1050 points per scan, ~21 KB per message (~115 KB/s — fine over Wi-Fi/ssh).
- Publishes continuously, even when no task is running (also on the dock — see `robot_robot_topic.md`).
- `x, y` in **mm**, robot/sensor frame; `x = rho·cos(theta)`, `y = rho·sin(theta)` exactly.
- `rho` in mm (0.49–5.3 m seen indoors); `theta` in rad, −π…π, not evenly spaced.
- `power` is always 1000 → no usable intensity.
- `ldsPose.pose` = robot pose at scan time, **mm / rad**, in the firmware's map frame. The bridge uses it
  for `map → base_link`.
- Raw decode offsets: `Header` = 16 bytes + frame_id string; then `uint32 n` + `n × 20` bytes of points;
  then two Poses (each Header + 3 × float32).

## Frames & orientation (verified 2026-11-10)

- **+x = forward, +y = left, θ CCW-positive** (standard ROS REP-103). Checked with an object placed
  behind the robot (appeared at −x) and on its left side (appeared at +y). Matches the wheel tests in
  `driving.md` (spin left → θ increases).
- Lidar mounting offset relative to the robot centre: **unknown**, treated as identity. A measured
  offset would go in the `base_link → lidar` transform in `robot_viz.py`.

## Related topics

| Topic | Notes |
|---|---|
| `/lds/ControlLds` | `lds/ControlLds` — START=0, STOP=1, RESET_POSE=2 (the task manager calls `controlLds(1)` before remote control) |
| `/lds/SendData`, `/lds/RecvData` | raw serial to/from the LDS module |
| `/robot/Robot` | also embeds the latest scan (`ldsPic`) plus pose and all other sensors |
