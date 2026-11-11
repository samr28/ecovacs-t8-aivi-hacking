# Lidar (LDS)

View it live with the Foxglove bridge (`scripts/robot_viz.py`, topic `/lidar`) — setup and panel
settings in `foxglove.md`.

## Robot topic: `/lds/Lds`

```
lds/Lds   md5 ba4104feb5e50b9c15d3e29666224d02     publisher /node (also subscribed by /node)
Header header                      # frame_id is empty; stamp = seconds since boot
LdsPoint[] points                  # float32 x, y, rho, theta, power
prediction/PredictPose ldsPose     # Pose predictPose (dead-reckoning), Pose pose (corrected) — each Header + x, y, theta
```

Measured (robot idle off-dock):

- **5.5 Hz**, ~1050 points per scan, ~21 KB per message (~115 KB/s — fine over Wi-Fi/ssh).
- Publishes continuously, even when no task is running (also on the dock — see `ros/robot_robot_topic.md`).
- `x, y` in **mm**, robot/sensor frame; `x = rho·cos(theta)`, `y = rho·sin(theta)` exactly.
- `rho` in mm (0.49–5.3 m seen indoors); `theta` in rad, −π…π, not evenly spaced.
- `power` is always 1000 → no usable intensity.
- `ldsPose.pose` = robot pose at scan time, **mm / rad**, in the firmware's map frame. The bridge uses it
  for `map → base_link`. `ldsPose.predictPose` is the firmware's dead-reckoning estimate.
- Raw decode offsets: `Header` = 16 bytes + frame_id string; then `uint32 n` + `n × 20` bytes of points;
  then two Poses (each Header + 3 × float32).

In Foxglove it's sent as a `PointCloud` (ROS: `sensor_msgs/PointCloud2`) in metres with a `range` field,
not as a `LaserScan`: LaserScan needs evenly spaced beams (`angle_min + i·angle_increment`), and these
angles aren't, so the points are sent exactly as measured.

## Frames & orientation (verified 2026-11-10)

- **+x = forward, +y = left, θ CCW-positive** (standard ROS REP-103). Checked with an object placed
  behind the robot (appeared at −x) and on its left side (appeared at +y). Matches the wheel tests in
  `driving.md` (spin left → θ increases).
- Mounting offset relative to the robot centre: **unknown**, treated as identity. A measured
  offset would go in the `base_link → lidar` transform in `robot_viz.py`.

## Related topics

| Topic | Notes |
|---|---|
| `/lds/ControlLds` | `lds/ControlLds` — START=0, STOP=1, RESET_POSE=2 (the task manager calls `controlLds(1)` before remote control) |
| `/lds/SendData`, `/lds/RecvData` | raw serial to/from the LDS module |
| `/robot/Robot` | also embeds the latest scan (`ldsPic`) plus pose and all other sensors |
| `onOffInfo` bump type 2 | `TYPE_BUMP_LDS` — the turret's bump switch (`sensors.md`) |
