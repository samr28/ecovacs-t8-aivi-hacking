# IMU, wheel odometry & the firmware's pose estimates
Live in Foxglove as `/imu`, `/odom/wheels`, `/robot/pose`, `/robot/path`, `/robot/path_predict` (`foxglove.md`).

## `/imu/ImuSensor` (~42 Hz, header stamp = seconds since boot)

```
imu/ImuSensor   md5 4c90e05745be1ab9370dcc4b7fb0cb58
Header header
float32[] gyros      # 4 values — see below
float32[] accs       # EMPTY on this robot — no accelerometer data is published
```

| Index | Meaning | Evidence |
|---|---|---|
| `gyros[0]` | **Integrated yaw / heading, rad, CCW-positive, wrapped to ±π** — an angle, *not* a rate | spin left +356.1°, spin right −96.6°; identical to `predictPose.theta` at every sample |
| `gyros[1]` | small tilt angle, rad (≤ 0.036 ≈ 2°) — **probably roll or pitch** | varies sinusoidally with heading during a spin, 90° out of phase with `[2]` — what a ~2° floor/mount tilt looks like as the robot turns |
| `gyros[2]` | the other tilt angle, rad (−0.048 … +0.006) | as above |
| `gyros[3]` | always exactly `−gyros[0]` | redundant |

- Scale confirmed by eye: the spin-left run was one full turn and the spin-right run a quarter turn.
- No drift was visible at rest over several seconds.
- Yaw rate isn't published; `robot_viz.py` derives `yaw_rate_dps` from successive samples (noisy, peaks
  ~±35 °/s while the true spin rate was ~24 °/s = 6.21 rad / 15 s).
- Which of `[1]`/`[2]` is roll vs pitch is not yet determined (tilt the robot nose-up vs sideways to check).
- `/imu/ImuState` (`gyroState`, `accState`) and `/imu/InitIMU` (calibration: GYRO / ROLL_PITCH_YAW /
  GYRO_AND_ACC) exist; ImuState doesn't publish at idle.

## `/wheel/WheelDistanceReport` (~42 Hz)

`float32[] wheelDistance = [left, right]` — **cumulative wheel travel in mm** since boot (never resets in
normal use). Forward 10 s: +570 mm / +567 mm; spin left: −708 / +711 mm.

## Firmware pose estimates (`/prediction/PredictPose`, ~80–110 Hz; also embedded in `/lds/Lds` and `/robot/Robot`)

| Field | What it is |
|---|---|
| `predictPose` | dead reckoning: **heading = IMU `gyros[0]` exactly**, position from wheel distances |
| `pose` | the SLAM-corrected estimate (what the rest of the firmware uses) |

Both in mm / rad in the firmware's `map` frame. During the test the corrected heading differed from the IMU
by ~2.5° after a full turn; forward, corrected position ≈ 587 mm vs wheels 569 mm. Over a long session the
two drift apart (527 mm / 28° after an evening of being driven and carried) — see `/robot/pose.diff_mm`.
