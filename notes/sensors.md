# Proximity & safety sensors

Bumpers, cliff sensors, wheel drops, wall/front IR, plus where to read them. Visualised by
`scripts/robot_viz.py` as `/robot/body` (3D markers, red when triggered) and `/robot/state` (JSON) —
see `foxglove.md`. Lidar is in `lidar.md`, camera in `camera.md`.

## Where the data comes from

All of these are in **`/robot/Robot`** (~50 Hz, see `ros/robot_robot_topic.md` for the full layout).
The per-sensor topics exist but are too slow to react to a bump:

| Topic | Rate (idle) | Content |
|---|---|---|
| `/robot/Robot` | ~50 Hz, ~20 KB | everything below + pose + battery + lidar scan |
| `/onOffInfo/OnOffInfo` | ~1.4 Hz | bump / downin / fall / dirtbox / carpet / mop flags |
| `/rangeDet/RangeDetect` | ~42 Hz | front IR, side IR, raw down_in, ultrasound |
| `/motor/MotorCurrent` | ~1.4 Hz | per-motor current (mA) |
| `/imu/ImuSensor` | ~42 Hz | gyro / accel arrays |
| `/power/Battery`, `/power/ChargeState`, `/task/WorkState`, `/alert/Alerts` | silent when idle | |

Positions below are in `base_link` (x forward, y left), measured from an underside photo with the body
Ø ≈ 350 mm — approximate to ~1 cm. The firmware reports sensors by index only.

## Cliff sensors (`downin`) — 5 fitted

Mapped 2026-11-10 by covering each underside window and watching the flags. Names are the
`onOffInfo/DownInValue` enum. (A bottom view is mirrored — left/right here are the robot's own.)

| Index | Firmware name | Position | x, y (m) |
|---|---|---|---|
| c0 | `TYPE_DOWNIN_LEFT` | left side, outside left wheel | +0.022, +0.158 |
| c1 | `TYPE_DOWNIN_FRONT` | front — **both** windows either side of the caster are this one sensor (emitter + receiver) | +0.132, 0 |
| c2 | `TYPE_DOWNIN_FRONT_RIGHT` | **not fitted on this model** — always 0 on the floor, lifted and upside down; not drawn | — |
| c3 | `TYPE_DOWNIN_RIGHT` | right side, outside right wheel | +0.022, −0.158 |
| c4 | `TYPE_DOWNIN_LEFT_BACK` | back-left | −0.074, +0.144 |
| c5 | `TYPE_DOWNIN_RIGHT_BACK` | back-right | −0.074, −0.144 |

- Semantics: **`1` = no floor seen** (cliff, lifted), `0` = floor present.
- Range is very short: they respond to an object within ~1 cm; a hand 2–3 cm away isn't detected.
  To test c1 you must cover both caster windows at once.
- The raw `rangeDet` `down_in` array has 5 values (one per fitted sensor) but reads all zeros in
  normal operation.
- Tool used: a read-only robot-side monitor that prints whenever a cliff / drop / IR value changes
  (subscribe `/robot/Robot`, compare `downin`, `fall`, `rangeDets`).

## Bumpers (`bump`, `onOffInfo/BumpValue`)

| Type | Name | Notes |
|---|---|---|
| 0 | `TYPE_BUMP_LEFT` | front-left half of the bumper — verified |
| 1 | `TYPE_BUMP_RIGHT` | front-right half — verified |
| 2 | `TYPE_BUMP_LDS` | the lidar turret's own bump switch |

`/onOffInfo` bump value is a bitmask (bit0 left, bit1 right) — both pressed when docked.

## Wheel drops (`fall`, `onOffInfo/FallValue`)

`[left, right]`, `1` = wheel extended (robot lifted). Verified by lifting. Drawn at the wheels (y ≈ ±0.12 m).
Note the wheels retract when the robot is upside down, so they read `0` then.

## IR range sensors (`rangeDet`)

| Type | Name | Values | Notes |
|---|---|---|---|
| 0 | `RANGE_TYPE_FRONT_BUFFER` | 3 | flickers 0–3 constantly at rest; meaning not yet characterised |
| 1 | `RANGE_TYPE_SIDE_IN` | 2 = `[left, right]` | **wall sensors**: 0 with nothing near, rising to ~3800 when touching; good for wall-following. Shown as "wall IR L/R" |
| 2 | `RANGE_TYPE_DOWN_IN` | 5 | raw cliff values, all 0 in normal operation |
| 3 | `RANGE_TYPE_ULTRASOUND` | 1 | 0 so far (probably carpet detection) |

## Safety use

`robot_teleop.py` and `robot_viz.py --drive` latch an e-stop when a bump, wheel drop or cliff flag
*appears* (rising edge), and stop while `/robot/Robot` is stale (> 1 s). Direct wheel commands bypass the
firmware's own reactions, so this is the only protection on that path — see `driving.md`.
