# Driving the Robot

Worked out 2026-11-10 by tracing ROS topics, disassembling `liberos_node_task_manager.so`,
`liberos_node_hardware_platform.so` and medusa's `plug_medusa_plugin_ros_node.so`, and floor tests.

**TL;DR:** publish `wheel/SetWheelSpeed` straight to `/node` at ~20 Hz. `scripts/robot_teleop.py` does this
from the Mac (WASD/arrows) with its own bump/drop/cliff e-stop.

---

## What works: `/wheel/SetWheelSpeed` (direct)

```
wheel/SetWheelSpeed   md5 6136b114420b8e6600151ac57541e32e
uint8 SPEED_TYPE_DRIVING = 0
uint8 SPEED_TYPE_PHYSICAL = 1
uint8 speed_type
float32[] speed        # [left, right]
```

- Subscriber: `/node` → `common::Wheel::onSetWheelSpeedMsgReceived` (hardware_platform plugin). It optionally
  converts PHYSICAL→DRIVING, packs a 19-byte frame and sends it to the motor MCU. **No task-state gating.**
- Because the task manager is bypassed, the firmware's own bump/cliff reactions do not apply. Do your own
  safety: watch `/robot/Robot` bump / fall / downin and send zeros.
- Wire format: `struct.pack('<BI2f', 0, 2, left, right)` (speed_type, array length, two float32).
- Send continuously (20 Hz). Ends with explicit zeros.
- Whether the MCU stops by itself when the stream stops (no zeros): **not yet measured.**

### Units & kinematics (measured, robot on hard floor)

| Command L/R | Duration | Steady speed | Odometry Δ (L/R) | Pose / heading Δ |
|---|---|---|---|---|
| 80 / 80 | 1.0 s | ~11 mm/s | 11.5 / 11.2 | barely visible |
| 150 / 150 | 1.5 s | ~25 mm/s | 36.4 / 37.1 | ~1 inch observed |
| 300 / 300 | 2.5 s | ~50 mm/s | 114.7 / 115.1 | pose moved 120 |
| 300 / 300 | 1.0 s | ~50 mm/s | 48.5 / 48.8 | |
| -150 / 150 | 1.5 s | ±24 mm/s | -35.7 / 36.4 | θ +0.30 rad (~15–17° left) |
| -200 / -200 | 1.5 s | ~36 mm/s back | -55.1 / -54.9 | |

- **Command units ≈ 6 × mm/s** (linear across 80–300; reaches speed within ~0.25 s, stops promptly on zeros).
  Higher speeds not yet characterised beyond what teleop testing showed working.
- Positive = forward. `[-w, +w]` spins **left / counter-clockwise**, θ increases (ROS convention).
- **Odometry `/wheel/WheelDistanceReport`**: cumulative per wheel, **mm**, ~40 Hz.
- **`/robot/Robot` pose x/y are mm** (not metres). θ in radians.
- Effective wheelbase ≈ **228 mm** (from a 15 s spin vs IMU heading, `imu_odometry.md`; a first 1.5 s spin suggested ~240 mm).
- Wheel motor current ~50–80 mA while driving on hard floor (`/robot/Robot` currents types 5/6).

### Driving from Foxglove

`.venv/bin/python scripts/robot_viz.py --drive` (opt-in) also accepts `geometry_msgs/Twist` on **`/cmd_vel`**
from Foxglove's **Teleop** panel (Foxglove WebSocket `ClientPublish` capability, JSON encoding) and converts it
with differential-drive kinematics: `left/right = v ∓ ω·B/2`, B = 228 mm, scaled so neither wheel exceeds
`--max-speed` (default 100 mm/s; `--max-turn` default 0.8 rad/s), then × 6 into wheel-command units.

Teleop panel settings that matter (the defaults don't work well):

| Setting | Value | Why |
|---|---|---|
| Topic | type `/cmd_vel` | the dropdown only lists server topics — type it |
| Publish rate | 10 Hz | the bridge's deadman stops the robot after 0.4 s without a command |
| Stop on release | On | sends a zero Twist immediately on release |
| Up / Down | Linear X ±0.1 | m/s (default 1 m/s would just be clamped) |
| Left / Right | Angular Z ±0.5 | rad/s |

Safety is the same as `robot_teleop.py` (latched e-stop on new bump / wheel-drop / cliff, stop on stale
telemetry, zeros + unregister on exit), plus Foxglove services `/drive/estop` and `/drive/clear_estop`
(Service Call panel, request `{}`) and a JSON `/drive/status` topic. Don't run `robot_teleop.py` at the same
time — both would publish wheel speeds.

---

## Dead ends (and why)

### 1. `mdsctl rosnode '{"todo":"setspeed", ...}'` — stubbed out
In `plug_medusa_plugin_ros_node.so`, the JSON-cmd handler does `if (!strcmp(todo, "setspeed")) return 0;`.
No state check — it simply does nothing in this firmware. (`rtctl`/`getBatteryInfo` next to it does work.)

### 2. Publishing to `/subtasks/remotecontrol_task/control` — dropped
The `/subtasks/*/control` + `/result` topics are a private parent↔child protocol between the task manager
and its subtasks, keyed by `session` (e.g. `/node:task_manager:...`) and `sn`. The child logs
`recv ctl cmd from invalid session %s, expect %s, drop msg`. The old `robot_drive.py` also used the wrong
JSON keys — the real ones are `{"type": RemoteMoveType, "time": lastTime_ms, "v": v, "w": w}`.

### 3. `/task/WorkManage` START + `workType=9` (REMOTE_CONTROL) — accepted but never runs
This is the "official" entry (`DxTaskManager::Impl::onTaskControlRequested` → `StartReMoteControlTask`).
Request = `managetype, workType, WorkData` (remote move in `data.extraWorkData.remotes`, a `task/RemoteMove`).
Off-dock it returns `OK (0)`, then the task manager sends STOP (`type 1`) to every subtask
(return, remotecontrol, relocation, goto, preparation), and **never sends START to `remotecontrol_task`**.
No wheel command is ever published. Root cause not found (stall inside the deferred `stopAllTask` callback);
the T8 AIVI app has no manual-drive feature, so this path may be vestigial. Not worth pursuing.

---

## The remote-control lockout (from disassembly)

Even if path 3 worked, these gates exist in `onTaskControlRequested` / `canStartRobot` for workType 9:

- `canStartRobot(9)` refuses (response code) on: **wheel-fall alert → 2**, **dirt box missing → 3**,
  **cliff/down alert → 11**. Low battery does *not* block remote. ⇒ a "wheels-up" test is impossible on
  this path — lifting the robot trips wheel-fall.
- **On the dock** (`robotMsg.isOnCharger`, in-memory offset 0xb2): remote START also requires a one-shot flag
  (`Impl+0x8a8`). It is set at boot and in `onChargeStateChanged` when the robot docks, and **cleared by the
  first remote START**. Any later START while still docked **returns OK and silently does nothing** until the
  robot leaves the dock and re-docks (or reboots).
- Off the dock: START is ignored (still OK) if a relocation/preparation task is current (`CurTaskIsRelOrPre`).
- If the robot was asleep, the request first calls `SleepWorkUp()` then continues normally.
- `WorkResponseType` codes: 0 OK, 1 FAILED, 2 WHEEL_FALL, 3 NO_DIRT_BOX, 4 DURING_RELOCATION,
  5 DURING_PREPARATION, 7 LOW_BATTERY, 8 RETURN_ON_CHARGER, 11 DOWN_ERR, 12 BUMP_ERR, 14 REMOTING, ...

Remote node internals, for reference: FORWARD/BACKWARD/SPIN use fixed internal speeds (forward = 200 in
wheel-command units) for `time` ms; `v`/`w` only matter for MVA_CUSTOM (9) and TURN_LEFT/RIGHT (10/11);
`setMoveControlSpeed` computes `left = v + w·wb/200`, `right = v − w·wb/200`.

---

## Debugging notes

- deebot's own logs are effectively off: `/tmp/log/ros.log.000` stays empty, no syslogd runs. Trace behaviour
  by subscribing to topics instead (every `/subtasks/*`, `/task/*`, `/wheel/SetWheelSpeed` ...).
- Robot-side Python 2 can decode any message with the shipped genpy classes:
  `PYTHONPATH=/opt/ros/melodic/lib/python2.7/site-packages python` then `from robot.msg import Robot` etc.
  Pipe scripts over `ssh ... 'python -' < script.py` so nothing is written to the robot.
- Service calls from the Mac need only a forward tunnel to the service's rosrpc port
  (`lookupService` → `rosrpc://localhost:PORT`). Publishing from the Mac needs two reverse tunnels
  (XML-RPC + TCPROS) — dropbear supports `-R`.
