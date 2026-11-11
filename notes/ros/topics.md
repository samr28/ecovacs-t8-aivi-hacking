# ROS Topics Reference

Full topic list from `ros_enumerate.py` via SSH tunnel. Two nodes publish: `/node` (deebot)
and `/ROSNODE` (Python MDS). See `msg_defs_raw.txt` for complete message definitions.

## Topics by Category

### Power / Charging

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/power/Battery` | `power/Battery` | `/node` | `uint8 battery` (0–100), `uint8 isLowVoltageToPowerOff` |
| `/power/ChargeState` | `power/ChargeState` | `/node` | `isOnCharger` (0/1/3=3D), `chargeState` enum (0=not charging, 1=charging, 2=full, 3=error) |
| `/power/ChargeSignals` | `power/ChargeSignals` | `/node` | IR dock signals: array of direction + dockSignal + virtualwall |
| `/power/ChargeStateCause` | `power/ChargeStateCause` | `/node` | Why charge state changed (bump, drag, lift, etc.) |
| `/power/ChargerType` | `std_msgs/UInt8` | `/node` | |
| `/power/PowerMode` | `power/PowerMode` | `/node` | ERP/CEC/PowerOff/Reboot modes |
| `/power/T3DChargerReport` | `power/T3DChargerReport` | `/node` | 3D charger report |

### Task / Work State

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/task/WorkState` | `task/WorkState` | `/node` | Current work type + state + work data. Key topic for monitoring. |
| `/task/TaskResult` | `task/TaskResult` | `/node` | Result when a task completes (GOTO/PATROL/REMOTE/CLEAN/RETURN) |
| `/task/TaskStateReport` | `task/TaskStateReport` | `/node` | tasktype + stateType + state uint32s |
| `/task/MapBuildState` | `task/MapBuildState` | `/node` | Whether map is being built / built |
| `/task/RelocationReport` | `task/RelocationReport` | `/node` | Relocation progress/result |
| `/task/WorkState` | `task/WorkState` | `/node` | |
| `/task/FanMode` | `task/FanMode` | `/node` | Current fan mode during task |
| `/task/WaterLevel` | `task/WaterLevel` | `/node` | Current water level during task |
| `/task/CurrentArea` | `task/CurrentArea` | `/node` | `int32 curareaid` — which room being cleaned |
| `/task/VideoState` | `task/VideoState` | `/ROSNODE` | Camera on/off + notice type |
| `/task/sleepstate` | `task/sleepstate` | `/node` | Sleep/wake events |
| `/task/chargereport` | `task/chargereport` | `/node` | Return-to-charge events |
| `/task/AiPoints` | `task/AiPoints` | `/node` | AI avoidance points and their clean state |
| `/task/AiPointCollect` | `task/AiPointCollect` | `/node` | Start/end/pop AI point collection |

### Subtask Control (Command Interface)

These are the **command channels** — publish here to control the robot.

| Topic | Type | Notes |
|-------|------|-------|
| `/subtasks/clean_task/control` | `task/common_task` | Start/stop clean |
| `/subtasks/clean_task/result` | `task/common_task_result` | Clean task result |
| `/subtasks/goto_task/control` | `task/common_task` | Navigate to position |
| `/subtasks/goto_task/result` | `task/common_task_result` | Goto result |
| `/subtasks/patrol_task/control` | `task/common_task` | Patrol to points |
| `/subtasks/patrol_task/result` | `task/common_task_result` | Patrol result |
| `/subtasks/remotecontrol_task/control` | `task/common_task` | Remote control subtask (internal session protocol — external publishes are dropped; see `notes/driving.md`) |
| `/subtasks/remotecontrol_task/result` | `task/common_task_result` | Remote result |
| `/subtasks/return_task/control` | `task/common_task` | Return to dock |
| `/subtasks/return_task/result` | `task/common_task_result` | Return result |
| `/subtasks/relocation_task/control` | `task/relocation_task` | Relocate (find position on map) |
| `/subtasks/relocation_task/result` | `task/relocation_task_result` | Relocation result with pose |
| `/subtasks/preparation_task/control` | `task/common_task` | Preparation task |
| `/subtasks/preparation_task/result` | `task/common_task_result` | Preparation result |

`common_task` format: `string session, uint8 sn, uint8 type, string data` (data is JSON).

### Position / Odometry

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/prediction/PredictPose` | `prediction/PredictPose` | `/node` | `predictPose` + `pose` (both `{header, x, y, theta}`). x/y in millimetres, theta in radians. |
| `/prediction/UpdatePose` | `prediction/UpdatePose` | `/node` | Pose update with interpolation flag |
| `/prediction/PredictionOperation` | `prediction/PredictionOperation` | `/node` | Reset pose |
| `/wheel/WheelDistanceReport` | `wheel/WheelDistanceReport` | `/node` | `header + float32[] wheelDistance` (left, right odometry — cumulative, mm, ~40 Hz) |

### Motor Control (Direct Hardware)

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/wheel/SetWheelSpeed` | `wheel/SetWheelSpeed` | `/ROSNODE`, `/node` | **Direct wheel speed control — the working drive path.** `speed_type` (DRIVING/PHYSICAL), `float32[] speed`; real mm/s ≈ value / 6 |
| `/motor/ControlMotor` | `motor/ControlMotor` | `/node` | Control any motor by type (mainbrush, sidebrush, fan, waterpump, wheels) + controlType + value |
| `/motor/MotorCurrent` | `motor/MotorCurrent` | `/node` | Current draw per motor (type + uint16 current) |
| `/motor/MotorOnOffControl` | `motor/MotorOnOffControl` | `/node` | Turn motors on/off |
| `/motor/MotorProtection` | `motor/MotorType` | `/node` | Motor protection events |
| `/motor/MotorCleanSilent` | `motor/MotorCleanSilent` | `/node` | Silent/water clean mode |

Motor types: 0=mainbrush, 1=left_sidebrush, 2=right_sidebrush, 3=fan, 4=waterpump, 5=left_wheel, 6=right_wheel

### LIDAR (LDS)

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/lds/Lds` | `lds/Lds` | `/node` | Full lidar scan: `header + LdsPoint[] + ldsPose`. Each point: `x, y, rho, theta, power` (float32). |
| `/lds/ControlLds` | `lds/ControlLds` | `/node` | Start/stop/reset LDS |
| `/lds/CommControl` | `comm/CommControl` | `/node` | |
| `/lds/SendData` / `/lds/RecvData` | `comm/SendData/RecvData` | `/node` | Raw serial comms to LDS hardware |
| `/linelaser/LineLaserData` | `lds/LineLaserData` | (sub only) | Structured light line laser |
| `/linelaser/LineLaserState` | `lds/LineLaserState` | (sub only) | Line laser state |
| `/rocklds/RockAct` / `/rocklds/RockResult` | `rocklds/*` | `/node` | Rock sensor (structured light dock detector) |

### IMU

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/imu/ImuSensor` | `imu/ImuSensor` | `/node` | `header + float32[] gyros + float32[] accs` — gyro (rad/s) and accel (m/s²) arrays |
| `/imu/ImuState` | `imu/ImuState` | `/node` | `uint8 gyroState, uint8 accState` |
| `/imu/InitIMU` | `imu/InitIMU` | `/node` | Calibrate IMU (GYRO / ROLL_PITCH_YAW / GYRO_AND_ACC) |

### SLAM / Map

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/slam/SlamMap` | `slam/SlamMap` | `/node` | Full occupancy grid: `SlamMapMeta + int8[] data`. Meta has sizex/sizey, xmin/xmax/ymin/ymax, resolution. |
| `/slam/SlamCloseRangeMap` | `slam/SlamCloseRangeMap` | `/node` | Close-range occupancy map |
| `/slam/SlamControl` | `slam/SlamControl` | `/node` | Start/stop/pause/resume/reset SLAM |
| `/slam/SlamReport` | `slam/SlamReport` | `/node` | SLAM events (slip warning, pose update) |
| `/slam/SlamMapControl` | `slam/SlamMapControl` | `/node` | Load a saved map |
| `/slam/SlamMapRelocationResult` | `slam/SlamMapRelocationResult` | `/node` | Map-based relocation result |
| `/slam/DynamicDirectionReport` | `slam/DynamicDirectionReport` | `/node` | `float32 theta` — direction of movement |
| `/map/SlamMap` (via `/map/CarpetMap`) | `slam/SlamMap` | `/node` | Carpet-detected version of the map |

### Map Management

| Topic | Type | Notes |
|-------|------|-------|
| `/map/CompressMap` | `map/CompressMap` | Compressed map (submaps with CRC32) sent to cloud |
| `/map/SpotAreas` | `map/SpotAreas` | Room/area polygons with names, settings (fan/water/cycles) |
| `/map/VirtualWalls` | `map/VirtualWalls` | Virtual wall polygons |
| `/map/ChargerDockInfo` | `map/ChargerDockInfo` | Dock position (x, y, theta) |
| `/map/TraceUpdate` | `map/TraceUpdate` | Robot path trace points |
| `/map/MapInfos` | `map/MapInfos` | Saved maps list (id, name, active flag) |
| `/map/MapControl` | `map/MapControl` | Reset/save/load/regenerate maps |
| `/map/AiAreas` | `map/AiAreas` | AI-detected areas |
| `/map/WatchPoints` | `map/WatchPoints` | Camera watch/patrol points |

### Sensors / On-Off Info

| Topic | Type | Notes |
|-------|------|-------|
| `/onOffInfo/OnOffInfo` | `onOffInfo/OnOffInfo` | Array of sensor states: bump(L/R/LDS), downin(6 positions), fall(L/R), dirtbox, carpet, mop |
| `/onOffInfo/DownInCovered` | `onOffInfo/DownInCovered` | `uint8 value` — dustbin bay covered (1) or not |
| `/onOffInfo/LackofWaterValue` | `onOffInfo/LackofWaterValue` | Water box has water (0) or lacks water (1) |
| `/onOffInfo/WaterBoxStateReport` | `onOffInfo/WaterBoxStateReport` | Water box test report |
| `/rangeDet/RangeDetect` | `rangeDet/RangeDetect` | Range sensor readings: front buffer, side-in, down-in, ultrasound |
| `/alert/Alerts` | `alert/Alerts` | Alert array: type + state (0=idle, 1=triggered) |
| `/alert/DirtboxAndFall` | `alert/DirtboxAndFall` | Dirtbox/fall alert events |
| `/alert/LdsErrorData` | `alert/LdsErrorData` | Raw LDS error hex data |
| `/key/Key` | `key/Key` | Physical button events: type(AUTO/RESET) + event(click/double/long/etc.) |

### Settings

| Topic | Type | Notes |
|-------|------|-------|
| `/setting/FanMode` | `setting/FanMode` | Fan speed: 0=low, 1=high, 2=very high + silent flag |
| `/setting/WaterLevel` | `setting/WaterLevel` | Water: 0=low, 1=middle, 2=high, 3=ultrahigh |
| `/setting/AiSetting` | `setting/AiSetting` | AI avoidance on/off + per-object settings (wire/shoe/cloth/bin/rug) |
| `/setting/MopMode` | `setting/MopMode` | Effective(0) vs Performance(1) mop mode |
| `/setting/SlOnOff` | `setting/SlOnOff` | Structured light on/off |
| `/setting/PersonalCleanOnOff` | `setting/PersonalCleanOnOff` | Personal clean feature on/off |
| `/setting/BlockTimeSetting` | `setting/BlockTimeSetting` | Do-not-disturb time window |
| `/setting/CustomSettings` | `setting/CustomSettings` | Array of custom setting type+value pairs |

### Audio

| Topic | Type | Notes |
|-------|------|-------|
| `/audio/AudioPlay` | `audio/AudioPlay` | Play audio by index (0–106 defined, e.g. 8=LOW_BATTERY, 23=CHARGING) |
| `/audio/AudioPlayNew` | `audio/AudioPlayNew` | Extended audio play |
| `/audio/AudioPlayControl` | `audio/AudioPlayControl` | `uint8[] audioIdx` |

### Communication / Protocol

| Topic | Type | Notes |
|-------|------|-------|
| `/comm/SendData` / `/comm/RecvData` | `comm/SendData/RecvData` | Raw `byte[]` serial data to/from mainboard MCU |
| `/protocol/SendSAData` / `/protocol/RecvSAData` | `protocol/SendSAData/RecvSAData` | Structured SA protocol: mainCmd, subCmd, size, data[], type(req/resp/report) |

### Big Data / Telemetry

| Topic | Type | Notes |
|-------|------|-------|
| `/bigdata/CastDataToInternet` | `bigdata/CastDataToInternet` | `string rn` (event name) + `string data` (JSON) — the cloud uplink |
| `/bigdata/InternetAvaliable` | `bigdata/InternetAvaliable` | Online(1)/offline(0) |
| `/bigdata/BigDataTaskStruct` | `bigdata/BigDataTaskStruct` | Task telemetry: type, subtype, action, content |
| `/bigdata/BigDataBasicInfo` | `bigdata/BigDataBasicInfo` | Battery, waterbox, sleep, charge, robotstate + positions |

### Wifi / App Messages

| Topic | Type | Notes |
|-------|------|-------|
| `/wifimsg/wifimsg` | `wifimsg/wifimsg` | App-level events (2nd-level pass, OTA) |
| `/wifimsg/wifimsgreport` | `wifimsg/wifimsgreport` | Status reports to app (40+ event types) |
| `/wifimsg/wifimsgcameractrl` | `wifimsg/wifimsgcameractrl` | Camera control from app: photograph, upload, delete |
| `/wifimsg/wifimsgcamerareport` | `wifimsg/wifimsgcamerareport` | Camera report to app |
| `/wifimsg/handrelocation` | `wifimsg/handrelocation` | App-triggered relocation |
| `/wifimsg/PatrolPoints` | `wifimsg/PatrolPoints` | Patrol point states (UNARRIVE/TAKINGPHOTO/FINISH/CANNOTARRIVE) |
| `/wifimsg/breakpointstatereport` | `wifimsg/breakpointstatereport` | Break-point clean state |

### Miscellaneous

| Topic | Type | Notes |
|-------|------|-------|
| `/robot/Robot` | `robot/Robot` | `/node` | **Composite snapshot**: pose + battery + chargestate + motor currents + sensors + LDS scan. **Publishes even while docked.** Parse: outer Header(16B) + PredictPose(56B) → byte 72=`battery`(0–100), 73=`isLowVoltageToPowerOff`, 74=`isOnCharger`, 75=`chargeState`. |
| `/rtc/Rtc` | `rtc/Rtc` | RTC get/set/report with UTC time (uint32) |
| `/rtc/TimeZone` | `rtc/TimeZone` | `int8 tz, int8 tz_min` |
| `/chatter` | `std_msgs/String` | Debug/test topic |
| `/debug/string` | `std_msgs/String` | Debug string output |
| `/node/eros_state` | `std_msgs/String` | Eros framework state string |
| `/mainboard/Deltats` | `std_msgs/Int64` | Mainboard timestamp delta |
| `/lifespan/Lifespan` | `lifespan/lifespan` | Brush/filter lifespan change or timeout events |
| `/schedule/schedulearea` | `schedule/schedulearea` | Scheduled clean area map IDs |

---

## Services

| Service | Node | Notes |
|---------|------|-------|
| `/task/WorkManage` | `/node` | Start/stop/pause tasks. Primary task control service. |
| `/task/RobotManage` | `/node` | Robot management commands |
| `/task/GetIsBreakPoint` | `/node` | Check if there's a resumable clean session |
| `/task/GetMapBuildState` | `/node` | Query map build state |
| `/map/GetCurrentCompressMap` | `/node` | Fetch current compressed map |
| `/map/ManipulateCharger` | `/node` | Set/reset charger dock position |
| `/map/ManipulateMapInfos` | `/node` | Get/set map metadata |
| `/map/ManipulateShowMap` | `/node` | Control what map is shown to app |
| `/map/ManipulateSpotArea` | `/node` | Create/delete/rename rooms |
| `/map/ManipulateTrace` | `/node` | Get/clear cleaning trace |
| `/map/ManipulateVirtualWall` | `/node` | Add/remove virtual walls |
| `/map/ManipulateWifiMap` | `/node` | WiFi signal map |
| `/map/WatchPointManage` | `/node` | Manage camera watch/patrol points |
| `/setting/SettingManage` | `/node` | Get/set robot settings |
| `/schedule/schedule` | `/node` | Get/set cleaning schedules |
| `/worklog/GetLogInfo` | `/node` | Fetch work history log |
| `/worklog/GetLastLogInfo` | `/node` | Fetch most recent work log entry |
| `/worklog/GetAiWorkStatistic` | `/node` | AI avoidance statistics |
| `/ota/Ota` | `/node` | OTA firmware update |
| `/lifespan/lifespan` | `/node` | Query lifespan stats |

Services use the standard ROS service call format (request/response message pairs).
Call via XML-RPC `lookupService` then TCPROS with `service=1` header flag.
