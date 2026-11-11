# ROS / Eros Framework — Overview

## What Is Running

The robot runs **ROS Melodic** (Python 2.7, stripped install — no `rostopic`, `rosservice`, etc.)
on top of Ecovacs' custom wrapper called **eros**.

### Two Nodes

| Node name | Process | Role |
|-----------|---------|------|
| `/node` | `/usr/bin/deebot` | Main robot process. Dynamically loads 14 eros plugin `.so` files from `/usr/lib/node/`. Owns hardware, SLAM, map, task manager, etc. |
| `/ROSNODE` | `medusa` (PID 370, `medusa -f /tmp/deebot_px30_ai.conf`; launched by `medusa.sh`) | The cloud/app connector — a **protocol bridge** between ROS 1 and the Ecovacs cloud/app. **Uplink:** subscribes robot-state topics (`/slam/SlamMap`, `/robot/Robot`, `/prediction/PredictPose`, `/imu/ImuSensor`) and the `/bigdata/CastDataToInternet` funnel. **Downlink:** publishes command topics (`/wheel/SetWheelSpeed`, `/key/Key`, `/light/LightControls`, `/map/ManipulateMap`, `/map/AiAreas`). Owns the `/wifimsg/*` and most `/bigdata/*` namespaces. Also publishes `/chatter`, `/debug/string`. **NOTE:** this is *not* the roscore/rosmaster chain — that is a separate python process (roscore PID 351 → rosmaster PID 458) that only runs the master on `:11311`. Confirmed 2026-09-08 by port→PID: `lookupNode('/ROSNODE')`=`:43323`, `netstat` shows `:43323 LISTEN 370/medusa`. medusa also holds an established outbound TLS socket to a cloud IP (`:443`), consistent with the connector role. |

### Eros Plugins (loaded by `/node`)

| Library | Role |
|---------|------|
| `liberos_node_hardware_platform.so` | IMU, LDS (lidar), motors, wheels, on/off sensors, power, rangefinder, comms |
| `liberos_node_slam.so` | SLAM algorithm, pose prediction |
| `liberos_node_map.so` | Map management, area/room detection, virtual walls |
| `liberos_node_task_manager.so` | Task orchestration (clean, goto, patrol, remote, relocation) |
| `liberos_node_clean.so` | Clean subtask |
| `liberos_node_return.so` | Return-to-dock subtask |
| `liberos_node_setting.so` | Fan mode, water level, AI settings, schedule block times |
| `liberos_node_alert.so` | Sensor alerts (bump, fall, dirtbox, LDS error) |
| `liberos_node_lifespan.so` | Brush/filter wear tracking |
| `liberos_node_reclean.so` | Re-clean logic |
| `liberos_node_inspect_charger.so` | Charger inspection |
| `liberos_node_bigdata.so` | Telemetry / analytics reporting |
| `liberos_node_rock.so` | Rock LDS (structured-light sensor for dock) |
| `liberos_node_smartvirtualwall.so` | Smart virtual wall / barrier |

Plugin config: `/etc/conf/dxai_node.json`

---

## Connecting from Mac

### Constraint: Everything Is Localhost-Only

The ROS master binds to `127.0.0.1:11311` on the robot. All node XML-RPC APIs and
TCPROS publisher ports also bind to `localhost` only. You **cannot** reach any of this
directly over the network — SSH tunnels are required.

### SSH Tunnel for the Master

```bash
ssh -f -N -L 11311:127.0.0.1:11311 root@<robot-ip>
```

After this, `http://127.0.0.1:11311` on your Mac reaches the ROS master.

`ros_enumerate.py` in the repo works once this tunnel is up:
```bash
python3 ros_enumerate.py 127.0.0.1
```

### Subscribing / Publishing from Mac

Because publisher TCPROS ports are also localhost-only on the robot, you **cannot**
subscribe to topics directly from the Mac. Options:

1. **Run scripts on the robot via SSH** — simplest, no extra tunnels needed.
2. **Per-port SSH tunnels** — for each topic you want, tunnel the TCPROS port that
   the publisher hands out. Fragile (port changes per restart).
3. **Write a relay node on the robot** — a Python 2 script that subscribes and
   re-publishes over a raw TCP socket to the Mac.

For one-shot data capture or publishing commands, option 1 (SSH + run on robot) is easiest.

### Getting Message Data via TCPROS (on robot)

The robot has Python 2.7 and `xmlrpclib`/`socket`. The key trick is that the TCPROS
connection header the publisher sends back contains the full `message_definition` text.
This is how we extracted all the message definitions — see `msg_defs_raw.txt` and the
extractor script (`/tmp/extract_msg_defs.py` on robot, source in this project).

Protocol summary:
1. XML-RPC: `master.getTopicTypes('/')` → topic→type map
2. XML-RPC: `master.lookupNode('/', node_name)` → publisher XML-RPC URI
3. XML-RPC: `pub_node.requestTopic(caller_id, topic, [['TCPROS']])` → `(proto, host, port)`
4. TCP: connect to `(host, port)`, send subscriber header, read publisher header
   → header contains `md5sum`, `type`, `message_definition`

---

## Available ROS Tools on Robot

```
/opt/ros/melodic/bin/   roscore, rosmaster, roslaunch, rospack, rosrun, rosgraph
/opt/ros/melodic/lib/python2.7/site-packages/   rospy, genpy, genmsg, rosgraph_msgs, std_msgs
```

Note: `rospy` cannot be imported standalone because `roscpp.srv` is missing. Raw
`xmlrpclib`/`socket` approach (no rospy) works fine.

---

## Key Architecture Observations

- No `.msg` files on the robot — message definitions are compiled into the `.so` plugins
  and exposed only via the TCPROS connection header at runtime.
- The eros framework is a thin wrapper around standard ROS Melodic; the publish/subscribe
  API is identical to standard `roscpp`.
- `/ROSNODE` **is** the medusa process (confirmed 2026-09-08, PID 370, port→PID via netstat), a
  protocol bridge proxying the Ecovacs cloud API to/from ROS topics. It publishes `/bigdata/*`
  telemetry + command topics (downlink) and subscribes to robot-state topics (uplink). See the
  Two Nodes table above for the full uplink/downlink topic split.
- `/wheel/SetWheelSpeed` has **two publishers**: `/node` (autonomous driving) and `/ROSNODE`/medusa.
  A third publisher from the Mac **does drive the robot** — `/node`'s wheel driver forwards every message
  to the MCU with no task-state gating (verified on the floor 2026-11-10). The remote-control-task
  hypothesis was wrong; see `notes/driving.md`.
- Task commands flow through `task/WorkManage` (service) or `subtasks/*/control` (topics).
