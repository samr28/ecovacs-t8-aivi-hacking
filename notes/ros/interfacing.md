# Interfacing with ROS — How To

## Setup

Always set up the SSH tunnel first:
```bash
ssh -f -N -L 11311:127.0.0.1:11311 root@<robot-ip>
```

Check master is reachable:
```bash
python3 ros_enumerate.py 127.0.0.1
```

---

## Reading Topics (Subscribe)

Since TCPROS ports are localhost-only on the robot, the easiest approach is to run
Python 2 scripts on the robot itself via SSH.

### Pattern: SSH + Run on Robot

```bash
ssh root@<robot-ip> "python2 /data/my_subscriber.py"
```

### Subscribing in Python 2 (raw TCPROS, no rospy)

```python
#!/usr/bin/env python
import xmlrpclib, socket, struct

MASTER = 'http://localhost:11311'
CALLER = '/my_node'

def read_ros_len_header(sock):
    raw = b''
    while len(raw) < 4:
        raw += sock.recv(4 - len(raw))
    total = struct.unpack('<I', raw)[0]
    data = b''
    while len(data) < total:
        data += sock.recv(total - len(data))
    fields = {}
    pos = 0
    while pos < len(data):
        flen = struct.unpack('<I', data[pos:pos+4])[0]; pos += 4
        field = data[pos:pos+flen]; pos += flen
        if b'=' in field:
            k, v = field.split(b'=', 1)
            fields[k] = v
    return fields

def encode_header(fields):
    parts = [struct.pack('<I', len(k+b'='+v)) + k + b'=' + v for k, v in fields.items()]
    data = b''.join(parts)
    return struct.pack('<I', len(data)) + data

def read_message(sock):
    hdr = b''
    while len(hdr) < 4:
        hdr += sock.recv(4 - len(hdr))
    msg_len = struct.unpack('<I', hdr)[0]
    data = b''
    while len(data) < msg_len:
        data += sock.recv(msg_len - len(data))
    return data

master = xmlrpclib.ServerProxy(MASTER)

# NOTE: /power/Battery never publishes while on dock. Use /robot/Robot instead,
# which publishes regardless of dock state.
topic = '/robot/Robot'
ttype = 'robot/Robot'

code, _, pub_uri = master.lookupNode(CALLER, '/node')
pub = xmlrpclib.ServerProxy(pub_uri)
code, _, protos = pub.requestTopic(CALLER, topic, [['TCPROS']])
proto, host, port = protos

sock = socket.socket()
sock.connect((host, port))
sock.sendall(encode_header({
    b'callerid': CALLER,
    b'topic': topic,
    b'type': ttype,
    b'md5sum': b'*',
    b'message_definition': b'',
}))
read_ros_len_header(sock)  # consume publisher header

# Parse /robot/Robot: outer Header(16B) + PredictPose(56B) → power/Battery at byte 72
msg = read_message(sock)
battery_pct = ord(msg[72])
low_voltage  = ord(msg[73])
on_charger   = ord(msg[74])
charge_state = ord(msg[75])
print("Battery: %d%% low_voltage=%d on_charger=%d charge_state=%d" % (
    battery_pct, low_voltage, on_charger, charge_state))
```

### Decoding Messages

Messages are serialized little-endian, field by field, in definition order:
- `uint8` → 1 byte
- `uint16` → 2 bytes LE
- `uint32` → 4 bytes LE
- `float32` → 4 bytes IEEE 754 LE
- `string` → uint32 length + UTF-8 bytes
- `byte[]` / `T[]` → uint32 count + N serialized items
- `time` → uint32 secs + uint32 nsecs

See `msg_defs_raw.txt` for all field definitions. Use Python `struct.unpack` to parse.

---

## Publishing Topics (Control Robot)

### Wheel Speed (Direct Drive)

Topic: `/wheel/SetWheelSpeed`
Type: `wheel/SetWheelSpeed`
```
uint8 speed_type   # 0=DRIVING, 1=PHYSICAL
float32[] speed    # [left_speed, right_speed]
```

**This is the working drive path** (verified 2026-11-10, see `notes/driving.md`,
`scripts/robot_teleop.py`). Publish continuously at ~20 Hz with `speed_type=0` (DRIVING).
Units are **not** mm/s: real speed ≈ command / 6.
```python
speed = [300.0, 300.0]    # forward ~50 mm/s
speed = [-150.0, 150.0]   # spin left (CCW) ~0.2 rad/s
speed = [0.0, 0.0]        # stop
```

### Remote Control Task — DOES NOT WORK (kept for reference)

Publishing here is dropped (parent/child session protocol), and `/task/WorkManage` START with
workType 9 returns OK but never starts the task. JSON keys are actually `type`/`time`/`v`/`w`.
Details and the dock lockout in `notes/driving.md`.

Topic: `/subtasks/remotecontrol_task/control`
Type: `task/common_task` → `string session, uint8 sn, uint8 type, string data`

The `data` field is JSON containing a `task/RemoteMove`-like structure:
```
RemoteMoveType: FORWARD=0, BACKWARD=1, STOP=2, BRAKE=3, SPINLEFT=4, SPINRIGHT=5,
                GOTO=6, ROTATE=7, REVERSE=8, TURN_LEFT=10, TURN_RIGHT=11
lastTime: uint16  (duration in ms)
v: int16  (linear velocity)
w: int16  (angular velocity)
```

### Motor Control (Individual Motors)

Topic: `/motor/ControlMotor`
Type: `motor/ControlMotor` → `ControlMotorValue[] controls`

Each `ControlMotorValue`:
```
MotorType type:     0=mainbrush, 1=left_sidebrush, 2=right_sidebrush, 3=fan, 4=waterpump
MotorControlType:   0=VOLTAGE, 1=SPEED, 2=CUSTOM, 3=PUMP, 4=PUMP_START
uint16 value
```

### Publishing in Python 2 (raw TCPROS)

To publish, you need to register as a publisher and listen for subscriber connections:

```python
import xmlrpclib, socket, struct, threading, time
# ... (register with master, start XML-RPC server to handle requestTopic calls)
# This is more involved — see standard TCPROS publisher implementation.
```

Working Mac-side implementation (XML-RPC + TCPROS servers reached via reverse SSH tunnels):
`scripts/robot_teleop.py` → `WheelPublisher`.

Note: `mdsctl rosnode '{"todo":"setspeed"}'` is a no-op stub in medusa — mdsctl cannot drive the wheels.

See `notes/mdsctl.md` for the cloud API command interface.

---

## Most Useful Topics for Hacking

### Monitoring (subscribe)

| Topic | Why |
|-------|-----|
| `/robot/Robot` | Battery + chargestate + pose + sensors in one message; works on dock. Bytes 72–75 = battery, isLowVoltage, isOnCharger, chargeState |
| `/power/Battery` | Battery level — but **never publishes on dock**; use `/robot/Robot` instead |
| `/power/ChargeState` | On charger? Charging? — also **never publishes on dock** |
| `/task/WorkState` | What is the robot doing right now |
| `/prediction/PredictPose` | Robot position (x, y, theta) |
| `/imu/ImuSensor` | Raw IMU — gyros + accs arrays |
| `/lds/Lds` | Full LIDAR scan — great for mapping |
| `/onOffInfo/OnOffInfo` | Bump/fall/carpet/dirtbox sensors |
| `/robot/Robot` | Composite of pose + battery + sensors + lidar in one message |
| `/slam/SlamMap` | Full occupancy grid map |
| `/key/Key` | Physical button presses |

### Control (publish)

| Topic | Why |
|-------|-----|
| `/wheel/SetWheelSpeed` | Simplest direct drive control |
| `/motor/ControlMotor` | Individual motor control (fan, brush, etc.) |
| `/slam/SlamControl` | Start/stop SLAM |
| `/map/MapControl` | Reset/save/load maps |
| `/audio/AudioPlay` | Trigger robot sound by index |

### Services (call)

| Service | Why |
|---------|-----|
| `/task/WorkManage` | Start/stop/pause tasks |
| `/map/GetCurrentCompressMap` | Pull the current map |
| `/setting/SettingManage` | Read/write settings |
| `/worklog/GetLastLogInfo` | Read last clean session stats |

---

## Getting the Current Map

1. Call service `/map/GetCurrentCompressMap` — returns a `map/CompressMap` with
   submaps (each is a uint32 mapid + MapMetaInfo + CompressSubMap[] with CRC32-checked int8[] data).
2. Or subscribe to `/slam/SlamMap` — publishes full `SlamMapMeta + int8[] data` on each SLAM update.

`SlamMapMeta`:
```
uint16 sizex, sizey   # grid dimensions in cells
float32 xmin, xmax, ymin, ymax  # world bounds in meters
float32 resolution   # meters per cell
```
`data`: flat array of `int8`, row-major. Values: 0=free, 100=occupied, -1=unknown.

---

## Discovering Node Ports at Runtime

```python
import xmlrpclib
master = xmlrpclib.ServerProxy('http://localhost:11311')
# Node XML-RPC URI (changes per restart):
code, _, node_uri = master.lookupNode('/', '/node')
# TCPROS port for a topic:
pub = xmlrpclib.ServerProxy(node_uri)
code, _, protos = pub.requestTopic('/', '/lds/Lds', [['TCPROS']])
proto, host, port = protos  # host='localhost', port=some_ephemeral_port
```
