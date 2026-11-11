# /robot/Robot Topic

**Type:** `robot/Robot`  
**Publisher:** `/node` (deebot process)  
**MD5:** `5cfb7b142ed6323fe50812ffb59c30d4`  
**Publishes:** continuously, even while docked

This is the most useful single topic for monitoring robot state. It bundles pose,
battery, charge state, motor currents, all sensors, and a full LIDAR scan into
one message.

---

## Full Field Reference

```
robot/Robot
├── Header header
├── prediction/PredictPose pose
│   ├── Pose predictPose    (dead-reckoning estimate)
│   │   ├── Header header
│   │   ├── float32 x       (mm)
│   │   ├── float32 y       (mm)
│   │   └── float32 theta   (radians)
│   └── Pose pose           (corrected pose — use this one)
│       ├── Header header
│       ├── float32 x       (mm)
│       ├── float32 y       (mm)
│       └── float32 theta   (radians)
├── power/Battery battery
│   ├── uint8 battery               (0–100 %)
│   └── uint8 isLowVoltageToPowerOff
├── power/ChargeState chargestate
│   ├── uint8 isOnCharger           (0=off, 1=standard, 3=3D charger)
│   └── uint8 chargeState           (0=not charging, 1=charging, 2=full, 3=error, 4=error+shutoff)
├── motor/MotorCurrent currents     (MotorCurrentValue[])
│   └── each: uint8 type + uint16 current (mA)
│       types: 0=mainbrush, 1=left_sidebrush, 2=right_sidebrush, 3=fan, 4=waterpump, 5=left_wheel, 6=right_wheel
├── rangeDet/RangeDetect rangeDetects
│   ├── Header header
│   └── RangeDetValue[]             (one entry per sensor type)
│       └── each: uint8 type + uint16[] values
│           types: 0=front_buffer (3 values), 1=side_in (2 values), 2=down_in (5 values), 3=ultrasound (1 value)
├── onOffInfo/OnOffInfo onOffInfo   (OnOffSensorValue[])
│   └── each: uint8 type + uint8 value
│       types: 0=bump, 1=downin, 2=fall, 3=dirtbox, 4=carpet, 5=mop, 6=waterbox
│       (value for bump appears to be a bitmask: bit0=left, bit1=right; 3=both pressed)
├── onOffInfo/BumpValue[]           (per-bump-sensor breakdown)
│   └── each: uint8 type + uint8 value
│       types: 0=left, 1=right, 2=lds
├── onOffInfo/DownInValue[]         (6 downward IR sensors)
│   └── each: uint8 type + uint8 value (0=type 0–5)
├── onOffInfo/FallValue[]           (2 fall sensors)
│   └── each: uint8 type + uint8 value
├── onOffInfo/DirtBoxValue[]        (1 entry)
│   └── uint8 type + uint8 value   (0=not full, 1=full)
├── onOffInfo/CarpetValue[]         (1 entry)
│   └── uint8 type + uint8 value   (0=no carpet, 1=detected)
├── onOffInfo/WaterBoxValue[]       (1 entry)
│   └── uint8 type + uint8 value   (0=not set, 1=set, 2=ultra set)
└── lds/Lds ldsPic
    ├── Header header               (has real timestamp: secs/nsecs since boot)
    ├── LdsPoint[] points           (typically ~981 points)
    │   └── each: float32 x, y, rho, theta, power  (x/y/rho in mm, theta in rad)
    └── prediction/PredictPose ldsPose
        └── (same two-Pose structure as top-level pose; usually 0,0,0 on dock)
```

---

## Parsing (Python 2 on robot)

```python
import xmlrpclib, socket, struct

class Buf(object):
    def __init__(self, data): self.data = data; self.pos = 0
    def read(self, n):
        v = self.data[self.pos:self.pos+n]; self.pos += n; return v
    def u8(self):  return struct.unpack('B',  self.read(1))[0]
    def u16(self): return struct.unpack('<H', self.read(2))[0]
    def u32(self): return struct.unpack('<I', self.read(4))[0]
    def f32(self): return struct.unpack('<f', self.read(4))[0]
    def string(self): n = self.u32(); return self.read(n)
    def header(self):
        seq = self.u32(); secs = self.u32(); nsecs = self.u32()
        return {'seq': seq, 'secs': secs, 'nsecs': nsecs, 'frame_id': self.string()}
    def pose(self):
        h = self.header()
        return {'header': h, 'x': self.f32(), 'y': self.f32(), 'theta': self.f32()}
    def predict_pose(self):
        return {'predictPose': self.pose(), 'pose': self.pose()}

# ... (subscribe and get raw message bytes, see interfacing.md) ...

b = Buf(raw)
h        = b.header()           # outer header (seq, secs always 0)
pose     = b.predict_pose()     # robot position — use pose['pose'] for corrected x/y/theta
battery  = b.u8()               # 0–100 %
low_volt = b.u8()               # 1 = critically low
on_chg   = b.u8()               # 1 = on charger
chg_st   = b.u8()               # 1 = charging, 2 = full

# motor currents
n = b.u32()
motors = [(b.u8(), b.u16()) for _ in range(n)]  # (type, mA)

# range sensors
rh = b.header()
n = b.u32()
ranges = {}
for _ in range(n):
    rtype = b.u8(); nv = b.u32()
    ranges[rtype] = [b.u16() for _ in range(nv)]

# onOffInfo summary (type→value map)
n = b.u32()
onoff = {}
for _ in range(n):
    t = b.u8(); onoff[t] = b.u8()

# per-sensor arrays
def read_sensor_array(b):
    n = b.u32(); return [(b.u8(), b.u8()) for _ in range(n)]

bump_vals    = read_sensor_array(b)   # [(type, value), ...]
downin_vals  = read_sensor_array(b)
fall_vals    = read_sensor_array(b)
dirtbox_vals = read_sensor_array(b)
carpet_vals  = read_sensor_array(b)
waterbox_vals= read_sensor_array(b)

# LDS
lh = b.header()                       # has real secs/nsecs (monotonic clock)
n = b.u32()
lds_points = []
for _ in range(n):
    lds_points.append((b.f32(), b.f32(), b.f32(), b.f32(), b.f32()))  # x, y, rho, theta, power
lds_pose = b.predict_pose()
```

---

## Observed Values (on dock, 1% battery, charging)

| Field | Value | Notes |
|-------|-------|-------|
| `pose.pose.x/y` | 5.822, -1.626 mm | Last known position before docking |
| `pose.pose.theta` | -0.0016 rad | Heading |
| `pose.predictPose.*` | 0, 0, 0 | Dead-reckoning inactive on dock |
| `battery` | 1 % | Deeply discharged |
| `isLowVoltageToPowerOff` | 1 | Low-voltage flag set |
| `isOnCharger` | 1 | On standard charger |
| `chargeState` | 1 | CHARGE_STATE_NORMAL_CHARGING |
| Motor currents | all 0 (right_wheel=1) | Idle on dock |
| `bump` (onOffInfo) | 3 | Bitmask: both L+R bumpers pressed against dock |
| `BumpValue left/right` | 1 / 1 | Both bumpers active |
| `front_buffer` | [0, 1, 1] | Two of three front IR sensors blocked by dock |
| `LdsPoint count` | 981 | LIDAR running even on dock |
| LDS point units | mm | x, y, rho in millimeters; theta in radians |

---

## Quick One-Liner (no TCPROS)

```sh
mdsctl rosnode '{"todo":"rtctl","cmd":"getBatteryInfo"}'
# → {"ret":"ok","battery":<pct>,"inCharging":<0|1>}
```
