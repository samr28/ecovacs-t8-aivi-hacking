#!/usr/bin/env python3
"""
Robot → Foxglove bridge (the start of our debug GUI).

Subscribes to the robot's ROS topics over SSH tunnels (same TCPROS approach as robot_watch.py),
converts them to Foxglove's ROS-style schemas, and serves them on a Foxglove WebSocket server.
Docs: notes/foxglove.md (setup, panels, topics), notes/lidar.md, notes/sensors.md, notes/camera.md,
notes/driving.md (--drive).

    .venv/bin/pip install foxglove-sdk           # once
    .venv/bin/python scripts/robot_viz.py        # then in Foxglove: Open connection →
                                                 #   Foxglove WebSocket → ws://localhost:8765
    .venv/bin/python scripts/robot_viz.py --record run.mcap   # also record (MCAP = rosbag2 format)

Topics served (all SI units — metres / radians, like ROS):
  /lidar        foxglove.PointCloud    current scan, frame "lidar"     (ROS: sensor_msgs/PointCloud2)
  /tf           foxglove.FrameTransforms map → base_link → lidar        (ROS: tf2_msgs/TFMessage)
  /robot/path   foxglove.PosesInFrame  recent trajectory in "map"      (ROS: nav_msgs/Path)
  /camera       foxglove.CompressedVideo  H.264 from the robot's hardware encoder, frame "camera"
                                       (ROS: foxglove_msgs/CompressedVideo; view in an Image panel)
  /robot/body   foxglove.SceneUpdate   robot outline + bumpers / cliff / wheel-drop markers, IR values,
                                       red when triggered; frame base_link  (ROS: visualization_msgs/MarkerArray)
  /robot/state  JSON                   battery, charging, hazards, sensors, motor currents, IR ranges
                                       (use Plot / Raw Messages / Indicator panels)

With --drive (opt-in, moves the robot):
  /cmd_vel      ← geometry_msgs/Twist from Foxglove's Teleop panel (set its topic to /cmd_vel,
                  publish rate 10 Hz, values ~0.1 m/s forward / ~0.5 rad/s turn)
  /drive/status JSON: e-stop state, hazards, commanded and wheel speeds, battery
  services      /drive/estop, /drive/clear_estop (std_srvs/Trigger-style; use the Service Call panel)
  Don't run robot_teleop.py at the same time — both would publish wheel speeds.

Frames:
  map        the robot firmware's own world frame (pose from /lds/Lds.ldsPose, mm → m)
  base_link  the robot body. x forward, y left, θ CCW-positive (verified with wheel tests and by
             placing objects behind / left of the robot)
  lidar      the LDS. Same orientation as base_link; mounting offset unknown → identity (notes/lidar.md)

Robot timestamps are seconds-since-boot; we shift them onto the Mac clock with a fixed offset
computed from the first message, so relative timing between messages is preserved.
"""

import argparse
import math
import signal
import socket
import struct
import sys
import threading
import time
import xmlrpc.client
from collections import deque
from urllib.parse import urlparse

import foxglove
from foxglove.messages import (ArrowPrimitive, Color, CompressedVideo, CubePrimitive, CylinderPrimitive,
                               FrameTransform, FrameTransforms, LinePrimitive, LinePrimitiveLineType,
                               PackedElementField, PackedElementFieldNumericType, Point3, PointCloud, Pose,
                               PosesInFrame, Quaternion, SceneEntity, SceneUpdate, SpherePrimitive,
                               TextPrimitive, Timestamp, Vector3)

from foxglove.websocket import ServerListener

import robot_watch as rw            # SSH tunnel + TCPROS framing helpers
from config import ROS_MASTER_PORT

MASTER_URL = f'http://localhost:{ROS_MASTER_PORT}'
CALLER_ID = '/robot_viz'
PATH_LEN = 600                      # poses kept in /robot/path (~2 min at 5 Hz)


# ── Generic TCPROS subscription over SSH tunnels ─────────────────────────────

def ensure_master():
    try:
        xmlrpc.client.ServerProxy(MASTER_URL).getSystemState(CALLER_ID)
    except OSError:
        import subprocess
        rw._tunnels.append(subprocess.Popen(
            ['ssh', '-N', '-o', 'StrictHostKeyChecking=no',
             '-L', f'{ROS_MASTER_PORT}:127.0.0.1:{ROS_MASTER_PORT}', rw.ROBOT],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        for _ in range(50):
            try:
                socket.create_connection(('localhost', ROS_MASTER_PORT), 0.2).close(); return
            except OSError:
                time.sleep(0.1)
        sys.exit('could not reach ROS master through ssh tunnel')


_node_ports: dict[str, int] = {}

def subscribe(topic: str, msg_type: str, node: str = '/node') -> socket.socket:
    """requestTopic on the publishing node → tunnel its TCPROS port → handshake. Returns the socket."""
    master = xmlrpc.client.ServerProxy(MASTER_URL)
    code, msg, uri = master.lookupNode(CALLER_ID, node)
    if code != 1:
        raise RuntimeError(f'lookupNode {node}: {msg}')
    rport = urlparse(uri).port
    if rport not in _node_ports:
        _node_ports[rport] = rw.open_tunnel(rport)
    api = xmlrpc.client.ServerProxy(f'http://localhost:{_node_ports[rport]}')
    code, msg, proto = api.requestTopic(CALLER_ID, topic, [['TCPROS']])
    if code != 1:
        raise RuntimeError(f'requestTopic {topic}: {msg}')
    if proto[2] not in _node_ports:
        _node_ports[proto[2]] = rw.open_tunnel(proto[2])
    s = socket.create_connection(('localhost', _node_ports[proto[2]]), 5)
    s.sendall(rw._encode_conn_header({'callerid': CALLER_ID, 'topic': topic,
                                      'type': msg_type, 'md5sum': '*', 'tcp_nodelay': '1'}))
    rw._read_framed(s)
    s.settimeout(10)
    return s


def run_subscriber(topic: str, msg_type: str, handler):
    """Thread body: subscribe, feed raw messages to handler, reconnect on failure."""
    while True:
        try:
            s = subscribe(topic, msg_type)
            print(f'  subscribed {topic}')
            while True:
                handler(rw._read_framed(s))
        except Exception as e:                              # noqa: BLE001 — keep the bridge alive
            print(f'  {topic}: {e!r} — reconnecting in 2 s')
            time.sleep(2)


# ── Time and geometry helpers ────────────────────────────────────────────────

class RobotClock:
    """Maps robot seconds-since-boot onto Mac wall-clock nanoseconds (fixed offset)."""
    def __init__(self):
        self.offset_ns = None

    def __call__(self, secs: int, nsecs: int) -> int:
        robot_ns = secs * 1_000_000_000 + nsecs
        if self.offset_ns is None:
            self.offset_ns = time.time_ns() - robot_ns
        return robot_ns + self.offset_ns

clock = RobotClock()

def ts(ns: int) -> Timestamp:
    return Timestamp(sec=ns // 1_000_000_000, nsec=ns % 1_000_000_000)

def yaw_quat(theta: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(theta / 2), w=math.cos(theta / 2))


# ── /lds/Lds decoding ────────────────────────────────────────────────────────
# lds/Lds: Header header | LdsPoint[] points (x, y, rho, theta, power: float32, mm/rad)
#          | prediction/PredictPose ldsPose (Pose predictPose, Pose pose; Pose = Header + x, y, theta)

def _header(b: bytes, o: int):
    seq, secs, nsecs, flen = struct.unpack_from('<IIII', b, o)
    return (secs, nsecs), o + 16 + flen

def decode_lds(b: bytes):
    stamp, o = _header(b, 0)
    (n,) = struct.unpack_from('<I', b, o); o += 4
    pts = b[o:o + 20 * n]; o += 20 * n
    _, o = _header(b, o); o += 12                     # predictPose (dead-reckoning) — skip
    _, o = _header(b, o)
    x, y, th = struct.unpack_from('<3f', b, o)        # corrected pose, mm / rad
    return stamp, n, pts, (x / 1000.0, y / 1000.0, th)


POINT_FIELDS = [
    PackedElementField(name='x', offset=0, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name='y', offset=4, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name='z', offset=8, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name='range', offset=12, type=PackedElementFieldNumericType.Float32),
]

path: deque = deque(maxlen=PATH_LEN)

def on_lds(raw: bytes):
    (secs, nsecs), n, pts, (px, py, pth) = decode_lds(raw)
    t = clock(secs, nsecs)

    # Points: mm → m, z = 0, keep range for colouring. 16-byte stride (x, y, z, range).
    out = bytearray(16 * n)
    for i, (x, y, rho, _th, _pw) in enumerate(struct.iter_unpack('<5f', pts)):
        struct.pack_into('<4f', out, 16 * i, x / 1000.0, y / 1000.0, 0.0, rho / 1000.0)
    foxglove.log('/lidar', PointCloud(timestamp=ts(t), frame_id='lidar', pose=Pose(
        position=Vector3(x=0, y=0, z=0), orientation=yaw_quat(0)),
        point_stride=16, fields=POINT_FIELDS, data=bytes(out)), log_time=t)

    # TF: map → base_link (robot pose at scan time), base_link → lidar (identity until measured)
    foxglove.log('/tf', FrameTransforms(transforms=[
        FrameTransform(timestamp=ts(t), parent_frame_id='map', child_frame_id='base_link',
                       translation=Vector3(x=px, y=py, z=0.0), rotation=yaw_quat(pth)),
        FrameTransform(timestamp=ts(t), parent_frame_id='base_link', child_frame_id='lidar',
                       translation=Vector3(x=0.0, y=0.0, z=0.0), rotation=yaw_quat(0.0)),
    ]), log_time=t)

    # Trajectory
    path.append(Pose(position=Vector3(x=px, y=py, z=0.0), orientation=yaw_quat(pth)))
    foxglove.log('/robot/path', PosesInFrame(timestamp=ts(t), frame_id='map', poses=list(path)),
                 log_time=t)


# ── Camera: hardware H.264 over ssh → foxglove.CompressedVideo ───────────────
# Same pipeline as robot_camera.sh (see notes/camera.md): ISP mainpath → mpi_enc_test → stdout.
# Stream facts (measured): one slice per frame, IDR every 60 frames (2 s @ 30 fps), but SPS/PPS
# are sent only once at the very start. Foxglove needs each keyframe to be decodable on its own
# (a viewer may join late), so we cache SPS/PPS and prepend them to every IDR frame.

def camera_remote_cmd(w: int, h: int) -> str:
    return (
        f"v4l2-ctl -d /dev/video0 --set-fmt-video=width={w},height={h},pixelformat=NV12 >/dev/null 2>&1;"
        f"{{ v4l2-ctl -d /dev/video0 --stream-mmap=4 --stream-count=0 --stream-to=- 2>/dev/null"
        # mpi_enc_test logs to *stdout*, which corrupted frames when the video also went to stdout.
        # So: video → fd 3 → `cat` → ssh; the tool's own stdout/stderr → /dev/null.
        f" | mpi_enc_test -i /dev/stdin -o /dev/fd/3 -w {w} -h {h} -f 0 -t 7 -n 0"
        f" 3>&1 1>/dev/null 2>/dev/null | cat; }} &"
        # watchdog: mpi_enc_test ignores EPIPE, so kill the group once our ssh session is gone
        "while kill -0 $PPID 2>/dev/null && [ $(cut -d' ' -f4 /proc/$$/stat) = $PPID ]; do sleep 1; done;"
        "kill 0")


def iter_nals(stream):
    """Yield H.264 NAL units (without start codes) from an Annex-B byte stream."""
    buf = b''
    while True:
        chunk = stream.read1(65536)
        if not chunk:
            break
        buf += chunk
        starts = [m for m in _find_starts(buf)]
        # emit all complete NALs (everything between consecutive start codes)
        for (s0, l0), (s1, _l1) in zip(starts, starts[1:]):
            yield buf[s0 + l0:s1]
        if starts:
            buf = buf[starts[-1][0]:]


def _find_starts(b: bytes):
    i = b.find(b'\x00\x00\x01')
    while i != -1:
        if i > 0 and b[i - 1] == 0:
            yield (i - 1, 4)
        else:
            yield (i, 3)
        i = b.find(b'\x00\x00\x01', i + 3)


def run_camera(w: int, h: int):
    import subprocess
    sc = b'\x00\x00\x00\x01'
    while True:
        proc = subprocess.Popen(['ssh', '-o', 'ServerAliveInterval=5', rw.ROBOT, camera_remote_cmd(w, h)],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        rw._tunnels.append(proc)              # terminated with the tunnels on exit
        print('  camera stream started')
        sps = pps = None
        pending = b''                         # SPS/PPS/SEI waiting for the next slice
        frames = 0
        for nal in iter_nals(proc.stdout):
            if not nal:
                continue
            t = nal[0] & 0x1f
            if t == 7:
                sps = nal
            elif t == 8:
                pps = nal
            if t not in (1, 5):               # not a slice → attach to the next frame
                if t not in (7, 8):
                    pending += sc + nal
                continue
            data = pending + sc + nal
            if t == 5:                        # IDR: make it self-contained
                if not (sps and pps):
                    pending = b''; continue
                data = sc + sps + sc + pps + data
            elif frames == 0:
                pending = b''; continue       # can't start on a P-frame
            pending = b''
            now = time.time_ns()
            foxglove.log('/camera', CompressedVideo(timestamp=ts(now), frame_id='camera',
                                                    data=data, format='h264'), log_time=now)
            frames += 1
        print('  camera stream ended — restarting in 2 s')
        proc.kill()
        time.sleep(2)


# ── Robot body, proximity sensors and state (/robot/Robot) ───────────────────
# /robot/Robot arrives at ~50 Hz and ~20 KB (it embeds the lidar scan) ≈ 1 MB/s, so the bridge
# subscribes to it ONCE and shares it: display (throttled to 10 Hz) + --drive safety (full rate).
# The small per-sensor topics are too slow for safety (/onOffInfo/OnOffInfo ~1.4 Hz).
#
# Sensor POSITIONS below are approximate — the firmware gives indices, not locations. Body
# diameter ≈ 350 mm. Cliff sensors: see CLIFF_POS (mapped from an underside photo).

class Telemetry:
    """Latest decoded /robot/Robot (without the lidar points) + robot_teleop's hazard() rules."""
    def __init__(self):
        self.d = None
        self.t = 0.0

    def hazard(self) -> str | None:
        import robot_teleop as rt
        return rt.Telemetry.hazard(self)        # same bump / wheel-drop / cliff / stale rules


def decode_robot_state(raw: bytes) -> dict:
    """Like robot_watch.decode but stops before the embedded lidar scan (cheap at 50 Hz)."""
    b = rw.Buf(raw)
    b.header()
    pose = b.predict_pose()
    battery, low_volt, on_charger, charge_state = b.u8(), b.u8(), b.u8(), b.u8()
    motors = {}
    for _ in range(b.u32()):
        t = b.u8(); motors[rw.MOTOR_NAMES.get(t, str(t))] = b.u16()
    b.header()
    ranges = {}
    for _ in range(b.u32()):
        t = b.u8(); nv = b.u32()
        ranges[rw.RANGE_NAMES.get(t, str(t))] = [b.u16() for _ in range(nv)]
    onoff = {t: v for t, v in b.sensor_array()}
    bump = {rw.BUMP_NAMES.get(t, str(t)): v for t, v in b.sensor_array()}
    downin = [v for _, v in b.sensor_array()]
    fall = [v for _, v in b.sensor_array()]
    dirtbox = next((v for _, v in b.sensor_array()), 0)
    carpet = next((v for _, v in b.sensor_array()), 0)
    waterbox = next((v for _, v in b.sensor_array()), 0)
    return dict(pose=pose['pose'], predict=pose['predict'], battery=battery, low_volt=low_volt,
                on_charger=on_charger, charge_state=charge_state, motors=motors, ranges=ranges,
                onoff=onoff, bump=bump, downin=downin, fall=fall, dirtbox=dirtbox, carpet=carpet,
                waterbox=waterbox)


telemetry = Telemetry()

BODY_R = 0.175                                   # m
# Cliff sensors downin[0..5]: (x, y) in base_link metres, measured from an underside photo
# (body Ø 350 mm) and mapped by covering each window (2026-11-10); left/right are the robot's own
# (top-down). Index names from onOffInfo/DownInValue. This model has FIVE cliff sensors: the two
# windows either side of the front caster are ONE sensor (F, downin[1]); downin[2] (FRONT_RIGHT) is
# an unpopulated slot of the shared message and always reads 0 → not drawn. (raw down_in has 5 values.)
CLIFF_POS = [
    (0.022, 0.158, True),     # c0  DOWNIN_LEFT        left side, outside the left wheel
    (0.132, 0.0, True),       # c1  DOWNIN_FRONT       front, both windows beside the caster
    (0.132, -0.038, False),   # c2  DOWNIN_FRONT_RIGHT not fitted on this model (always 0)
    (0.022, -0.158, True),    # c3  DOWNIN_RIGHT       right side, outside the right wheel
    (-0.074, 0.144, True),    # c4  DOWNIN_LEFT_BACK   back-left
    (-0.074, -0.144, True),   # c5  DOWNIN_RIGHT_BACK  back-right
]
OK = Color(r=0.2, g=0.85, b=0.35, a=0.95)
HIT = Color(r=1.0, g=0.15, b=0.15, a=1.0)
BODY = Color(r=0.65, g=0.65, b=0.72, a=0.30)
TEXT = Color(r=1.0, g=1.0, b=1.0, a=0.9)

def _pose(x=0.0, y=0.0, z=0.0, yaw=0.0) -> Pose:
    return Pose(position=Vector3(x=x, y=y, z=z), orientation=yaw_quat(yaw))

def _arc(a0: float, a1: float, r: float, z: float, n: int = 12) -> list:
    return [Point3(x=r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
                   y=r * math.sin(math.radians(a0 + (a1 - a0) * i / n)), z=z) for i in range(n + 1)]

def robot_scene(d: dict, t_ns: int) -> SceneUpdate:
    bump, fall, cliff = d['bump'], d['fall'], d['downin']
    lines = [  # bumper arcs: left covers front-left, right covers front-right
        LinePrimitive(type=LinePrimitiveLineType.LineStrip, thickness=0.02, points=_arc(5, 85, BODY_R + 0.01, 0.05),
                      color=HIT if bump.get('L') else OK),
        LinePrimitive(type=LinePrimitiveLineType.LineStrip, thickness=0.02, points=_arc(-85, -5, BODY_R + 0.01, 0.05),
                      color=HIT if bump.get('R') else OK),
    ]
    spheres = [SpherePrimitive(pose=_pose(x, y, 0.0), size=Vector3(x=0.03, y=0.03, z=0.03),
                               color=HIT if (i < len(cliff) and cliff[i]) else OK)
               for i, (x, y, fitted) in enumerate(CLIFF_POS) if fitted]
    cubes = [CubePrimitive(pose=_pose(0.0, side * 0.115, 0.02), size=Vector3(x=0.07, y=0.025, z=0.04),
                           color=HIT if (i < len(fall) and fall[i]) else OK)       # wheel-drop L, R
             for i, side in enumerate((1, -1))]
    cylinders = [
        CylinderPrimitive(pose=_pose(0, 0, 0.045), size=Vector3(x=2 * BODY_R, y=2 * BODY_R, z=0.09),
                          bottom_scale=1, top_scale=1, color=BODY),
        CylinderPrimitive(pose=_pose(0, 0, 0.105), size=Vector3(x=0.08, y=0.08, z=0.03),     # LDS turret
                          bottom_scale=1, top_scale=1, color=HIT if bump.get('LDS') else BODY),
    ]
    arrows = [ArrowPrimitive(pose=_pose(0, 0, 0.1), shaft_length=0.15, shaft_diameter=0.015,
                             head_length=0.05, head_diameter=0.04, color=TEXT)]
    rg = d['ranges']
    side = rg.get('side_in', []) + [0, 0]           # [left, right] wall sensors: 0 = nothing, ~3800 = touching
    texts = [
        TextPrimitive(pose=_pose(BODY_R + 0.08, 0, 0.1), billboard=True, font_size=12, scale_invariant=True,
                      color=TEXT, text=f"front IR {rg.get('front_buf', [])}"),
        TextPrimitive(pose=_pose(0, BODY_R + 0.1, 0.1), billboard=True, font_size=12, scale_invariant=True,
                      color=TEXT, text=f"wall IR L {side[0]}"),
        TextPrimitive(pose=_pose(0, -(BODY_R + 0.1), 0.1), billboard=True, font_size=12, scale_invariant=True,
                      color=TEXT, text=f"wall IR R {side[1]}"),
        TextPrimitive(pose=_pose(-(BODY_R + 0.1), 0, 0.1), billboard=True, font_size=12, scale_invariant=True,
                      color=TEXT, text=f"{d['battery']}%{' ⚡' if d['on_charger'] else ''}"),
    ]
    for i, (x, y, fitted) in enumerate(CLIFF_POS):
        if fitted:
            texts.append(TextPrimitive(pose=_pose(x, y, 0.04), billboard=True, font_size=9,
                                       scale_invariant=True, color=TEXT, text=f'c{i}'))
    return SceneUpdate(entities=[SceneEntity(
        timestamp=ts(t_ns), frame_id='base_link', id='robot', frame_locked=True,
        lines=lines, spheres=spheres, cubes=cubes, cylinders=cylinders, arrows=arrows, texts=texts)])


_last_scene = 0.0

def on_robot(raw: bytes):
    global _last_scene
    d = decode_robot_state(raw)
    telemetry.d, telemetry.t = d, time.time()
    now = time.time()
    if now - _last_scene < 0.1:                  # display at 10 Hz
        return
    _last_scene = now
    t = time.time_ns()
    foxglove.log('/robot/body', robot_scene(d, t), log_time=t)
    foxglove.log('/robot/state', {
        'battery': d['battery'], 'low_voltage': d['low_volt'], 'on_charger': d['on_charger'],
        'charge_state': rw.CHARGE_STATES.get(d['charge_state'], str(d['charge_state'])),
        'hazard': telemetry.hazard() or '',
        'bump': d['bump'], 'cliff': d['downin'], 'wheel_drop': d['fall'],
        'dirtbox': d['dirtbox'], 'carpet': d['carpet'], 'waterbox': d['waterbox'],
        'motors_mA': d['motors'], 'ranges': d['ranges'], 'onoff_raw': d['onoff'],
    }, log_time=t)


# ── Driving from Foxglove (--drive) ──────────────────────────────────────────
# Foxglove's Teleop panel publishes geometry_msgs/Twist (JSON) back to us (ClientPublish capability).
# We turn (linear.x m/s, angular.z rad/s) into left/right wheel speeds and feed robot_teleop's
# WheelPublisher (→ /wheel/SetWheelSpeed). Safety, same as robot_teleop.py:
#   - deadman: no Twist for CMD_TIMEOUT_S → stop (the panel only publishes while a button is held)
#   - bump / wheel-drop / cliff appearing → latched e-stop; stale telemetry → stop
#   - /drive/estop and /drive/clear_estop services (Foxglove "Service Call" panel buttons)

WHEELBASE_MM = 240.0                # measured, notes/driving.md
CMD_TIMEOUT_S = 0.4                 # set the Teleop panel's publish rate to ≥ 5 Hz (10 Hz recommended)

class DriveControl(ServerListener):
    def __init__(self, tel, max_speed_mmps: float, max_turn_radps: float):
        import robot_teleop as rt
        self.rt = rt
        self.tel = tel                         # shared, fed by the bridge's /robot/Robot subscription
        self.max_v, self.max_w = max_speed_mmps, max_turn_radps
        self.v_mmps = self.w_radps = 0.0
        self.last_cmd = 0.0
        self.estop: str | None = None
        self.prev_hz: str | None = None
        self.channels: dict[int, str] = {}     # client channel id → topic

    def start(self):
        self.pub = self.rt.WheelPublisher(); self.pub.start()
        for _ in range(50):
            if self.pub.connected():
                break
            time.sleep(0.1)
        print(f'  drive: /wheel/SetWheelSpeed publisher, {self.pub.connected()} subscriber(s)')
        threading.Thread(target=self._loop, daemon=True).start()

    # Twist from the Teleop panel
    def on_twist(self, msg: dict):
        lin, ang = msg.get('linear', {}), msg.get('angular', {})
        self.v_mmps = max(-self.max_v, min(self.max_v, float(lin.get('x', 0.0)) * 1000.0))
        self.w_radps = max(-self.max_w, min(self.max_w, float(ang.get('z', 0.0))))
        self.last_cmd = time.time()

    def wheel_speeds(self) -> tuple[float, float]:
        """Differential drive: v ± ω·B/2, scaled so neither wheel exceeds max speed. mm/s."""
        half = self.w_radps * WHEELBASE_MM / 2
        l, r = self.v_mmps - half, self.v_mmps + half
        k = max(1.0, abs(l) / self.max_v, abs(r) / self.max_v)
        return l / k, r / k

    def _loop(self):
        while True:
            now = time.time()
            hz = self.tel.hazard()
            stale = hz == 'telemetry stale'
            if hz and not stale and hz != self.prev_hz:
                self.estop = hz                # latch on rising edge
            self.prev_hz = hz
            active = now - self.last_cmd < CMD_TIMEOUT_S
            if active and not self.estop and not stale:
                l, r = self.wheel_speeds()
            else:
                l = r = 0.0
            self.pub.cmd = (l * self.rt.CMD_PER_MMPS, r * self.rt.CMD_PER_MMPS)
            d = self.tel.d or {}
            foxglove.log('/drive/status', {
                'estop': self.estop or '', 'hazard': hz or '', 'active': bool(l or r),
                'cmd_v_mmps': round(self.v_mmps if active else 0.0, 1),
                'cmd_w_radps': round(self.w_radps if active else 0.0, 3),
                'wheel_mmps': [round(l, 1), round(r, 1)],
                'subscribers': self.pub.connected(), 'battery': d.get('battery'),
            })
            time.sleep(0.05)

    def stop(self):
        self.pub.shutdown()

    # Foxglove server callbacks (ServerListener protocol)
    def on_client_advertise(self, client, channel):
        self.channels[channel.id] = channel.topic
        print(f'  drive: client advertised {channel.topic} ({channel.schema_name}, {channel.encoding})')

    def on_client_unadvertise(self, client, client_channel_id):
        self.channels.pop(client_channel_id, None)
        self.last_cmd = 0.0                    # stop immediately

    def on_message_data(self, client, client_channel_id, data):
        import json
        if self.channels.get(client_channel_id) == '/cmd_vel':
            try:
                self.on_twist(json.loads(data))
            except (ValueError, TypeError, AttributeError):
                pass

    def services(self):
        import json
        from foxglove import Schema, Service, ServiceSchema, MessageSchema
        def schema(name, props):
            return MessageSchema(encoding='json', schema=Schema(
                name=name, encoding='jsonschema',
                data=json.dumps({'type': 'object', 'properties': props}).encode()))
        trig = ServiceSchema(name='std_srvs/Trigger', request=schema('std_srvs/Trigger_Request', {}),
                             response=schema('std_srvs/Trigger_Response',
                                             {'success': {'type': 'boolean'}, 'message': {'type': 'string'}}))
        def estop(_req):
            self.estop = 'manual e-stop'; self.last_cmd = 0.0
            return json.dumps({'success': True, 'message': 'stopped'}).encode()
        def clear(_req):
            self.estop = None
            return json.dumps({'success': True, 'message': 'cleared'}).encode()
        return [Service(name='/drive/estop', schema=trig, handler=estop),
                Service(name='/drive/clear_estop', schema=trig, handler=clear)]


# ── Main ─────────────────────────────────────────────────────────────────────

SOURCES = [
    ('/lds/Lds', 'lds/Lds', on_lds),
    ('/robot/Robot', 'robot/Robot', on_robot),
]

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--record', metavar='FILE.mcap', help='also record everything to an MCAP file')
    ap.add_argument('--no-camera', action='store_true', help="don't stream the camera")
    ap.add_argument('--cam-size', default='640x480', metavar='WxH', help='camera resolution (default 640x480)')
    ap.add_argument('--drive', action='store_true',
                    help='accept geometry_msgs/Twist on /cmd_vel from the Foxglove Teleop panel and drive')
    ap.add_argument('--max-speed', type=float, default=100, metavar='MM_S', help='max wheel speed (default 100)')
    ap.add_argument('--max-turn', type=float, default=0.8, metavar='RAD_S', help='max turn rate (default 0.8)')
    args = ap.parse_args()

    ensure_master()
    drive = None
    server_kw = {}
    if args.drive:
        from foxglove import Capability
        drive = DriveControl(telemetry, args.max_speed, args.max_turn)
        drive.start()
        server_kw = dict(capabilities=[Capability.ClientPublish, Capability.Services],
                         supported_encodings=['json'], server_listener=drive,
                         services=drive.services())
    server = foxglove.start_server(name='deebot', host='127.0.0.1', port=args.port, **server_kw)
    mcap = foxglove.open_mcap(args.record, allow_overwrite=True) if args.record else None
    print(f'Foxglove server on ws://localhost:{args.port}' + (f', recording to {args.record}' if mcap else ''))

    for topic, typ, handler in SOURCES:
        threading.Thread(target=run_subscriber, args=(topic, typ, handler), daemon=True).start()
    if not args.no_camera:
        w, h = (int(v) for v in args.cam_size.lower().split('x'))
        threading.Thread(target=run_camera, args=(w, h), daemon=True).start()

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)       # also when started as a background job (SIGINT ignored)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if drive:
            drive.stop()                       # explicit zeros + unregister
        if mcap:
            mcap.close()
        server.stop()
        rw.cleanup_tunnels()
        print('\nstopped.')


if __name__ == '__main__':
    main()
