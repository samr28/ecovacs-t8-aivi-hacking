#!/usr/bin/env python3
"""
Robot → Foxglove bridge (the start of our debug GUI).

Subscribes to the robot's ROS topics over SSH tunnels (same TCPROS approach as robot_watch.py),
converts them to Foxglove's ROS-style schemas, and serves them on a Foxglove WebSocket server.

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
from foxglove.messages import (CompressedVideo, FrameTransform, FrameTransforms, PackedElementField,
                               PackedElementFieldNumericType, PointCloud, Pose, PosesInFrame,
                               Quaternion, Timestamp, Vector3)

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


# ── Main ─────────────────────────────────────────────────────────────────────

SOURCES = [
    ('/lds/Lds', 'lds/Lds', on_lds),
]

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--record', metavar='FILE.mcap', help='also record everything to an MCAP file')
    ap.add_argument('--no-camera', action='store_true', help="don't stream the camera")
    ap.add_argument('--cam-size', default='640x480', metavar='WxH', help='camera resolution (default 640x480)')
    args = ap.parse_args()

    ensure_master()
    server = foxglove.start_server(name='deebot', host='127.0.0.1', port=args.port)
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
        if mcap:
            mcap.close()
        server.stop()
        rw.cleanup_tunnels()
        print('\nstopped.')


if __name__ == '__main__':
    main()
