#!/usr/bin/env python3
"""
Live /robot/Robot decoder.

Connects to the robot's ROS master via SSH tunnel, automatically creates
additional tunnels for the node XML-RPC port and TCPROS publisher port,
then subscribes to /robot/Robot and renders a live terminal dashboard.

Setup (run once before starting this script):
    ssh -f -N -L 11311:127.0.0.1:11311 <ROBOT_SSH from config.py>

Usage:
    python3 robot_watch.py
"""

import argparse
import json
import signal
import socket
import struct
import subprocess
import sys
import time
import xmlrpc.client
from urllib.parse import urlparse

from config import ROBOT_SSH as ROBOT, ROS_MASTER_PORT

MASTER_URL  = f'http://localhost:{ROS_MASTER_PORT}'
CALLER_ID   = '/robot_watch'
TOPIC       = '/robot/Robot'
TOPIC_TYPE  = 'robot/Robot'

# ── SSH tunnel helpers ───────────────────────────────────────────────────────

_tunnels: list[subprocess.Popen] = []

def _free_port() -> int:
    """Ask the OS for a free local port."""
    with socket.socket() as s:
        s.bind(('', 0))
        return s.getsockname()[1]

def open_tunnel(robot_port: int) -> int:
    """Forward localhost:<free_port> → robot's 127.0.0.1:<robot_port>."""
    local = _free_port()
    proc = subprocess.Popen(
        ['ssh', '-N', '-o', 'StrictHostKeyChecking=no',
         '-L', f'{local}:127.0.0.1:{robot_port}', ROBOT],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    _tunnels.append(proc)
    # Poll until the port is accepting connections (up to 5s)
    for _ in range(25):
        try:
            with socket.create_connection(('localhost', local), timeout=0.2):
                pass
            break
        except OSError:
            time.sleep(0.2)
    return local

def cleanup_tunnels():
    for p in _tunnels:
        try:
            p.terminate()
        except Exception:
            pass

# ── Buffer / decoder ─────────────────────────────────────────────────────────

class Buf:
    def __init__(self, data: bytes):
        self.data = data
        self.pos  = 0

    def read(self, n: int) -> bytes:
        v = self.data[self.pos:self.pos + n]
        self.pos += n
        return v

    def u8(self)  -> int:   return struct.unpack('B',  self.read(1))[0]
    def u16(self) -> int:   return struct.unpack('<H', self.read(2))[0]
    def u32(self) -> int:   return struct.unpack('<I', self.read(4))[0]
    def f32(self) -> float: return struct.unpack('<f', self.read(4))[0]

    def string(self) -> str:
        return self.read(self.u32()).decode('utf-8', errors='replace')

    def header(self) -> dict:
        return {'seq': self.u32(), 'secs': self.u32(),
                'nsecs': self.u32(), 'frame_id': self.string()}

    def pose(self) -> dict:
        h = self.header()
        return {'h': h, 'x': self.f32(), 'y': self.f32(), 'theta': self.f32()}

    def predict_pose(self) -> dict:
        return {'predict': self.pose(), 'pose': self.pose()}

    def sensor_array(self) -> list[tuple[int, int]]:
        return [(self.u8(), self.u8()) for _ in range(self.u32())]


CHARGE_STATES = {0: 'NOT CHARGING', 1: 'CHARGING', 2: 'FULL',
                 3: 'ERROR', 4: 'ERROR+OFF'}
MOTOR_NAMES   = {0: 'mainbrush', 1: 'L-side', 2: 'R-side',
                 3: 'fan', 4: 'pump', 5: 'L-wheel', 6: 'R-wheel'}
RANGE_NAMES   = {0: 'front_buf', 1: 'side_in', 2: 'down_in', 3: 'ultrasound'}
BUMP_NAMES    = {0: 'L', 1: 'R', 2: 'LDS'}


def decode(raw: bytes) -> dict:
    b = Buf(raw)

    header       = b.header()
    pose         = b.predict_pose()
    battery      = b.u8()
    low_volt     = b.u8()
    on_charger   = b.u8()
    charge_state = b.u8()

    motors = {}
    for _ in range(b.u32()):
        t = b.u8();  motors[MOTOR_NAMES.get(t, str(t))] = b.u16()

    b.header()   # rangeDet header
    ranges = {}
    for _ in range(b.u32()):
        t = b.u8();  nv = b.u32()
        ranges[RANGE_NAMES.get(t, str(t))] = [b.u16() for _ in range(nv)]

    onoff = {t: v for t, v in b.sensor_array()}
    bump  = {BUMP_NAMES.get(t, str(t)): v for t, v in b.sensor_array()}
    downin    = [v for _, v in b.sensor_array()]
    fall      = [v for _, v in b.sensor_array()]
    dirtbox   = next((v for _, v in b.sensor_array()), 0)
    carpet    = next((v for _, v in b.sensor_array()), 0)
    waterbox  = next((v for _, v in b.sensor_array()), 0)

    lds_hdr = b.header()
    n_pts   = b.u32()
    lds_pts = [(b.f32(), b.f32(), b.f32(), b.f32(), b.f32()) for _ in range(n_pts)]
    b.predict_pose()   # ldsPose (skip)

    return {
        'seq'         : header['seq'],
        'pose'        : pose['pose'],
        'predict'     : pose['predict'],
        'battery'     : battery,
        'low_volt'    : low_volt,
        'on_charger'  : on_charger,
        'charge_state': charge_state,
        'motors'      : motors,
        'ranges'      : ranges,
        'bump'        : bump,
        'downin'      : downin,
        'fall'        : fall,
        'dirtbox'     : dirtbox,
        'carpet'      : carpet,
        'waterbox'    : waterbox,
        'lds_secs'    : lds_hdr['secs'],
        'lds_nsecs'   : lds_hdr['nsecs'],
        'lds_count'   : n_pts,
        'lds_pts'     : lds_pts,
    }

# ── Display ──────────────────────────────────────────────────────────────────

def _bar(pct: int, w: int = 20) -> str:
    filled = round(pct / 100 * w)
    return '[' + '█' * filled + '░' * (w - filled) + f'] {pct:3d}%'

def render(d: dict, hz: float):
    p  = d['pose']
    pp = d['predict']
    W  = 64

    lines = [
        f"{'Ecovacs T8 AIVI — /robot/Robot':^{W}}",
        f"{'seq #' + str(d['seq']):>{W//2}}{'  {:.1f} Hz'.format(hz):>{W//2}}",
        '─' * W,
        '  POWER',
    ]

    batt_bar = _bar(d['battery'])
    low_flag = '  ⚠ LOW VOLTAGE' if d['low_volt'] else ''
    lines.append(f'    Battery   {batt_bar}{low_flag}')

    chg_type  = {0: 'off charger', 1: 'standard charger', 3: '3D charger'}.get(d['on_charger'], '?')
    chg_state = CHARGE_STATES.get(d['charge_state'], str(d['charge_state']))
    lines.append(f'    Charger   {chg_type}   {chg_state}')
    lines.append('')

    lines += [
        '  POSITION  (corrected pose)',
        f'    x = {p["x"]:+.3f} m     y = {p["y"]:+.3f} m     θ = {p["theta"]:+.4f} rad',
        f'    predict  x = {pp["x"]:+.3f} m     y = {pp["y"]:+.3f} m',
        '',
        '  SENSORS',
    ]

    bump_str  = '  '.join(f'{k}={v}' for k, v in sorted(d['bump'].items()))
    fall_str  = '  '.join(f'[{i}]={v}' for i, v in enumerate(d['fall']))
    lines.append(f'    Bump  {bump_str}           Fall  {fall_str}')
    lines.append(f'    Cliff  [{" ".join(str(v) for v in d["downin"])}]   (6 downward IR)')
    lines.append(f'    Dirtbox={d["dirtbox"]}   Carpet={d["carpet"]}   Waterbox={d["waterbox"]}')
    lines.append('')

    lines.append('  RANGE SENSORS')
    for name in ('front_buf', 'side_in', 'down_in', 'ultrasound'):
        vals = d['ranges'].get(name, [])
        lines.append(f'    {name:<14} {vals}')
    lines.append('')

    lines.append('  MOTORS (mA)')
    row1 = '  '.join(f'{k}={d["motors"].get(k, 0):3d}'
                     for k in ('mainbrush', 'L-side', 'R-side'))
    row2 = '  '.join(f'{k}={d["motors"].get(k, 0):3d}'
                     for k in ('fan', 'pump', 'L-wheel', 'R-wheel'))
    lines += [f'    {row1}', f'    {row2}', '']

    t_lds = d['lds_secs'] + d['lds_nsecs'] / 1e9
    lines.append('  LDS SCAN')
    lines.append(f'    {d["lds_count"]} points   t = {t_lds:.3f} s')
    valid = [pt for pt in d['lds_pts'] if pt[4] > 0]
    if valid:
        rhos = [pt[2] for pt in valid]
        lines.append(f'    rho  min={min(rhos):.0f} mm   max={max(rhos):.0f} mm   '
                     f'{len(valid)} valid pts')
    lines.append('─' * W)

    sys.stdout.write('\033[H' + '\n'.join(lines) + '\033[J')
    sys.stdout.flush()

# ── TCPROS subscribe helpers ─────────────────────────────────────────────────

def _recv_all(sock: socket.socket, n: int) -> bytes:
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError('socket closed')
        buf += chunk
    return buf

def _read_framed(sock: socket.socket) -> bytes:
    sz = struct.unpack('<I', _recv_all(sock, 4))[0]
    return _recv_all(sock, sz)

def _encode_conn_header(fields: dict) -> bytes:
    parts = []
    for k, v in fields.items():
        line = (k + '=' + v).encode()
        parts.append(struct.pack('<I', len(line)) + line)
    data = b''.join(parts)
    return struct.pack('<I', len(data)) + data

# ── Main ─────────────────────────────────────────────────────────────────────

def _connect() -> socket.socket:
    """Set up all SSH tunnels and return a connected, subscribed TCPROS socket."""
    print('Checking master tunnel on localhost:11311 ...')
    try:
        master = xmlrpc.client.ServerProxy(MASTER_URL)
        master.getSystemState('/')
    except Exception:
        print('  Not reachable — starting tunnel ...')
        proc = subprocess.Popen(
            ['ssh', '-N', '-o', 'StrictHostKeyChecking=no',
             '-L', f'{ROS_MASTER_PORT}:127.0.0.1:{ROS_MASTER_PORT}', ROBOT],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        _tunnels.append(proc)
        for _ in range(25):
            try:
                with socket.create_connection(('localhost', ROS_MASTER_PORT), timeout=0.2):
                    pass
                break
            except OSError:
                time.sleep(0.2)
        master = xmlrpc.client.ServerProxy(MASTER_URL)

    print('Looking up /node ...')
    code, msg, node_uri = master.lookupNode(CALLER_ID, '/node')
    if code != 1:
        sys.exit(f'lookupNode failed: {msg}')

    robot_node_port  = urlparse(node_uri).port
    local_node_port  = open_tunnel(robot_node_port)
    print(f'  XML-RPC  robot:{robot_node_port} → local:{local_node_port}')

    node = xmlrpc.client.ServerProxy(f'http://localhost:{local_node_port}')
    code, msg, proto = node.requestTopic(CALLER_ID, TOPIC, [['TCPROS']])
    if code != 1:
        sys.exit(f'requestTopic failed: {msg}')

    robot_tcpros_port = proto[2]
    local_tcpros_port = open_tunnel(robot_tcpros_port)
    print(f'  TCPROS   robot:{robot_tcpros_port} → local:{local_tcpros_port}')

    sock = socket.socket()
    sock.connect(('localhost', local_tcpros_port))
    sock.sendall(_encode_conn_header({
        'callerid'   : CALLER_ID,
        'topic'      : TOPIC,
        'type'       : TOPIC_TYPE,
        'md5sum'     : '*',
        'tcp_nodelay': '0',
    }))
    _read_framed(sock)   # consume publisher connection header
    print('Subscribed.\n')
    return sock


def _print_line(d: dict):
    """Single compact line per message — good for streaming / piping."""
    p = d['pose']
    chg = CHARGE_STATES.get(d['charge_state'], '?')
    bump = ''.join(k for k, v in sorted(d['bump'].items()) if v)
    print(
        f"seq={d['seq']:>8}  "
        f"bat={d['battery']:>3d}%{'⚡' if d['on_charger'] else '  '}  "
        f"pos=({p['x']:+.2f},{p['y']:+.2f},{p['theta']:+.3f})  "
        f"bump=[{bump or '-'}]  "
        f"LDS={d['lds_count']}pts  "
        f"{chg}"
    )


def main(watch: bool = False, log_path: str | None = None, duration: float | None = None):
    def _exit(sig=None, frame=None):
        cleanup_tunnels()
        if watch:
            sys.stdout.write('\033[?25h\n')   # restore cursor
        sys.exit(0)

    signal.signal(signal.SIGINT,  _exit)
    signal.signal(signal.SIGTERM, _exit)

    sock       = _connect()
    log_file   = open(log_path, 'w') if log_path else None
    start_time = time.time()
    last_time  = start_time
    hz         = 0.0
    count      = 0

    if watch:
        sys.stdout.write('\033[?25l\033[2J')   # hide cursor, clear screen
        sys.stdout.flush()

    try:
        while True:
            if duration and (time.time() - start_time) >= duration:
                break

            raw = _read_framed(sock)
            try:
                d = decode(raw)
            except Exception as e:
                print(f'decode error: {e}', file=sys.stderr)
                continue

            if log_file:
                log_file.write(json.dumps(d) + '\n')
                log_file.flush()

            count += 1
            now = time.time()
            if now - last_time >= 0.5:
                hz        = count / (now - last_time)
                count     = 0
                last_time = now

            if watch:
                render(d, hz)
            else:
                _print_line(d)

    finally:
        if log_file:
            log_file.close()
        _exit()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='/robot/Robot decoder')
    ap.add_argument('--watch',    action='store_true', help='top-like live dashboard (refreshes in place)')
    ap.add_argument('--log',      metavar='FILE',      help='write decoded messages as JSON lines to FILE')
    ap.add_argument('--duration', metavar='SECS',      type=float, default=None, help='exit after N seconds')
    args = ap.parse_args()
    main(watch=args.watch, log_path=args.log, duration=args.duration)
