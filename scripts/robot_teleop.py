#!/usr/bin/env python3
"""
Keyboard teleop for the Deebot — WASD / arrow keys — from the Mac.

How it drives
-------------
Publishes wheel/SetWheelSpeed ([left, right], speed_type DRIVING) at 20 Hz directly to
/node's wheel driver (common::Wheel), which forwards it to the motor MCU with no task-state
gating. This bypasses the task manager, so the firmware's own bump/cliff reactions do NOT
apply — this script watches /robot/Robot and e-stops itself on bump / wheel-drop / cliff.

(Dead ends, for the record: mdsctl rosnode {"todo":"setspeed"} is a stub; publishing to
/subtasks/remotecontrol_task/control is dropped by the session protocol; /task/WorkManage
START workType=9 returns OK but never starts the remote task. See notes/driving.md.)

Units: commanded wheel value ≈ 6 × actual mm/s (measured 2026-11-10: 300 → ~50 mm/s).
Odometry and /robot/Robot pose are in mm.

ROS plumbing (standard ROS 1 publisher, done by hand like rospy would):
  Mac XML-RPC server  ◄─ reverse ssh tunnel ─  /node calls requestTopic()
  Mac TCPROS server   ◄─ reverse ssh tunnel ─  /node connects and receives messages
  master :11311 and /robot/Robot telemetry via forward tunnels (see robot_watch.py)

Keys (hold to move, release to stop)
  w / ↑   forward          q / e   arc forward-left / forward-right
  s / ↓   backward         a / ←   spin left      d / →   spin right
  space   E-STOP (also clears a safety stop)
  + / -   faster / slower  x / Ctrl-C   quit
"""

import argparse
import os
import random
import select
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty
import xmlrpc.client
from xmlrpc.server import SimpleXMLRPCServer

import robot_watch as rw
from config import ROBOT_SSH, ROS_MASTER_PORT

MASTER_URL = f'http://localhost:{ROS_MASTER_PORT}'
CALLER_ID  = '/robot_teleop'
TOPIC      = '/wheel/SetWheelSpeed'
TOPIC_TYPE = 'wheel/SetWheelSpeed'
MD5SUM     = '6136b114420b8e6600151ac57541e32e'

CMD_PER_MMPS   = 6.0     # empirical: wheel command units per mm/s of real speed
PUB_HZ         = 20
SPEED_MMPS     = 50      # default linear speed (mm/s) — 50 is what we've tested
TURN_MMPS      = 30      # wheel speed during spins (mm/s per wheel)
SPEED_STEP     = 25
SPEED_MAX      = 200     # hard cap (mm/s); robot cleans at ~300
FIRST_HOLD_S   = 0.6     # grace after first keypress (covers OS key-repeat delay)
REPEAT_HOLD_S  = 0.15    # once key-repeat is flowing, stop this soon after release
TELEMETRY_STALE_S = 1.0  # stop if /robot/Robot goes quiet


# ── Publisher (Mac-side TCPROS + XML-RPC, reached by /node via reverse tunnels) ──

def _reverse_tunnel(local_port: int) -> int:
    """robot 127.0.0.1:<robot_port> → Mac 127.0.0.1:<local_port>. Returns robot_port."""
    for _ in range(5):
        robot_port = random.randint(40000, 59999)
        p = subprocess.Popen(
            ['ssh', '-N', '-o', 'ExitOnForwardFailure=yes', '-o', 'StrictHostKeyChecking=no',
             '-R', f'{robot_port}:127.0.0.1:{local_port}', ROBOT_SSH],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.8)
        if p.poll() is None:            # still running → forward was accepted
            rw._tunnels.append(p)
            return robot_port
    sys.exit('could not open reverse tunnel')


class WheelPublisher:
    def __init__(self):
        self.subs: list[socket.socket] = []
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)           # (left, right) in wheel-command units
        self.running = True

    def start(self):
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('127.0.0.1', 0)); srv.listen(5)
        self._srv = srv
        threading.Thread(target=self._accept, daemon=True).start()
        robot_tcpros = _reverse_tunnel(srv.getsockname()[1])

        x = SimpleXMLRPCServer(('127.0.0.1', 0), logRequests=False, allow_none=True)
        x.register_function(lambda *a: [1, 'ready', ['TCPROS', '127.0.0.1', robot_tcpros]], 'requestTopic')
        x.register_function(lambda *a: [1, '', []], 'getBusInfo')
        x.register_function(lambda *a: [1, '', MASTER_URL], 'getMasterUri')
        x.register_function(lambda *a: [1, '', 0], 'publisherUpdate')
        x.register_function(lambda *a: [1, '', 0], 'shutdown')
        x.register_function(lambda *a: [1, '', os.getpid()], 'getPid')
        threading.Thread(target=x.serve_forever, daemon=True).start()
        self._xmlrpc = x
        robot_xmlrpc = _reverse_tunnel(x.server_address[1])

        self.caller_api = f'http://127.0.0.1:{robot_xmlrpc}/'
        code, msg, _ = xmlrpc.client.ServerProxy(MASTER_URL).registerPublisher(
            CALLER_ID, TOPIC, TOPIC_TYPE, self.caller_api)
        if code != 1:
            sys.exit(f'registerPublisher failed: {msg}')
        threading.Thread(target=self._loop, daemon=True).start()

    def _accept(self):
        while True:
            try:
                c, _ = self._srv.accept()
                rw._read_framed(c)      # subscriber's connection header
                c.sendall(rw._encode_conn_header({
                    'callerid': CALLER_ID, 'topic': TOPIC, 'type': TOPIC_TYPE,
                    'md5sum': MD5SUM, 'latching': '0'}))
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.lock:
                    self.subs.append(c)
            except Exception:
                if not self.running:
                    return

    def _send(self, left: float, right: float):
        body = struct.pack('<BI2f', 0, 2, left, right)   # speed_type=DRIVING, float32[2]
        frame = struct.pack('<I', len(body)) + body
        with self.lock:
            for s in list(self.subs):
                try:
                    s.sendall(frame)
                except OSError:
                    self.subs.remove(s)

    def _loop(self):
        while self.running:
            self._send(*self.cmd)
            time.sleep(1 / PUB_HZ)

    def connected(self) -> int:
        with self.lock:
            return len(self.subs)

    def shutdown(self):
        self.cmd = (0.0, 0.0)
        self.running = False
        for _ in range(10):             # 0.5 s of explicit zeros
            self._send(0.0, 0.0); time.sleep(0.05)
        try:
            xmlrpc.client.ServerProxy(MASTER_URL).unregisterPublisher(CALLER_ID, TOPIC, self.caller_api)
        except Exception:
            pass


# ── Telemetry (/robot/Robot) ─────────────────────────────────────────────────

class Telemetry:
    def __init__(self):
        self.d = None
        self.t = 0.0

    def start(self):
        sock = rw._connect()
        def loop():
            while True:
                try:
                    self.d = rw.decode(rw._read_framed(sock)); self.t = time.time()
                except Exception:
                    time.sleep(0.1)
        threading.Thread(target=loop, daemon=True).start()

    def hazard(self) -> str | None:
        d = self.d
        if d is None or time.time() - self.t > TELEMETRY_STALE_S:
            return 'telemetry stale'
        if any(d['bump'].values()):
            return 'BUMP ' + ''.join(k for k, v in d['bump'].items() if v)
        if any(d['fall']):
            return 'WHEEL DROP'
        if any(d['downin']):
            return 'CLIFF'
        return None


# ── Keyboard ─────────────────────────────────────────────────────────────────

KEYMAP = {
    'w': 'fwd', '\x1b[A': 'fwd', 's': 'back', '\x1b[B': 'back',
    'a': 'left', '\x1b[D': 'left', 'd': 'right', '\x1b[C': 'right',
    'q': 'arcl', 'e': 'arcr',
}

def read_keys(timeout: float) -> list[str]:
    """Non-blocking read of pending keypresses (arrow keys come as 3-byte escapes)."""
    keys = []
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return keys
    buf = os.read(sys.stdin.fileno(), 64).decode(errors='ignore')
    i = 0
    while i < len(buf):
        if buf[i] == '\x1b' and buf[i + 1:i + 2] == '[' and i + 2 < len(buf):
            keys.append(buf[i:i + 3]); i += 3
        else:
            keys.append(buf[i]); i += 1
    return keys


def wheel_cmd(action: str, speed: float, turn: float) -> tuple[float, float]:
    v, w = speed * CMD_PER_MMPS, turn * CMD_PER_MMPS
    return {
        'fwd':   (v, v),       'back':  (-v, -v),
        'left':  (-w, w),      'right': (w, -w),
        'arcl':  (v * 0.5, v), 'arcr':  (v, v * 0.5),
    }.get(action, (0.0, 0.0))


def status_line(tel: Telemetry, pub: WheelPublisher, action, speed, estop) -> str:
    d = tel.d
    if d:
        p = d['pose']
        m = d['motors']
        tele = (f"bat {d['battery']:3d}%  pose ({p['x']:+7.0f},{p['y']:+7.0f}) mm "
                f"θ {p['theta']:+.2f}  wheel mA {m.get('L-wheel', 0):3d}/{m.get('R-wheel', 0):3d}")
    else:
        tele = 'waiting for telemetry...'
    state = f'E-STOP: {estop}' if estop else (action or 'idle')
    return (f"\r\033[K[{state:^18}] speed {speed:3d} mm/s  subs {pub.connected()}  {tele}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    global SPEED_MMPS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=int, default=SPEED_MMPS, help='initial speed mm/s')
    ap.add_argument('--turn', type=int, default=TURN_MMPS, help='spin wheel speed mm/s')
    args = ap.parse_args()
    speed, turn = min(args.speed, SPEED_MAX), min(args.turn, SPEED_MAX)

    tel = Telemetry(); tel.start()
    pub = WheelPublisher(); pub.start()
    print('Waiting for /node to subscribe to our publisher ...')
    for _ in range(50):
        if pub.connected():
            break
        time.sleep(0.1)
    if not pub.connected():
        pub.shutdown(); rw.cleanup_tunnels()
        sys.exit('/node never connected — check reverse tunnels (dropbear -R support?)')

    print(__doc__.split('Keys')[1].split('"""')[0])
    old = termios.tcgetattr(sys.stdin)

    done = []
    def quit_(*_):
        if done:
            sys.exit(0)
        done.append(1)
        pub.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        rw.cleanup_tunnels()
        print('\nstopped.')
        sys.exit(0)
    signal.signal(signal.SIGTERM, quit_)

    action, held_until, repeating, estop, prev_hz = None, 0.0, False, None, None
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.time()
            for k in read_keys(0.02):
                if k in ('x', '\x03'):
                    quit_()
                elif k == ' ':
                    action, estop = None, None          # stop, and clear any safety stop
                elif k in '+=':
                    speed = min(speed + SPEED_STEP, SPEED_MAX)
                elif k in '-_':
                    speed = max(speed - SPEED_STEP, SPEED_STEP)
                elif k in KEYMAP:
                    a = KEYMAP[k]
                    repeating = (a == action)           # same key again → OS auto-repeat
                    action = a
                    held_until = now + (REPEAT_HOLD_S if repeating else FIRST_HOLD_S)

            if action and now > held_until:             # key released
                action = None

            # Safety: a hazard latches an e-stop when it *appears* (rising edge). Space clears it,
            # after which you can drive again even if the bumper is still pressed — so you can
            # back away from the wall. Stale telemetry blocks motion but doesn't latch.
            hz = tel.hazard()
            stale = hz == 'telemetry stale'
            if hz and not stale and hz != prev_hz:
                estop, action = hz, None
            prev_hz = hz

            moving = action and not estop and not stale
            pub.cmd = wheel_cmd(action, speed, turn) if moving else (0.0, 0.0)
            sys.stdout.write(status_line(tel, pub, action if moving else None, speed,
                                         estop or ('no telemetry' if stale else None)))
            sys.stdout.flush()
    except KeyboardInterrupt:
        quit_()
    finally:
        quit_()


if __name__ == '__main__':
    main()
