#!/usr/bin/env python3
"""
Ecovacs root password calculator.
Algorithm from Dennis Giese (dontvacuum.me).

Usage: python3 ecopassword.py <serial_number>
"""

import hashlib
import base64
import sys

MACHINES = {
    "goat": "mr2201",
    "x1": "x1",
    "x2": "x2",
    "t20": "t20",
    "t30": "t30",
    "airbot_z1": "airbot_z1",
    "9x0": "9x0",
    "t8": "px30-sl",
    "t8 AIVI": "AI_px30",
}

MAC = "d4:3d:7e:fa:12:5d:C8:02:8F:0A:E2:F5"


def calc_password(serial: str, machine: str) -> str:
    sn = serial.strip().upper()[-8:]
    raw = f"{machine}{MAC}{sn}\n"
    sha = hashlib.sha256(raw.encode()).hexdigest()
    pw = base64.b64encode(f"{sha}  -\n".encode()).decode()
    return pw


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_number> [machine_key]")
        print(f"       machine_key defaults to 't8' (px30-sl)")
        print(f"       available keys: {', '.join(MACHINES.keys())}")
        sys.exit(1)

    serial = sys.argv[1]
    key = sys.argv[2] if len(sys.argv) > 2 else "t8"

    if key not in MACHINES:
        print(f"Unknown machine key '{key}'. Choose from: {', '.join(MACHINES.keys())}")
        sys.exit(1)

    machine = MACHINES[key]
    password = calc_password(serial, machine)

    print(f"Serial (last 8): {serial.strip().upper()[-8:]}")
    print(f"Machine string:  {machine}")
    print(f"Root password:   {password}")


if __name__ == "__main__":
    main()