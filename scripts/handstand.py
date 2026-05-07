"""
Yanshee handstand attempt — multi-phase servo sequence over the REST API.

Hits PUT /v1/servos/angles directly. No SDK needed beyond `requests`.

Reality check: handstand is NOT a Yanshee preset. We sequence raw servo angles.
Arm servos (~5-6 kg-cm) are not designed to hold the ~2.05 kg body inverted.
Run on a soft surface, hold the robot through phase 4, e-stop ready.

Usage:
    ROBOT_IP=10.73.35.187 python scripts/handstand.py            # interactive
    ROBOT_IP=10.73.35.187 python scripts/handstand.py --auto     # full sequence
    python scripts/handstand.py --dry-run                         # print only
    ROBOT_IP=10.73.35.187 python scripts/handstand.py --phase 2  # one phase
    ROBOT_IP=10.73.35.187 python scripts/handstand.py --stop     # halt + reset
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from typing import Dict, List, Tuple


# Servo safe ranges per YanAPI docs. (servo_name -> (min, max))
SERVO_RANGES: Dict[str, Tuple[int, int]] = {
    "RightShoulderRoll": (0, 180),
    "RightShoulderFlex": (0, 180),
    "RightElbowFlex":    (0, 180),
    "LeftShoulderRoll":  (0, 180),
    "LeftShoulderFlex":  (0, 180),
    "LeftElbowFlex":     (0, 180),
    "RightHipLR":        (0, 120),
    "RightHipFB":        (10, 180),
    "RightKneeFlex":     (0, 180),
    "RightAnkleFB":      (0, 180),
    "RightAnkleUD":      (65, 180),
    "LeftHipLR":         (60, 180),
    "LeftHipFB":         (0, 170),
    "LeftKneeFlex":      (0, 180),
    "LeftAnkleFB":       (0, 180),
    "LeftAnkleUD":       (0, 115),
    "NeckLR":            (15, 165),
}

# 7-phase plan. (label, {servo: angle}, runtime_ms).
# Angles are first-pass guesses — tune live with --phase before --auto.
PHASES: List[Tuple[str, Dict[str, int], int]] = [
    ("0/reset — neutral standing pose", {
        "RightShoulderRoll": 90, "RightShoulderFlex": 90, "RightElbowFlex": 180,
        "LeftShoulderRoll":  90, "LeftShoulderFlex":  90, "LeftElbowFlex":  180,
        "RightHipLR": 60, "RightHipFB": 90, "RightKneeFlex": 180, "RightAnkleFB": 90, "RightAnkleUD": 120,
        "LeftHipLR":  120, "LeftHipFB":  90, "LeftKneeFlex":  180, "LeftAnkleFB":  90, "LeftAnkleUD":  60,
        "NeckLR": 90,
    }, 1500),

    ("1/deep crouch — lower hips, bend knees", {
        "RightHipFB": 150, "RightKneeFlex": 60,
        "LeftHipFB":  150, "LeftKneeFlex":  60,
        "RightAnkleFB": 60, "LeftAnkleFB": 60,
    }, 1500),

    ("2/plant hands — rotate shoulders fully forward, elbows straight", {
        "RightShoulderFlex": 180, "RightElbowFlex": 180,
        "LeftShoulderFlex":  180, "LeftElbowFlex":  180,
        "NeckLR": 90,
    }, 1200),

    ("3/hip lift — straighten legs while pivoting weight to arms", {
        "RightKneeFlex": 170, "LeftKneeFlex": 170,
        "RightHipFB": 170, "LeftHipFB": 170,
    }, 1500),

    ("4/kick up — drive legs vertical (CRITICAL: catch the robot)", {
        "RightHipFB": 30, "LeftHipFB": 30,
        "RightKneeFlex": 180, "LeftKneeFlex": 180,
        "RightAnkleFB": 90, "LeftAnkleFB": 90,
    }, 800),

    ("5/hold — minimal corrections (manual support recommended)", {}, 2000),

    ("6/recover — bring legs back down to crouch", {
        "RightHipFB": 150, "LeftHipFB": 150,
        "RightKneeFlex": 60, "LeftKneeFlex": 60,
    }, 1500),

    ("7/stand — return to neutral", {
        "RightShoulderFlex": 90, "RightElbowFlex": 180,
        "LeftShoulderFlex":  90, "LeftElbowFlex":  180,
        "RightHipFB": 90, "LeftHipFB": 90,
        "RightKneeFlex": 180, "LeftKneeFlex": 180,
        "RightAnkleFB": 90, "LeftAnkleFB": 90,
    }, 1500),
]


def _request(method: str, ip: str, path: str, body: dict | None = None, timeout: float = 8.0) -> dict:
    url = f"http://{ip}:9090/v1{path}"
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def validate(angles: Dict[str, int]) -> None:
    for name, angle in angles.items():
        if name not in SERVO_RANGES:
            raise ValueError(f"unknown servo: {name}")
        lo, hi = SERVO_RANGES[name]
        if not lo <= angle <= hi:
            raise ValueError(f"{name}={angle}° outside safe range [{lo},{hi}]")


def set_angles(ip: str, angles: Dict[str, int], runtime_ms: int) -> dict:
    return _request("PUT", ip, "/servos/angles",
                    {"angles": angles, "runtime": runtime_ms})


def play_preset(ip: str, name: str, speed: str = "normal", repeat: int = 1) -> dict:
    return _request("PUT", ip, "/motions",
                    {"operation": "start",
                     "motion": {"name": name, "repeat": repeat, "speed": speed},
                     "timestamp": 0, "version": "v1"})


def stop_motion(ip: str) -> dict:
    return _request("PUT", ip, "/motions",
                    {"operation": "stop",
                     "motion": {"name": "", "repeat": 1, "speed": "normal"},
                     "timestamp": 0, "version": "v1"})


def run_phase(ip: str | None, label: str, angles: Dict[str, int], runtime_ms: int, dry_run: bool) -> None:
    print(f"\n>>> {label}")
    if not angles:
        print("    (no servo changes — pause)")
    for name, angle in angles.items():
        print(f"    {name:24s} → {angle}°")
    print(f"    runtime: {runtime_ms}ms")
    if dry_run:
        return
    validate(angles)
    if angles:
        result = set_angles(ip, angles, runtime_ms)
        if result.get("code") != 0:
            print(f"    !! API: {result}", file=sys.stderr)
    time.sleep(runtime_ms / 1000.0 + 0.1)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--auto", action="store_true", help="run all phases without prompting")
    parser.add_argument("--dry-run", action="store_true", help="print only, no API calls")
    parser.add_argument("--phase", type=int, help="run only this phase index")
    parser.add_argument("--stop", action="store_true", help="halt motion and play Reset preset")
    args = parser.parse_args()

    ip = os.environ.get("ROBOT_IP")
    if not args.dry_run and not ip:
        print("ERROR: set ROBOT_IP env var", file=sys.stderr)
        return 1

    if args.stop:
        print("stopping motion + playing Reset…")
        stop_motion(ip)
        time.sleep(0.3)
        play_preset(ip, "Reset")
        return 0

    phases = PHASES if args.phase is None else [PHASES[args.phase]]
    for i, (label, angles, runtime) in enumerate(phases):
        idx = args.phase if args.phase is not None else i
        if not args.auto and not args.dry_run:
            try:
                input(f"\n[ENTER] to run phase {idx} — '{label}' (Ctrl-C to abort) ")
            except KeyboardInterrupt:
                print("\naborted")
                return 0
        run_phase(ip, label, angles, runtime, args.dry_run)

    if not args.dry_run:
        print("\nDone. Playing Reset to settle.")
        play_preset(ip, "Reset")
    return 0


if __name__ == "__main__":
    sys.exit(main())
