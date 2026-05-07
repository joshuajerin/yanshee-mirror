"""
Dance Challenge — say "dance" and the Yanshee performs a 10-second choreography
while scoring how well YOU match it. The robot watches via its onboard camera,
samples your pose at each keyframe, and compares your shoulder/elbow joint
angles to the expected pose. Final 0-100 score is announced over TTS.

Usage:
    ROBOT_IP=10.73.35.187 .venv/bin/python scripts/dance.py            # listen for voice
    ROBOT_IP=10.73.35.187 .venv/bin/python scripts/dance.py --start    # start immediately
    ROBOT_IP=10.73.35.187 .venv/bin/python scripts/dance.py --no-display
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
import urllib.parse
import urllib.request
from typing import Dict, List, Optional, Tuple

# ---- shared with mirror.py (kept inline so dance.py is standalone) ----------

SERVO_RANGES: Dict[str, Tuple[int, int]] = {
    "RightShoulderRoll": (0, 180),
    "RightShoulderFlex": (0, 180),
    "RightElbowFlex":    (0, 180),
    "LeftShoulderRoll":  (0, 180),
    "LeftShoulderFlex":  (0, 180),
    "LeftElbowFlex":     (0, 180),
    "NeckLR":            (15, 165),
}

# Direction flip per servo (must match mirror.py to stay consistent).
SERVO_DIR: Dict[str, int] = {
    "LeftShoulderRoll":  +1,
    "LeftShoulderFlex":  +1,
    "LeftElbowFlex":     +1,
    "RightShoulderRoll": -1,
    "RightShoulderFlex": -1,
    "RightElbowFlex":    -1,
}

ARM_NEUTRAL = {
    "RightShoulderRoll": 90, "RightShoulderFlex": 90, "RightElbowFlex": 180,
    "LeftShoulderRoll":  90, "LeftShoulderFlex":  90, "LeftElbowFlex":  180,
    "NeckLR": 90,
}

LM_L_SHOULDER, LM_R_SHOULDER = 11, 12
LM_L_ELBOW,    LM_R_ELBOW    = 13, 14
LM_L_WRIST,    LM_R_WRIST    = 15, 16


# ---- Yanshee REST helpers ---------------------------------------------------

def _put(ip: str, path: str, body: dict, timeout: float = 2.5) -> dict:
    data = json.dumps(body).encode()
    req = urllib.request.Request(f"http://{ip}:9090/v1{path}",
                                 data=data, method="PUT",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def _post(ip: str, path: str, body: Optional[dict] = None, timeout: float = 5.0) -> dict:
    data = json.dumps(body).encode() if body is not None else b""
    req = urllib.request.Request(f"http://{ip}:9090/v1{path}",
                                 data=data, method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def _delete(ip: str, path: str, timeout: float = 3.0) -> dict:
    req = urllib.request.Request(f"http://{ip}:9090/v1{path}", method="DELETE")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except Exception:
        return {}


def send_angles(ip: str, angles: Dict[str, int], runtime_ms: int) -> None:
    try:
        _put(ip, "/servos/angles", {"angles": angles, "runtime": runtime_ms})
    except Exception as e:
        print(f"[send err] {e}", file=sys.stderr)


def say(ip: str, text: str) -> None:
    try:
        _put(ip, "/voice/tts", {"tts": text, "interrupt": True, "timestamp": 0})
    except Exception as e:
        print(f"[tts err] {e}", file=sys.stderr)


def open_stream(ip: str, resolution: str = "320x240") -> None:
    _delete(ip, "/visions/streams")
    time.sleep(0.4)
    res = _post(ip, "/visions/streams", {"resolution": resolution})
    if res.get("code") != 0:
        raise RuntimeError(f"open_vision_stream failed: {res}")


# ---- voice ASR (waits for the user to say something containing 'dance') ----

def listen_for_keyword(ip: str, keyword: str = "dance", timeout_s: float = 30.0) -> Optional[str]:
    """Block until the robot's ASR returns text containing `keyword`, or timeout.
    Returns the recognized phrase or None."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ts = int(time.time())
        try:
            _put(ip, "/voice/asr", {"continues": False, "timestamp": ts})
        except Exception as e:
            print(f"[asr start err] {e}", file=sys.stderr)
            time.sleep(0.5)
            continue
        # Poll for completion
        for _ in range(40):
            time.sleep(0.25)
            try:
                req = urllib.request.Request(
                    f"http://{ip}:9090/v1/voice/asr?timestamp={ts}")
                with urllib.request.urlopen(req, timeout=2.0) as r:
                    state = json.loads(r.read().decode())
            except Exception:
                continue
            if state.get("status") in ("idle", "complete"):
                heard = (state.get("data") or {}).get("question") or ""
                if heard:
                    print(f"  heard: {heard!r}")
                    if keyword.lower() in heard.lower():
                        return heard
                break
    return None


# ---- pose math --------------------------------------------------------------

def clamp(v: float, lo: float, hi: float) -> int:
    return int(max(lo, min(hi, v)))


def angle_at(p_a, p_b, p_c) -> float:
    ax, ay = p_a.x - p_b.x, p_a.y - p_b.y
    cx, cy = p_c.x - p_b.x, p_c.y - p_b.y
    dot = ax * cx + ay * cy
    mag = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-9
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / mag))))


def shoulder_decompose(shoulder, elbow, side: str) -> Tuple[float, float]:
    dx = elbow.x - shoulder.x
    dy = elbow.y - shoulder.y
    dz = getattr(elbow, "z", 0.0) - getattr(shoulder, "z", 0.0)
    out_sign = 1.0 if side == "R" else -1.0
    side_x = out_sign * dx
    abduction = math.degrees(math.atan2(max(side_x, 0.0), dy))
    flexion = math.degrees(math.atan2(-dz, dy))
    return (max(0.0, min(180.0, abduction)),
            max(-90.0, min(180.0, flexion)))


def extract_user_pose(landmarks) -> Dict[str, float]:
    """Return user's joint angles in a dict matching the choreography keys."""
    R_sh, R_el, R_wr = landmarks[LM_R_SHOULDER], landmarks[LM_R_ELBOW], landmarks[LM_R_WRIST]
    L_sh, L_el, L_wr = landmarks[LM_L_SHOULDER], landmarks[LM_L_ELBOW], landmarks[LM_L_WRIST]
    R_abd, R_flex = shoulder_decompose(R_sh, R_el, "R")
    L_abd, L_flex = shoulder_decompose(L_sh, L_el, "L")
    R_elb = angle_at(R_sh, R_el, R_wr)
    L_elb = angle_at(L_sh, L_el, L_wr)
    return {"L_abd": L_abd, "L_flex": L_flex, "L_elb": L_elb,
            "R_abd": R_abd, "R_flex": R_flex, "R_elb": R_elb}


def expected_to_robot_servos(exp: Dict[str, float]) -> Dict[str, int]:
    """Mirror: user's L -> robot's RIGHT (with SERVO_DIR flip)."""
    raw = {
        "LeftShoulderRoll":  90 + exp["R_abd"],
        "LeftShoulderFlex":  90 + exp["R_flex"],
        "LeftElbowFlex":     exp["R_elb"],
        "RightShoulderRoll": 90 + exp["L_abd"],
        "RightShoulderFlex": 90 + exp["L_flex"],
        "RightElbowFlex":    exp["L_elb"],
    }
    out = {}
    for name, val in raw.items():
        flipped = 90 + SERVO_DIR.get(name, 1) * (val - 90)
        lo, hi = SERVO_RANGES[name]
        out[name] = clamp(flipped, lo, hi)
    return out


# ---- 10-second choreography -------------------------------------------------
# Each keyframe: (time_s, label, expected_user_pose). The robot moves into the
# pose by time_s; we sample the user 0.6s after to give them time to react.
# Angles are in MediaPipe-derived terms (shoulder abduction, flexion, elbow flex).

CHOREOGRAPHY: List[Tuple[float, str, Dict[str, float]]] = [
    (0.0, "GET READY",   {"L_abd":  0, "L_flex":  0, "L_elb":180, "R_abd":  0, "R_flex":  0, "R_elb":180}),
    (1.5, "T-POSE",      {"L_abd": 90, "L_flex":  0, "L_elb":180, "R_abd": 90, "R_flex":  0, "R_elb":180}),
    (3.0, "ARMS UP",     {"L_abd":150, "L_flex":  0, "L_elb":180, "R_abd":150, "R_flex":  0, "R_elb":180}),
    (4.5, "RIGHT UP",    {"L_abd":  0, "L_flex":  0, "L_elb":180, "R_abd":150, "R_flex":  0, "R_elb":180}),
    (6.0, "ARMS FORWARD",{"L_abd":  0, "L_flex": 90, "L_elb":180, "R_abd":  0, "R_flex": 90, "R_elb":180}),
    (7.5, "LEFT UP",     {"L_abd":150, "L_flex":  0, "L_elb":180, "R_abd":  0, "R_flex":  0, "R_elb":180}),
    (9.0, "FINISH",      {"L_abd":  0, "L_flex":  0, "L_elb":180, "R_abd":  0, "R_flex":  0, "R_elb":180}),
]
DURATION_S = 10.0
SAMPLE_DELAY_S = 0.6  # how long after each keyframe to sample the user


def score_pose(actual: Dict[str, float], expected: Dict[str, float]) -> Tuple[float, Dict[str, float]]:
    """Return (similarity_0_to_1, per-joint-percent dict). 90° off -> 0%, exact -> 100%."""
    diffs = {}
    sims = []
    for k in expected:
        d = abs(actual.get(k, 0.0) - expected[k])
        # Allow 10° free; degrade to 0 by 90°.
        sim = max(0.0, 1.0 - max(0.0, d - 10.0) / 80.0)
        sims.append(sim)
        diffs[k] = sim * 100.0
    return (sum(sims) / max(1, len(sims))), diffs


# ---- frame grabber (latest only) --------------------------------------------

class LatestFrame:
    def __init__(self, cap):
        self.cap = cap
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _loop(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame = f

    def get(self):
        with self._lock:
            return self._frame

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.5)


# ---- main dance routine -----------------------------------------------------

def run_dance(ip: str, show: bool = True) -> int:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mp_py
    from mediapipe.tasks.python import vision as mp_vision

    print("opening stream...")
    open_stream(ip, "320x240")
    cap = cv2.VideoCapture(f"http://{ip}:8000/stream.mjpg")
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    grabber = LatestFrame(cap).start()

    options = mp_vision.PoseLandmarkerOptions(
        base_options=mp_py.BaseOptions(model_asset_path="models/pose_landmarker_lite.task"),
        running_mode=mp_vision.RunningMode.IMAGE,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    detector = mp_vision.PoseLandmarker.create_from_options(options)

    # Wait for first frame
    for _ in range(40):
        if grabber.get() is not None:
            break
        time.sleep(0.05)

    # Countdown
    for n in (3, 2, 1):
        say(ip, f"{n}")
        time.sleep(0.7)
    say(ip, "Dance!")
    time.sleep(0.3)

    t0 = time.time()
    sampled: List[Tuple[str, float, Dict[str, float]]] = []  # (label, sim, diffs)
    next_kf_idx = 0

    print("\n=== DANCE ===")
    # We schedule: at each keyframe time, send servo command. At time + SAMPLE_DELAY, sample user.
    while True:
        elapsed = time.time() - t0
        if elapsed >= DURATION_S:
            break

        # Send next keyframe if its time is up
        if next_kf_idx < len(CHOREOGRAPHY) and elapsed >= CHOREOGRAPHY[next_kf_idx][0]:
            ts, label, exp = CHOREOGRAPHY[next_kf_idx]
            servos = expected_to_robot_servos(exp)
            # Movement runtime: until next keyframe, or until end if last one
            if next_kf_idx + 1 < len(CHOREOGRAPHY):
                rt = int((CHOREOGRAPHY[next_kf_idx + 1][0] - ts) * 1000) - 100
            else:
                rt = 1000
            rt = max(400, min(4000, rt))
            print(f"[{elapsed:5.2f}s] -> {label}  servos={servos}")
            send_angles(ip, servos, runtime_ms=rt)

            # Schedule a sample after SAMPLE_DELAY
            sample_at = ts + SAMPLE_DELAY_S
            # Wait, then sample
            while time.time() - t0 < sample_at:
                time.sleep(0.02)
            frame = grabber.get()
            if frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = detector.detect(mp_image)
                if result.pose_landmarks:
                    actual = extract_user_pose(result.pose_landmarks[0])
                    sim, diffs = score_pose(actual, exp)
                    sampled.append((label, sim, diffs))
                    print(f"        sample  sim={sim*100:5.1f}%  per-joint={ {k:f'{v:.0f}' for k,v in diffs.items()} }")
                else:
                    sampled.append((label, 0.0, {}))
                    print(f"        sample  NO POSE DETECTED (0%)")

                if show:
                    cv2.imshow("Dance Challenge", frame)
                    cv2.waitKey(1)
            next_kf_idx += 1
        else:
            # Idle frame display
            if show:
                f = grabber.get()
                if f is not None:
                    overlay = f.copy()
                    cv2.putText(overlay, f"DANCE!  {DURATION_S-elapsed:4.1f}s",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255), 2)
                    cv2.imshow("Dance Challenge", overlay)
                    cv2.waitKey(1)
            time.sleep(0.03)

    # Reset arms
    send_angles(ip, ARM_NEUTRAL, runtime_ms=1500)

    # Final score
    if sampled:
        avg = sum(s for _, s, _ in sampled) / len(sampled)
    else:
        avg = 0.0
    score = int(round(avg * 100))
    print(f"\n=== SCORE: {score}/100  (samples: {len(sampled)}) ===")
    for label, sim, _ in sampled:
        print(f"  {label:14s}  {sim*100:5.1f}%")

    # Pick a phrase
    if score >= 85:
        phrase = f"Wow, {score} out of 100. You are a dance machine!"
    elif score >= 65:
        phrase = f"Nice moves. {score} out of 100."
    elif score >= 40:
        phrase = f"You scored {score}. Keep practicing."
    else:
        phrase = f"Only {score}. You did not dance, you just stood there."
    say(ip, phrase)
    time.sleep(2.0)

    # Cleanup
    grabber.stop()
    cap.release()
    if show:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    _delete(ip, "/visions/streams")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", action="store_true",
                        help="start dance immediately (skip voice listen)")
    parser.add_argument("--no-display", action="store_true",
                        help="skip cv2 window")
    parser.add_argument("--keyword", default="dance",
                        help="ASR keyword to trigger (default 'dance')")
    args = parser.parse_args()

    ip = os.environ.get("ROBOT_IP")
    if not ip:
        print("ERROR: set ROBOT_IP env var", file=sys.stderr)
        return 1

    if not args.start:
        say(ip, f"Say 'robot {args.keyword}' to start.")
        print(f"listening for '{args.keyword}'... (Ctrl-C to cancel)")
        heard = listen_for_keyword(ip, args.keyword, timeout_s=60)
        if not heard:
            print("no keyword heard, exiting")
            return 0
        say(ip, "Let's go!")
        time.sleep(0.7)

    return run_dance(ip, show=not args.no_display)


if __name__ == "__main__":
    sys.exit(main())
