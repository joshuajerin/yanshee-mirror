"""
Yanshee mirror — robot mimics a person seen by ITS OWN camera, mirrored.

Pipeline:
  Yanshee MJPEG stream (http://<IP>:8000/stream.mjpg)
    -> MediaPipe Tasks PoseLandmarker (33 body landmarks)
    -> joint angles
    -> mirrored servo commands (arms only)
    -> PUT /v1/servos/angles at SEND_HZ

Mirror semantics: person's RIGHT arm drives robot's LEFT arm and vice-versa,
so it looks like a literal mirror image.

Only ARMS + neck are driven. Legs are NEVER touched (we don't even send the
neutral leg pose) so whatever standing position the robot is in stays put.

Usage:
    ROBOT_IP=10.73.35.187 .venv/bin/python scripts/mirror.py
    ROBOT_IP=10.73.35.187 .venv/bin/python scripts/mirror.py --rate 1
    ROBOT_IP=10.73.35.187 .venv/bin/python scripts/mirror.py --photo-mode --interval 5

Keys: q quit · p/space pause · r send Reset preset
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
import threading
from typing import Dict, List, Optional, Tuple

# Servo safe ranges (arms + neck only — we never touch legs).
SERVO_RANGES: Dict[str, Tuple[int, int]] = {
    "RightShoulderRoll": (0, 180),
    "RightShoulderFlex": (0, 180),
    "RightElbowFlex":    (0, 180),
    "LeftShoulderRoll":  (0, 180),
    "LeftShoulderFlex":  (0, 180),
    "LeftElbowFlex":     (0, 180),
    "NeckLR":            (15, 165),
}

# Neutral arm pose used on startup and exit (legs untouched).
ARM_NEUTRAL = {
    "RightShoulderRoll": 90, "RightShoulderFlex": 90, "RightElbowFlex": 180,
    "LeftShoulderRoll":  90, "LeftShoulderFlex":  90, "LeftElbowFlex":  180,
    "NeckLR": 90,
}

# Per-servo direction flip — set to -1 to invert a joint if it moves the wrong way
# during live calibration. Multiplies (target - 90) so neutral 90 stays neutral.
SERVO_DIR: Dict[str, int] = {
    "LeftShoulderRoll":  +1,
    "LeftShoulderFlex":  +1,
    "LeftElbowFlex":     +1,
    "RightShoulderRoll": -1,
    "RightShoulderFlex": -1,
    "RightElbowFlex":    -1,
}

# MediaPipe Pose landmark indices (33 body landmarks).
LM_NOSE = 0
LM_L_SHOULDER, LM_R_SHOULDER = 11, 12
LM_L_ELBOW,    LM_R_ELBOW    = 13, 14
LM_L_WRIST,    LM_R_WRIST    = 15, 16
LM_L_HIP,      LM_R_HIP      = 23, 24
LM_L_KNEE,     LM_R_KNEE     = 25, 26
LM_L_ANKLE,    LM_R_ANKLE    = 27, 28

# Subset of POSE_CONNECTIONS for drawing skeleton (kept short).
POSE_CONNECTIONS: List[Tuple[int, int]] = [
    (LM_L_SHOULDER, LM_R_SHOULDER),
    (LM_L_SHOULDER, LM_L_ELBOW), (LM_L_ELBOW, LM_L_WRIST),
    (LM_R_SHOULDER, LM_R_ELBOW), (LM_R_ELBOW, LM_R_WRIST),
    (LM_L_SHOULDER, LM_L_HIP),   (LM_R_SHOULDER, LM_R_HIP),
    (LM_L_HIP, LM_R_HIP),
    (LM_L_HIP, LM_L_KNEE), (LM_L_KNEE, LM_L_ANKLE),
    (LM_R_HIP, LM_R_KNEE), (LM_R_KNEE, LM_R_ANKLE),
]


# ---------- Yanshee REST helpers ----------------------------------------------

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


def _get_bytes(url: str, timeout: float = 5.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.read()


def send_angles(ip: str, angles: Dict[str, int], runtime_ms: int) -> None:
    try:
        _put(ip, "/servos/angles", {"angles": angles, "runtime": runtime_ms})
    except Exception as e:
        print(f"[send err] {e}", file=sys.stderr)


def play_preset(ip: str, name: str, speed: str = "normal") -> None:
    try:
        _put(ip, "/motions",
             {"operation": "start",
              "motion": {"name": name, "repeat": 1, "speed": speed},
              "timestamp": 0, "version": "v1"})
    except Exception as e:
        print(f"[preset err] {e}", file=sys.stderr)


def open_stream(ip: str, resolution: str = "640x480") -> None:
    _delete(ip, "/visions/streams")
    time.sleep(0.4)
    res = _post(ip, "/visions/streams", {"resolution": resolution})
    if res.get("code") != 0:
        raise RuntimeError(f"open_vision_stream failed: {res}")


def take_photo(ip: str, resolution: str = "320x240") -> bytes:
    res = _post(ip, "/visions/photos", {"resolution": resolution})
    if res.get("code") != 0:
        raise RuntimeError(f"take_vision_photo failed: {res}")
    name = res["data"]["name"]
    # YanAPI uses ?body=<name> query string, not a path component.
    import urllib.parse as up
    return _get_bytes(
        f"http://{ip}:9090/v1/visions/photos?body={up.quote(name)}",
        timeout=8,
    )


def battery_pct(ip: str) -> Optional[int]:
    try:
        with urllib.request.urlopen(f"http://{ip}:9090/v1/devices/battery", timeout=1.5) as r:
            return int(json.loads(r.read())["data"]["percent"])
    except Exception:
        return None


# ---------- pose math ---------------------------------------------------------

def clamp(v: float, lo: float, hi: float) -> int:
    return int(max(lo, min(hi, v)))


def angle_at(p_a, p_b, p_c) -> float:
    """Joint angle at p_b in degrees, range [0,180]."""
    ax, ay = p_a.x - p_b.x, p_a.y - p_b.y
    cx, cy = p_c.x - p_b.x, p_c.y - p_b.y
    dot = ax * cx + ay * cy
    mag = math.hypot(ax, ay) * math.hypot(cx, cy) + 1e-9
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / mag))))


def shoulder_decompose(shoulder, elbow, side: str) -> Tuple[float, float]:
    """Return (abduction_deg, flexion_deg) using MediaPipe's 3D landmarks.

    Image coords: x rightward, y downward, z toward camera (negative = away).
    Vector v = elbow - shoulder. Decompose into the body planes:

      abduction (sideways from torso, drives ShoulderRoll):
        angle in the coronal (x,y) plane between -y axis (up) and v_xy.
        Range: 0° = arm hanging down, 90° = T-pose, 180° = overhead.
        Sign convention: outward-from-body is positive.

      flexion (forward/back, drives ShoulderFlex):
        angle in the sagittal (y,z) plane between -y axis (up) and v_yz.
        Range: 0° = arm down, 90° = arm pointing forward, 180° = behind.
        z<0 (toward camera) means forward.
    """
    dx = elbow.x - shoulder.x
    dy = elbow.y - shoulder.y
    dz = getattr(elbow, "z", 0.0) - getattr(shoulder, "z", 0.0)

    # Outward-from-body sign: for the person's RIGHT arm, outward = +x;
    # for LEFT arm, outward = -x. (MediaPipe labels by anatomy.)
    out_sign = 1.0 if side == "R" else -1.0
    side_x = out_sign * dx

    # Abduction: angle of (side_x, dy) from straight-down (0, +1).
    abduction = math.degrees(math.atan2(max(side_x, 0.0), dy))

    # Flexion: forward (-dz) vs down (dy). Negative z = elbow is closer to camera.
    flexion = math.degrees(math.atan2(-dz, dy))

    # Clamp to plausible ranges.
    abduction = max(0.0, min(180.0, abduction))
    flexion = max(-90.0, min(180.0, flexion))
    return abduction, flexion


class EMA:
    def __init__(self, alpha: float = 0.35) -> None:
        self.alpha = alpha
        self.values: Dict[str, float] = {}

    def step(self, key: str, val: float) -> float:
        prev = self.values.get(key, val)
        new = prev * (1 - self.alpha) + val * self.alpha
        self.values[key] = new
        return new


def map_pose_to_servos(landmarks, smoother: EMA, scale: float = 1.0) -> Dict[str, int]:
    """Mirrored mapping using 3D landmarks — drives all 4 arm joints.

    Person's RIGHT limb -> robot's LEFT limb (mirror image).

    Servo conventions (assumed; flip in MIRROR_FLIPS if a joint moves the wrong way):
      *ShoulderRoll: 90=arm at side, 180=arm raised outward to overhead.
      *ShoulderFlex: 90=arm at side, 180=arm forward at shoulder height.
      *ElbowFlex:    180=straight, 90=bent 90°, 0=hyperflexed.
    """
    R_sh, R_el, R_wr = landmarks[LM_R_SHOULDER], landmarks[LM_R_ELBOW], landmarks[LM_R_WRIST]
    L_sh, L_el, L_wr = landmarks[LM_L_SHOULDER], landmarks[LM_L_ELBOW], landmarks[LM_L_WRIST]

    R_abd, R_flex = shoulder_decompose(R_sh, R_el, "R")
    L_abd, L_flex = shoulder_decompose(L_sh, L_el, "L")
    R_elbow = angle_at(R_sh, R_el, R_wr)           # 90 (bent) .. 180 (straight)
    L_elbow = angle_at(L_sh, L_el, L_wr)

    # Mirror: person's right arm drives robot's LEFT, person's left -> robot's RIGHT.
    raw = {
        "LeftShoulderRoll":  90 + R_abd  * scale,
        "LeftShoulderFlex":  90 + R_flex * scale,
        "LeftElbowFlex":     R_elbow,
        "RightShoulderRoll": 90 + L_abd  * scale,
        "RightShoulderFlex": 90 + L_flex * scale,
        "RightElbowFlex":    L_elbow,
    }

    out: Dict[str, int] = {}
    for name, val in raw.items():
        smoothed = smoother.step(name, val)
        # Apply optional per-servo direction flip: pivot around neutral 90.
        flipped = 90 + SERVO_DIR.get(name, 1) * (smoothed - 90)
        lo, hi = SERVO_RANGES[name]
        out[name] = clamp(flipped, lo, hi)
    return out


# ---------- drawing -----------------------------------------------------------

def _draw_skeleton(frame, landmarks) -> None:
    import cv2
    h, w = frame.shape[:2]
    for lm in landmarks:
        x, y = int(lm.x * w), int(lm.y * h)
        cv2.circle(frame, (x, y), 4, (0, 200, 255), -1)
    for a, b in POSE_CONNECTIONS:
        ax, ay = int(landmarks[a].x * w), int(landmarks[a].y * h)
        bx, by = int(landmarks[b].x * w), int(landmarks[b].y * h)
        cv2.line(frame, (ax, ay), (bx, by), (255, 200, 0), 2)


def _draw_dashboard(frame, target, fps, battery, paused, source_label, sending_hz):
    import cv2
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 36), (20, 20, 20), -1)
    cv2.rectangle(overlay, (0, h - 32), (w, h), (20, 20, 20), -1)
    frame[:] = cv2.addWeighted(overlay, 0.55, frame, 0.45, 0)

    bat_str = f"BAT {battery}%" if battery is not None else "BAT --"
    bat_color = (0, 220, 0) if (battery or 0) > 25 else (0, 100, 255)
    cv2.putText(frame, "YANSHEE MIRROR", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, bat_str, (w - 110, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, bat_color, 2)
    cv2.putText(frame, f"{fps:5.1f} fps", (w - 220, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

    if target:
        bars = [
            ("L Shoulder", target.get("LeftShoulderRoll"),  SERVO_RANGES["LeftShoulderRoll"]),
            ("L Elbow",    target.get("LeftElbowFlex"),     SERVO_RANGES["LeftElbowFlex"]),
            ("R Shoulder", target.get("RightShoulderRoll"), SERVO_RANGES["RightShoulderRoll"]),
            ("R Elbow",    target.get("RightElbowFlex"),    SERVO_RANGES["RightElbowFlex"]),
        ]
        bar_x, bar_y, bar_w = 10, 50, 220
        for label, val, (lo, hi) in bars:
            if val is None:
                continue
            pct = (val - lo) / max(1, hi - lo)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 14), (60, 60, 60), -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(bar_w * pct), bar_y + 14),
                          (0, 200, 255), -1)
            cv2.putText(frame, f"{label}: {val:>3}", (bar_x + bar_w + 8, bar_y + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            bar_y += 22

    state = "PAUSED" if paused else f"SENDING {sending_hz:.1f} Hz"
    state_color = (0, 0, 255) if paused else (0, 255, 0)
    cv2.putText(frame, state, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, state_color, 2)
    cv2.putText(frame, "q=quit  p=pause  r=Reset", (w - 280, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


# ---------- main loop ---------------------------------------------------------

def _decode_jpeg(buf: bytes):
    import cv2, numpy as np
    arr = np.frombuffer(buf, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


class LatestFrame:
    """Background reader that keeps only the most recent frame from a cv2 stream.

    OpenCV's HTTP/MJPEG backend buffers frames internally. If we don't drain
    fast enough, cap.read() returns stale frames. This thread runs read() in a
    tight loop and overwrites a single slot — readers always get the freshest.
    """

    def __init__(self, cap) -> None:
        self.cap = cap
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> "LatestFrame":
        self._thread.start()
        return self

    def _loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame = frame

    def get(self):
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.5)


def _make_pose_detector(model_path: str, photo_mode: bool):
    """photo_mode=True uses IMAGE running mode (independent frames),
    False uses VIDEO mode (temporal tracking across frames)."""
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    rm = vision.RunningMode.IMAGE if photo_mode else vision.RunningMode.VIDEO
    options = vision.PoseLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=model_path),
        running_mode=rm,
        num_poses=1,
        min_pose_detection_confidence=0.55,
        min_pose_presence_confidence=0.55,
        min_tracking_confidence=0.55,
    )
    return vision.PoseLandmarker.create_from_options(options)


def _loop(cap, ip: str, args, model_path: str, photo_mode: bool, source_label: str) -> int:
    import cv2
    import mediapipe as mp

    # Startup: arms-only neutral, legs UNTOUCHED.
    print(f"sending arm-neutral pose to {ip} (legs left as-is)...")
    send_angles(ip, ARM_NEUTRAL, runtime_ms=1500)
    time.sleep(1.6)

    detector = _make_pose_detector(model_path, photo_mode=photo_mode)
    smoother = EMA(alpha=args.smoothing)
    period = 1.0 / args.rate
    last_send = 0.0
    last_photo = 0.0
    paused = False

    fps_t0 = time.time()
    fps_count = 0
    fps = 0.0
    battery = battery_pct(ip)
    last_battery_check = time.time()
    t0_ms = int(time.time() * 1000)

    # Stream mode: spawn a background reader so we always work on the freshest frame.
    grabber: Optional[LatestFrame] = None
    if not photo_mode and cap is not None:
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        grabber = LatestFrame(cap).start()
        # Wait briefly for the first frame.
        for _ in range(40):
            if grabber.get() is not None:
                break
            time.sleep(0.05)

    print("running. q=quit, p/space=pause, r=Reset preset")
    frame = None
    frame_is_new = False  # photo-mode: only run detection on freshly fetched frames
    try:
        while True:
            now = time.time()
            if photo_mode:
                if now - last_photo >= args.interval:
                    try:
                        frame = _decode_jpeg(take_photo(ip))
                        last_photo = now
                        frame_is_new = True
                    except Exception as e:
                        print(f"[photo err] {e}", file=sys.stderr)
                        time.sleep(0.5)
                        continue
                else:
                    time.sleep(0.05)
                    if frame is None:
                        continue
            else:
                # Always pull the latest frame from the background grabber.
                latest = grabber.get() if grabber else None
                if latest is None:
                    time.sleep(0.02)
                    continue
                frame = latest.copy()

            target = None
            should_detect = (not photo_mode) or frame_is_new
            if should_detect:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                if photo_mode:
                    result = detector.detect(mp_image)
                else:
                    ts_ms = int(time.time() * 1000) - t0_ms
                    result = detector.detect_for_video(mp_image, ts_ms)
                if result.pose_landmarks:
                    lm = result.pose_landmarks[0]
                    target = map_pose_to_servos(lm, smoother, scale=args.scale)
                    if not args.no_display:
                        _draw_skeleton(frame, lm)

            send_now = (
                target is not None and not paused
                and (frame_is_new if photo_mode else (now - last_send) >= period)
            )
            if send_now:
                # Yanshee servo runtime is capped at 4000ms.
                runtime = (
                    min(4000, int(args.interval * 1000 * 0.85)) if photo_mode
                    else int(period * 1000 * 1.4)
                )
                send_angles(ip, target, runtime_ms=runtime)
                last_send = now
                if args.debug_angles:
                    parts = "  ".join(f"{k}={v}" for k, v in target.items())
                    print(f"[{time.strftime('%H:%M:%S')}] {parts}", file=sys.stderr)
                if photo_mode:
                    frame_is_new = False  # consume the new frame

            fps_count += 1
            if (now - fps_t0) >= 1.0:
                fps = fps_count / (now - fps_t0)
                fps_count = 0
                fps_t0 = now
            if (now - last_battery_check) > 10:
                battery = battery_pct(ip)
                last_battery_check = now

            if not args.no_display:
                if frame.shape[1] < 800:
                    frame = cv2.resize(frame, (frame.shape[1] * 2, frame.shape[0] * 2))
                _draw_dashboard(frame, target, fps, battery, paused, source_label, args.rate)
                cv2.imshow("Yanshee Mirror", frame)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    break
                if k in (ord('p'), ord(' ')):
                    paused = not paused
                    print("paused" if paused else "resumed")
                if k == ord('r'):
                    play_preset(ip, "Reset")
    except KeyboardInterrupt:
        pass
    finally:
        print("\ncleaning up: arms back to neutral (legs untouched)...")
        if grabber is not None:
            grabber.stop()
        if cap is not None:
            cap.release()
        if not args.no_display:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass
        send_angles(ip, ARM_NEUTRAL, runtime_ms=1200)
        if not photo_mode:
            try:
                _delete(ip, "/visions/streams")
            except Exception:
                pass
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=4.0,
                        help="servo updates per second (default 4)")
    parser.add_argument("--smoothing", type=float, default=0.6,
                        help="EMA alpha 0..1, higher = snappier (default 0.6)")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="motion scale factor (default 1.0)")
    parser.add_argument("--debug-angles", action="store_true",
                        help="print raw and final per-servo angles to stderr")
    parser.add_argument("--photo-mode", action="store_true",
                        help="snapshot every --interval seconds instead of streaming")
    parser.add_argument("--interval", type=float, default=5.0,
                        help="seconds between photo snapshots in --photo-mode (default 5)")
    parser.add_argument("--no-display", action="store_true",
                        help="skip cv2 window (headless)")
    parser.add_argument("--model", default="models/pose_landmarker_lite.task",
                        help="path to MediaPipe pose .task model")
    args = parser.parse_args()

    ip = os.environ.get("ROBOT_IP")
    if not ip:
        print("ERROR: set ROBOT_IP env var", file=sys.stderr)
        return 1

    if not os.path.exists(args.model):
        print(f"ERROR: model not found at {args.model}", file=sys.stderr)
        return 1

    if args.photo_mode:
        return _loop(None, ip, args, args.model, photo_mode=True,
                     source_label="photo poll")

    print("opening MJPEG stream on robot...")
    open_stream(ip, resolution="640x480")
    stream_url = f"http://{ip}:8000/stream.mjpg"
    print(f"stream URL: {stream_url}")
    import cv2
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"ERROR: cv2 could not open {stream_url}", file=sys.stderr)
        return 1
    return _loop(cap, ip, args, args.model, photo_mode=False, source_label=stream_url)


if __name__ == "__main__":
    sys.exit(main())
