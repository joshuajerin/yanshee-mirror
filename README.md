# Yanshee Mirror

Real-time pose mirroring for the UBTech Yanshee mini humanoid robot. The robot watches a person with its onboard camera and physically mirrors their arm movements — your right arm raises, the robot's left arm raises.

The pipeline: Yanshee's MJPEG camera stream → MediaPipe Pose (33-point body landmarks, 3D) → 3D shoulder/elbow joint decomposition → mirrored servo commands → REST API to the robot's `motions` daemon.

Bonus: a multi-phase handstand attempt sequence (servo angles only, no preset exists), TTS helpers, and a complete OM1 agent config for layering an LLM on top.

Built at the OpenMind hackathon, 2026-05-06.

## What's in here

- **`scripts/mirror.py`** — the real-time mirror. cv2 dashboard with live skeleton overlay, servo bars, FPS, and battery readout. Drives 6 arm joints + neck via MediaPipe's 3D landmarks; legs are never touched.
- **`scripts/handstand.py`** — 7-phase servo sequence that attempts a handstand. Has `--dry-run`, `--phase N`, and `--auto`. Realistically: the arm servos aren't built to support the body inverted, so this is for demonstration / iteration.
- **`scripts/say.sh` / `move.sh` / `stop.sh`** — one-liner shell helpers for TTS, preset motions, and emergency stop.
- **`config/ubtech_yanshee.json5`** — OM1 agent config to drive the robot via an OpenAI cortex with voice input.

## Layout
```
5-6-26-hack-omi/
├── config/
│   └── ubtech_yanshee.json5   # OM1 agent config (drop into OM1's config/ dir)
├── scripts/
│   └── handstand.py           # standalone phase-by-phase servo sequence
├── .env / .env.example        # OM_API_KEY, OPENAI_API_KEY, ROBOT_IP
└── requirements.txt
```

## Setup

### 1. Pair the robot and find its IP
- Install the Yanshee mobile app, pair with the robot over Bluetooth.
- App → Setup → Robot information → copy the IP.
- Verify reachability: `ping <ROBOT_IP>` from your laptop (must be on the same Wi-Fi).
- Put the IP in `.env` as `ROBOT_IP=...`.

### 2. Clone OM1 + the UBTech connector
```bash
# alongside this folder
git clone https://github.com/OpenMind/OM1.git
git clone https://github.com/OpenMind/ubtech.git
pip install -e ./ubtech         # installs YanAPI as `ubtechapi`
```

### 3. Install OM1 itself
Follow OM1's own README (uses `uv`). Then drop our config in:
```bash
cp config/ubtech_yanshee.json5 ../OM1/config/
cd ../OM1
uv run src/run.py ubtech_yanshee
```
Open WebSim at <http://localhost:8000> for live debug.

## Handstand — what's actually happening

Yanshee has **no built-in handstand preset**. Built-ins are: `reset, raise, crouch, stretch, come on, wave, bend, walk, turn around, head, bow`.

We sequence raw servo angles via `YanAPI.set_servos_angles({servo: angle}, runtime_ms)`. The 7-phase plan in `scripts/handstand.py`:

| Phase | What it does |
|---|---|
| 0 | Reset to neutral |
| 1 | Deep crouch (lower hips, bend knees) |
| 2 | Plant arms forward, elbows locked |
| 3 | Hip lift — straighten legs, weight onto arms |
| 4 | Kick legs vertical |
| 5 | Hold (mostly a pause for manual support) |
| 6 | Recover to crouch |
| 7 | Stand |

**Reality check:** Yanshee's arm servos (~5-6 kg-cm) are not designed to support its ~1.5kg body inverted. Expect tipping forward in phase 4. **Run the first attempts on a soft surface and physically catch the robot.** The script defaults to interactive mode — press ENTER between phases so you can abort.

### Test the sequence

```bash
# print without sending anything
python scripts/handstand.py --dry-run

# interactive — pauses between phases
ROBOT_IP=$(grep ROBOT_IP .env | cut -d= -f2) python scripts/handstand.py

# isolate one phase while tuning
ROBOT_IP=... python scripts/handstand.py --phase 2

# full sequence, no prompts (only after you've tuned each phase)
ROBOT_IP=... python scripts/handstand.py --auto
```

### Tuning notes
- Servo angle conventions in `handstand.py` are **first-pass guesses** — use `--phase` mode to verify each step on the real robot, log the angles that work, then update `PHASES`.
- Once tuned, the sequence can be uploaded as a custom HTS motion file via the `motions/upload` REST endpoint, after which `move: "handstand"` in the OM1 config will trigger it natively. (Not implemented yet — current path is the standalone script.)

## Mirror mode (live)

Robot watches you with its onboard camera, and mirrors your arm movements in real time. Live MJPEG → MediaPipe Pose → mirrored servo commands.

```bash
# one-time setup
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# run (laptop must reach robot's :9090 and :8000)
ROBOT_IP=10.73.35.187 .venv/bin/python scripts/mirror.py
```

A cv2 window opens showing the robot-camera feed with pose skeleton overlay, live servo angles (with bars), FPS, and battery %. Keys:

- `q` — quit (resets robot before exit)
- `p` / `space` — pause/resume sending commands
- `r` — fire `Reset` preset

Flags:
- `--rate 5` — servo updates per second (5 is a good default)
- `--smoothing 0.35` — EMA alpha 0..1, lower = smoother (less jittery)
- `--scale 0.85` — motion scale, lower = more conservative
- `--no-display` — headless
- `--photo-mode --interval 5` — fallback: snap a photo every 5s instead of streaming

Notes:
- Only **arms + neck** are driven. Legs are locked in standing pose (moving them while standing risks tipping).
- Mirror semantics: your **right** arm raises → robot's **left** arm raises (literal mirror image).
- Servo angle conventions are empirical — tweak `--scale` if motion under/overshoots.

## Quick reference: shell helpers

```bash
ROBOT_IP=10.73.35.187 ./scripts/say.sh "Hello"
ROBOT_IP=10.73.35.187 ./scripts/move.sh Reset
ROBOT_IP=10.73.35.187 ./scripts/move.sh RaiseRightHand normal 1
ROBOT_IP=10.73.35.187 ./scripts/move.sh PushUp slow 1
ROBOT_IP=10.73.35.187 ./scripts/stop.sh           # halt motion
```

## References
- Workshop doc: https://docs.google.com/document/d/1NDUcJYipm_j0F1CvNxx09VdDpm54Bb2lXrYa7IukQGc
- OM1 runtime: https://github.com/OpenMind/OM1
- UBTech connector + YanAPI: https://github.com/OpenMind/ubtech
- Yanshee setup guide: https://docs.openmind.com/developer-cookbook/om1-integration-with-different-machines/ubtech_yanshee
