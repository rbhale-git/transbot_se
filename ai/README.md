# AI behaviors

Laptop-side autonomous behaviors for the Transbot SE. Each behavior is one
process; all of them talk to the unmodified robot through the same two
interfaces the dashboard uses (rosbridge WebSocket + MJPEG stream). Full
design notes and test logs: [docs/AI_NOTEBOOK.md](../docs/AI_NOTEBOOK.md).

```
ai/
  config.py         addresses, servo facts, tracker gains (mirrors dashboard config.js)
  common/           shared infrastructure: video reader, rosbridge client, safety
  face_tracking/    stage 1: face detection + gimbal tracking
  models/           YuNet face detector ONNX (ships in-repo, ~230 KB)
  tests/            pytest suite — run `python -m pytest ai/tests` from repo root
```

## Setup

```
pip install -r ai/requirements.txt
```

## Face tracking (stage 1)

Safe-by-construction: publishes only `/PWMServo` (gimbal). Never touches
`/cmd_vel`.

```
# offline, laptop webcam, commands printed not sent
python -m ai.face_tracking --source 0 --dry-run

# offline, recorded clip
python -m ai.face_tracking --source clip.avi --dry-run

# robot camera but no commands (verify detection on the real stream)
python -m ai.face_tracking --dry-run

# live (robot on home Wi-Fi; --profile hotspot for the robot AP)
python -m ai.face_tracking
```

Preview keys: `q`/ESC quit, `c` re-center gimbal. Useful flags: `--kp`,
`--deadband`, `--max-step`, `--rate`, `--pan-sign/--tilt-sign` (flip if the
gimbal runs away from the face), `--record out.avi` (capture the stream for
offline tuning), `--no-preview`.
