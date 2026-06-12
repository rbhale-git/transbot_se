# AI behaviors

Laptop-side autonomous behaviors for the Transbot SE. Each behavior is one
process; all of them talk to the unmodified robot through the same two
interfaces the dashboard uses (rosbridge WebSocket + MJPEG stream). Full
design notes and test logs: [docs/AI_NOTEBOOK.md](../docs/AI_NOTEBOOK.md).

```
ai/
  config.py           addresses, servo facts, behavior gains (mirrors dashboard config.js)
  common/             shared infrastructure: video reader, rosbridge client, safety,
                      detection type, rosbridge actuation check
  face_tracking/      stage 1: face detection + gimbal tracking (YuNet)
  person_following/   stage 2: person/dog following on the chassis (YOLO11n)
  models/             ONNX models, in-repo: YuNet (~230 KB), YOLO11n (~10 MB)
  tests/              pytest suite — run `python -m pytest` from repo root
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

## Person following (stage 2)

Drives the chassis: publishes `/ai/cmd_vel`, which the robot-side
`cmd_vel_mux` forwards to `/cmd_vel` only while the dashboard's AI switch is
ARMED and the joystick is quiet — clamped to the AI caps (0.25 m/s fwd,
0.12 rev, 1.2 rad/s) no matter what this process sends. Locks the largest
person (or `--target-class dog`) at arm time, follows by IoU association
only, stops on loss. Engineering notes:
[docs/PERSON_FOLLOWING_NOTEBOOK.md](../docs/PERSON_FOLLOWING_NOTEBOOK.md).

The gimbal tracks the locked person too (armed or not — arming gates chassis
motion only): it aims at the face/chest (20% from the bbox top) with the
stage-1 move-and-settle tracker, and the chassis steers on the total bearing
(image error + pan offset), so lock survives lateral motion that the
chassis alone is too slow to catch. `--fixed-gimbal` parks the gimbal for
A/B comparison with the stage-2 behavior. Design:
`docs/superpowers/specs/2026-06-11-gimbal-assisted-following-design.md`.

```
# offline, recorded clip
python -m ai.person_following --source clip.avi --dry-run

# robot camera, no commands
python -m ai.person_following --dry-run

# live: start it, then ARM AI on the dashboard. SPACE = stop + disarm.
python -m ai.person_following

# first runs in a new space: half caps
python -m ai.person_following --cap-scale 0.5
```

Preview keys: `q`/ESC quit, `t` pause (publishes zeros, freezes the gimbal).
Useful flags: `--target-class`, `--cap-scale`, `--kp-ang`, `--kp-lin`,
`--height-setpoint` (bigger = follows closer), `--smoothing`,
`--fixed-gimbal`, `--record out.avi`. AI behaviors
request a 480p server-side-downscaled stream by default (the Nano can't
afford 720p per client — see the notebook's latency section); pass
`--source` with the bare stream URL if you ever need full resolution.

Detector throughput check: `python -m ai.person_following.benchmark`
(design gate: 10+ fps; ~12 fps measured on the dev laptop at 640 px).
