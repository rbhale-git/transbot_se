# Transbot SE — AI Behaviors Notebook

Project: laptop-hosted autonomous behaviors for the Yahboom Transbot SE
Repository: https://github.com/rbhale-git/transbot_se
Started: June 2026 (stage 1 implemented 2026-06-10)
Companion to: BRINGUP_NOTEBOOK.md (dashboard bring-up, complete)

## 1. Approach

AI behaviors are developed and run on the laptop in modern Python 3, treating
the robot as a fixed appliance: frames in over the MJPEG stream, commands out
over rosbridge. No robot-side changes are needed — the same interfaces the
dashboard drives. Models get ported to the Jetson later only if a behavior
needs to run untethered.

Staged roadmap (agreed before starting):

1. **Face detection + gimbal tracking** — no chassis motion. COMPLETE
   (2026-06-10, operator-accepted after live calibration). THIS DOCUMENT.
2. Person following (chassis motion; requires the command-priority mux and
   dashboard AI arm/disarm panel first).
3. ArUco-marker waypoint navigation; evaluate lidar purchase for real SLAM.
4. Autonomous pick and place (visual servo + calibrated grasp sequences).

## 2. Stage 1 — Face tracking on the gimbal

Full engineering notebook (design iterations, calibration data, the rosbridge
incident, test record): GIMBAL_TRACKING_NOTEBOOK.md (+ generated .docx). The
sections below are the running lab log kept during the work.

### Design

| Piece | Choice | Why |
| --- | --- | --- |
| Detector | OpenCV YuNet (`cv2.FaceDetectorYN`, ONNX in `ai/models/`) | Ships with opencv-python, fast on CPU, no extra runtime; MediaPipe was the alternative but adds a heavy dependency for the same job |
| Target selection | Largest bounding box | Closest face wins; deterministic |
| Control | P-control per axis on normalized image-center error | Simplest thing that works; gains are CLI-tunable for live tuning |
| Transport | `/PWMServo` via roslibpy (`ws://robot:9090`) | Same verified interface as the dashboard gimbal panel |
| Frames | `VideoSource` — background thread keeps only the newest frame | Letting the MJPEG stream queue adds seconds of control latency |

Control step, per axis, at a fixed rate (default 10 Hz):

```
error      = (face_center - frame_center) / (frame_size / 2)   # -1..+1
if |error| < deadband: no command                              # 0.06 default
delta      = sign * kp * error, capped at ±max_step            # 8°/unit, 4° cap
angle      = clamp(angle + delta, axis min..max)               # 0..180
command only when the rounded angle actually changes
```

Worst-case slew is therefore `max_step × rate` = 40°/s with defaults — gentle
by construction. Angle state is float so sub-degree corrections accumulate
instead of being lost to rounding.

### Safety envelope

- Publishes **only** `/PWMServo` — physically incapable of driving the chassis.
  (Stage 2's chassis motion is gated on the agreed mux + arm/disarm panel.)
- Lost target → hold position, zero commands (a stationary gimbal is safe;
  re-centering on loss would fight the operator).
- Deadband, per-step cap, and range clamp as above; settled tracker is silent
  on the wire.
- Startup drives the gimbal to home (pan 90 / tilt 22) because the driver
  never echoes servo positions — that sync is the only way tracker state can
  match reality.

### Direction signs (TO-VERIFY live)

From the dashboard bring-up: positive pan angle moves the view LEFT (the
`invertStep: true` finding), and positive tilt moves the view UP. Both axes
therefore use `sign = -1` (servo step opposes image error). **First live test
must confirm this** — if an axis runs away from the face instead of toward
it, flip it with `--pan-sign` / `--tilt-sign` and bake the result into
`ai/config.py`.

### Testing (done offline, 2026-06-10)

- 37-test pytest suite (`python -m pytest ai/tests`): controller behaviors
  (response direction, deadband, step cap, clamping, accumulation, lost
  target, recenter), face selection, wire shape against the verified
  `transbot_msgs/PWMServo` layout, video reader against real generated files,
  YuNet against a real photographic face.
- End-to-end integration test: synthetic 720p clip with an off-center face →
  full pipeline emits monotonic, correctly-signed, clamped command walks.
- CLI dry-run against a generated clip reproduced the same on the real
  runner path.

### Live finding (2026-06-10): rosbridge can silently drop a whole session

First live commands did nothing despite a healthy connection. Robot-side
`rostopic pub` moved the gimbal, and a later identical laptop session also
worked — the journal showed why: on the first session, rosbridge (0.11,
Melodic, py2) hit `Internal error processing topic [/PWMServo]` while
registering its publisher, then refused the driver's connections with
`[/rosbridge_websocket] is not a publisher of [/PWMServo]` for the session's
entire lifetime. Master believed the publisher existed; the driver could
never connect to it; every publish was dropped without any client-visible
error. Disconnecting cleared it (unadvertise), and fresh sessions were fine.

Countermeasures:

- `RosClient.connect()` now advertises explicitly and waits 1 s before any
  traffic, rather than roslibpy's advertise-lazily-then-publish-immediately.
- The startup homing doubles as an actuation check: the gimbal must visibly
  snap to home when the tracker starts. **If it doesn't move, quit and
  restart the tracker** — a fresh websocket session re-registers cleanly.
- Signature on the robot if it recurs:
  `sudo journalctl -u rosbridge-dashboard.service | grep "is not a publisher"`.

### Live finding (2026-06-10): latency overshoot, and the fix

First live tracking run confirmed both direction signs (sign = -1 on both
axes) but exposed a failure mode recorded video can't show: with pure P at
kp 8 / step 4, camera-stream latency kept the measured error large after the
gimbal had already reached the face — it overshot, the face left the frame,
and hold-on-lost left the camera staring at the ceiling.

Fixes, verified live on the second run:

- **D term** (`kd_deg`, default 4): damps the step while the error is
  already shrinking — the latency brake.
- **Gentler defaults**: kp 5, step cap 2.5° (25°/s max slew).
- **Recenter on prolonged loss** (`lost_recenter_after`, ~5 s): recovery
  instead of staring at where the face last wasn't.

Second run tracked continuously with no loss. Residual vertical error
(~0.2-0.3) when the subject stands close over the floor-level robot is
geometric, not a tuning problem — the face is huge in frame and the gimbal
is at the sensible end of its useful tilt.

Operational note: the Jetson Nano (~80% CPU serving one 720p MJPEG client)
cannot feed extra diagnostic stream taps while a behavior is running — the
behavior's reader starts timing out. One stream client at a time.

### Live finding (2026-06-10): measure, don't guess — the calibration session

Continuous-control retunes oscillated and move-and-settle still double-fired;
the user's ranking of iterations made it clear guessing constants wasn't
converging. A calibration script (command a known 8° move, watch when and how
far the face shifts) measured the truth:

| Quantity | Guessed | Measured |
| --- | --- | --- |
| Loop latency (command -> visible in stream) | 0.4 s | **0.79-0.83 s** |
| Pan full-correction gain | 20°/err | **48.5°/err** |
| Tilt full-correction gain | 20°/err | **23.2°/err** |

Consequences now in config: per-axis `kp_deg` on AxisConfig (41 pan / 20
tilt, ~0.85x measured so each move slightly undershoots), settle window 11
updates (~1.1 s ≥ measured latency), per-move clamp 20°. The residual
overshoot had been the settle window (0.4 s) being half the real blind time —
the tracker re-measured a stale frame and fired the same correction twice.

### Live finding (2026-06-10, escalated): the rosbridge dropped-session bug

Worse than first thought. Observed across two boots:

- It hits roughly every other fresh session in the minutes after boot, and
  recurred ~25 min in — not a one-off.
- Once a session corrupts the registration, the broken `/PWMServo` publisher
  can persist inside rosbridge EVEN AFTER ALL CLIENTS DISCONNECT
  (client_count 0, five consecutive fresh sessions all dead). Only
  `sudo systemctl restart rosbridge-dashboard.service` clears that state.
- The robot's clock boots in 2021 until NTP syncs (no RTC), so
  `journalctl --since` can hide the evidence — use `journalctl -b`.

Countermeasures in code:

- `connect_with_actuation_check()` in the runner: every connect wiggles the
  tilt servo and verifies the camera view changed (`frames_differ`, threshold
  calibrated live: noise 4-8, motion ~40) before tracking starts; on failure
  it reconnects with a fresh session, up to 3 attempts, then aborts with the
  journal pointer.
- roslibpy gotcha baked into RosClient: `terminate()` kills the process-wide
  Twisted reactor and it can NEVER restart — reconnect loops must use
  `disconnect()` (websocket close only); `close()` only at process exit.

### Live-test checklist (robot day)

1. Robot on home Wi-Fi, NordVPN fully exited.
2. `python -m ai.face_tracking --dry-run` — confirm detection boxes on the
   real camera, sane printed commands, ~no lag in the preview.
3. `python -m ai.face_tracking` — stand center, then step sideways slowly.
   Wrong-direction axis → quit (`q`), rerun with `--pan-sign 1` and/or
   `--tilt-sign 1`, then fix config.py.
4. Tune: oscillates → lower `--kp` (or raise `--deadband`); sluggish → raise
   `--kp`; jerky steps → lower `--max-step`.
5. `--record tuning.avi` during a live session to build a real test clip for
   regression runs.

## 3. Stage 2+ notes

Person following adds `/cmd_vel` and is blocked on the two agreed
prerequisites: a command-priority mux node (e-stop > manual > AI, with a
lower AI speed cap) and a dashboard AI arm/disarm panel. The robot-side
cmd_vel watchdog (bring-up phase 5) already covers AI-crash and link-loss
cases.

## Auto-heal for the rosbridge registration bug (2026-06-12)

The per-topic silent-drop bug now cures itself. Three layers, one shared
cure (`ai/common/heal.py`: SSH `sudo -n systemctl restart
rosbridge-dashboard.service`, then wait for the rosbridge port; passwordless
key + sudo already provisioned):

- Runner preflight (`ai/common/connect.py`): every fresh session is checked
  in two phases — rosapi publisher registration for /PWMServo, /ai/cmd_vel
  and /ai/status (the check that would have caught both 2026-06-11
  incidents), then the tilt-wiggle end-to-end proof. Escalation: 2 extra
  fresh sessions, then ONE heal per run, then one final session, then
  abort naming the dead topics. rosapi silent (mock server) = unverified,
  the wiggle decides alone. The heal restarts the whole robot stack, so
  the video source is reopened afterwards (`VideoSource.reopen()` retries
  up to 60 s while the camera comes back).
- Dashboard: CMD FAULT now carries a RESTART ROSBRIDGE button (only when
  served by `tools/serve_dashboard.py` — a browser cannot SSH). POST
  /api/heal refuses (409) while a behavior process is running, and a
  server-side lock stops concurrent heals from stacking restarts.
- By hand: `python tools/heal_rosbridge.py [--profile hotspot]`. Also
  clears the wedged /voltage publisher seen after power cycles.

Blast radius of any heal: camera + all connections drop ~15-30 s (the
service runs bringup + usb_cam + web_video_server + rosbridge). All paths
fire only with the robot stationary: the preflight runs before arming is
possible, the button is operator-clicked and refused while a runner is up.

Known limitation: a robot that is unreachable at process start still fails
fast — the heal fires only when a session CONNECTS but its checks fail.

Also new: the AI panel has a TARGET picker (person/dog/cat, `config.js`
`AI.targetClasses`), passed as `--target-class` at the next START. Dog-mode
caveat: `height_setpoint` 0.8 was tuned on a standing person, so the follow
distance reads closer on a dog — live dog tuning is its own session. The
runner matches the class label literally; a typo'd config value would start
cleanly and simply never lock (the picker constrains values, so only a bad
`config.js` edit can hit this).

### Live validation (2026-06-12, robot ~4 min after boot)

All automatable checklist items PASSED against the live robot:

- CLI heal with the stack healthy: `python tools/heal_rosbridge.py` →
  "restarted; rosbridge port is back"; camera + rosbridge ports back ~4 s
  after the port probe resumed; service active.
- Runner preflight (person): registration check silent-pass, wiggle passed,
  "actuation check passed (attempt 1)", runner stable.
- THE LADDER FIRED FOR REAL on the dog-mode start: attempt 1's wiggle
  failed ("commands are not reaching the gimbal" — the known dropped-session
  behavior; registration had passed), the ladder forced a fresh session and
  passed on attempt 2. Zero operator intervention — exactly the failure the
  feature was built for, cured by the first escalation rung.
- Dog mode: `/ai/status` verified ARRIVING ROBOT-SIDE via a second rosbridge
  subscription — `{"state": "SEARCHING", "target_class": "dog", "fps": ~8-10}`.
- API heal (the button's backend): 409 "stop the runner first" while the
  runner was up; after stop, `POST /api/heal` → `{ok: true, seconds: 38}`,
  stack healthy after. Still pending: the literal browser button click,
  which needs a natural CMD FAULT while the dashboard is open.

OPERATIONAL GOTCHA found during validation: a STALE `serve_dashboard.py`
left running across a code upgrade keeps serving the OLD module, and on
Windows a second instance binds the same port anyway (SO_REUSEADDR), so
requests split between old and new code — symptom: new endpoints 404
"randomly" while everything else works. After pulling new dashboard-server
code, kill every old `serve_dashboard.py` (check `Get-Process python` /
port 8000's owner) before starting the new one.
