# Person Following (AI Stage 2) — Design

Date: 2026-06-10
Status: Approved by operator (this session)

## Goal

The robot base follows a chosen target (person by default, dog selectable)
around the home, building on the stage-1 face-tracking work. This is the
first stage that commands `/cmd_vel` autonomously, so it also delivers the
two agreed safety prerequisites: a robot-side command-priority mux and a
dashboard AI arm/disarm panel.

Scope decision: prerequisites + follower are built in one effort, in that
order. The follower cannot be live-tested safely without them.

## Architecture

```
LAPTOP                                    ROBOT (Jetson Nano, Melodic/Py2)
─────────────────────────────             ──────────────────────────────
ai/person_following  ──ai/cmd_vel──────▶  cmd_vel_mux.py ──/cmd_vel──▶ driver
  (YOLO + lock tracker + controller)         ▲    │ /mux/status              ▲
                                             │    ▼                          │
dashboard ──manual/cmd_vel───────────────────┘  dashboard panel    cmd_vel_watchdog
          ──/ai/enabled (arm/disarm)──────▶ mux                    (unchanged)
```

Three components:

1. `robot/cmd_vel_mux.py` — new robot-side ROS node, sole publisher to
   `/cmd_vel`. Deployed exactly like `cmd_vel_watchdog.py` (scp into
   `~/transbot_ws/src/transbot_bringup/scripts/`, started from
   `robot/transbot_dashboard.launch`, shipped by `tools/deploy_robot.ps1`).
2. Dashboard — drive commands move to `manual/cmd_vel`; new AI panel with
   ARM/DISARM and status readout.
3. `ai/person_following/` — new laptop-side behavior package, sibling of
   `ai/face_tracking`, reusing `ai/common` (video, ros client, safety).

The existing robot-side `cmd_vel_watchdog.py` is unchanged and remains the
last line of defense (zeroes `/cmd_vel` after 0.5 s of silence while moving).

## Component: robot-side mux (`robot/cmd_vel_mux.py`)

Python 2 / rospy, ~100 lines. Subscribes `manual/cmd_vel`, `ai/cmd_vel`,
`/ai/enabled` (std_msgs/Bool, latched). Publishes `/cmd_vel` and
`/mux/status`.

Arbitration rules, in priority order:

- A manual message is always forwarded immediately and stamps manual
  activity. Manual activity suppresses AI input for 1.0 s after the last
  manual message (touch the joystick → instant takeover, hold until 1 s of
  manual silence). The dashboard e-stop is a manual zero, so it preempts
  everything.
- An AI message is forwarded only if: armed AND no manual message in the
  last 1.0 s. It is then clamped in the mux to the AI caps:
  - linear forward: 0.25 m/s
  - linear reverse: 0.12 m/s
  - angular: 1.2 rad/s
  (Manual caps stay 0.45 / 2.0, enforced dashboard-side as today.)
- On disarm, and on AI-input silence > 0.5 s while AI was the active
  source, the mux publishes a single zero Twist (instant halt, no coasting
  on the AI's last command).
- `/mux/status` at 2 Hz: JSON string with active source (manual/ai/none),
  armed flag, AI caps. Consumed by the dashboard panel.

Implementation note: arbitration logic lives in a plain class separate from
the rospy plumbing, written Python 2/3 compatible, unit-tested with pytest
on the laptop before deployment (same approach as the watchdog).

Failure modes (all end stopped): AI crash / link loss → AI silence → mux
zero + watchdog. Mux crash → no `/cmd_vel` publisher → watchdog zero.
Buggy AI commands → clamped in mux; disarm or joystick always wins.

## Component: dashboard changes

- `config.js`: drive publish topic becomes `manual/cmd_vel` (topic is
  already config-driven); add AI topic names (`/ai/enabled`, `/mux/status`,
  `/ai/status`) and AI caps for display.
- New AI panel:
  - ARM / DISARM toggle publishing latched `/ai/enabled`. Disarmed is the
    default at every page load; the panel never auto-arms.
  - Status readout: who is driving (from `/mux/status`) and behavior state
    (from `/ai/status`: state, target class, detection fps).

## Component: person follower (`ai/person_following/`)

**Detector** — YOLO nano (YOLO11n, ONNX, ~6 MB) in `ai/models/`, run via
OpenCV DNN (no new heavy runtime dependency; same stack as YuNet). Target
class is config: `person` default, `dog` selectable (COCO classes). The
first implementation task is a laptop benchmark to confirm ≥10 fps at
640-px input; YuNet stays for face-specific behaviors.

**Lock tracker** — target selection is "lock at arm, stop on loss", no
appearance re-ID (rejected: silent identity swaps are the worst failure
shape for a follower; off-the-shelf re-ID is fragile at this camera height
and lighting; extra compute stretches an already ~0.8 s blind loop).
State machine:

- SEARCHING: armed, no target. Robot still. Lock the largest detection of
  the target class when one appears.
- FOLLOWING: associate the locked box frame-to-frame by IoU overlap.
- LOST: no association for ~0.5 s → stop and hold. When a target-class
  detection reappears, re-lock the largest one (operator accepts that
  re-acquire is "largest", not identity-matched).

**Controller** — continuous 10 Hz publish to `ai/cmd_vel`:

- Angular: P on bbox x-offset from frame center, with deadband. The gimbal
  is fixed at a follow pose (configurable pan/tilt; chassis does the
  turning).
- Linear: P on bbox-height error vs a follow-distance setpoint (bbox height
  fraction as distance proxy). Too close → slow reverse, capped at
  0.12 m/s and limited to ~1.5 s continuous reverse, then stop and wait
  (no rear sensors — no blind cross-room reversing).
- Lost or disarmed → publish zeros. The AI node also self-clamps to the
  same caps as the mux and refuses to publish motion when disarmed
  (defense in depth; the mux is the enforcement, this is hygiene).
- Gains start low. Known risk: stream latency (~0.8 s) may make continuous
  chassis control oscillate; mitigations in order — EMA smoothing on the
  bbox, lower gains, then the stage-1 move-and-settle pattern as fallback.
  A calibration run measures chassis response before gain tuning.

**Runner** (`python -m ai.person_following`) — reuses the stage-1
connect-with-actuation-check pattern (rosbridge half-failure mitigation,
RosClient.disconnect() between retries), preview window with detection
boxes / lock state / commanded velocities, keyboard pause and quit.
Remember: the Nano serves only ONE MJPEG client — no diagnostic taps while
a behavior runs.

## Testing & rollout order

1. Mux: pytest unit tests for arbitration logic (priority window, caps,
   halt-on-disarm, AI-silence zero) → deploy to robot → verify with
   dashboard alone (manual drive through mux, arm/disarm round-trip,
   status readout) before any AI publisher exists.
2. Dashboard panel: tested against `mock/mock_rosbridge.py` (extend mock
   with the new topics).
3. Follower offline: detector validated on still images (person + dog);
   tracker/controller unit-tested; full behavior run against recorded
   video of walking around the house — no robot.
4. First live run: half caps, open space, operator on the joystick (manual
   override is the tested takeover path).

Documentation: notebook-style write-up in docs/ after live tuning, per
project convention (BRINGUP_NOTEBOOK.md / GIMBAL_TRACKING_NOTEBOOK.md
pattern; tools/md_to_docx.py converter takes no bold/backticks/code-fences).

## Decisions log

- Scope: prerequisites + follower in one effort (operator, this session).
- Detector: YOLO nano, multi-class, config target class — also chosen as
  the reusable foundation for stage 3/4 (find-and-approach for pick and
  place; final grasp alignment will still need markers/close-range
  geometry).
- Targeting: lock at arm, stop on loss, no re-ID (operator chose after
  trade-off review).
- Too close: slow reverse allowed (operator), capped + time-limited.
- Mux: custom robot-side node (Option A) over twist_mux (apt fragility,
  no built-in speed capping, inverted lock semantics) and over laptop-side
  gating (self-enforcement, command interleaving).
