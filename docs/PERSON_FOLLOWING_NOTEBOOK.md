# Person Following Engineering Notebook

AI stage 2 for the Transbot SE. Written 2026-06-11 after the first live
session. Companion documents: the approved design in
docs/superpowers/specs/2026-06-10-person-following-design.md, the task plan
with execution status in docs/superpowers/plans/2026-06-10-person-following.md,
and the stage-1 gimbal notebook (GIMBAL_TRACKING_NOTEBOOK.md), whose latency
lessons this stage inherited.

## 1. Goal and scope

The robot base follows a chosen target around the home. Person by default;
the detector is multi-class, so dog following is a flag, not a feature.
This is the first stage where an autonomous process commands the chassis,
so the bulk of the engineering went into the safety chain, not the
following itself.

Status at time of writing: validated indoors at half speed caps in a small
room. Open-space validation at full caps is pending (task 12b in the plan).

## 2. Safety architecture

ROS does not arbitrate between publishers, so before this stage the
dashboard was the only thing allowed to publish /cmd_vel. Stage 2 added a
robot-side command-priority mux which is now the sole publisher of
/cmd_vel. Everything else goes through it.

- The dashboard publishes drive commands to /manual/cmd_vel. The mux
  forwards them always, unmodified, and any manual message suppresses AI
  input for one second. Grabbing the joystick mid-follow takes over
  instantly; the AI resumes one second after release.
- AI behaviors publish to /ai/cmd_vel. The mux forwards these only when the
  operator has armed AI from the dashboard panel, and clamps them to the AI
  caps no matter what the laptop sends: 0.25 m/s forward, 0.12 m/s reverse,
  1.2 rad/s turn. Manual caps are roughly double, so a buggy AI process is
  capped to slower-than-teleop speeds at the robot, not the laptop.
- Arming is a latched boolean on /ai/enabled set by a dashboard switch. The
  switch defaults to disarmed on every page load and reconnect, and the
  spacebar e-stop both stops the robot and disarms AI.
- On disarm, and on AI silence while AI was driving, the mux publishes one
  zero so the robot halts instead of coasting on the last command.
- The stage-0 cmd_vel watchdog is unchanged underneath all of this: any
  /cmd_vel silence while moving still zeroes the robot within half a second.

Every failure direction lands on stopped: AI crash, laptop sleep, link
loss, mux crash, disarm, or operator override.

The laptop-side follower also self-clamps to the same caps and refuses to
publish when disarmed. That is hygiene, not enforcement; the mux is the
enforcement point.

## 3. Detector

YOLO11n exported once to ONNX (about 10 MB, in-repo) and run with OpenCV
DNN, the same runtime stack as the stage-1 YuNet face detector, so the AI
package gained no new dependencies. 80 COCO classes; the follow target
class is configuration. The same model is intended to serve stage 3 and 4
find-and-approach work.

Throughput gate from the design: at least 10 fps on the laptop. Measured
12 fps at 640 px input on the dev laptop (i7-13700H, CPU only). Live
detection rate is lower (5 to 8 fps) because the stream, not the detector,
is the limiter; the controller runs at 10 Hz on the freshest frame either
way.

Decode notes for future reference: the YOLO11 ONNX head outputs 84 rows by
8400 anchors with no objectness term. Frames are letterboxed top-left with
gray padding, so mapping boxes back to frame coordinates is one divide and
no pad offset.

## 4. Targeting: lock at arm, stop on loss, no re-identification

Appearance re-ID was considered and rejected. When re-ID mismatches it
silently follows the wrong person, which is the worst failure shape a
follower can have, and off-the-shelf re-ID models are trained on
surveillance viewpoints that look nothing like a knee-height robot camera.

What shipped instead: at arm time the largest detection of the target class
is locked; association frame to frame is bounding-box overlap only; if
overlap fails for half a second the robot stops and waits; when a target
reappears, the largest detection is re-locked. The failure mode is always
stop. A second person crossing through does not steal the lock while the
original target stays visible, which the recorded-clip validation
confirmed.

## 5. Controller

Two independent P loops with deadbands, both clamped to the mux caps.

- Angular: proportional on the target's horizontal offset from frame
  center. The gimbal stays parked at a fixed follow pose; the chassis does
  the turning.
- Linear: proportional on bounding-box height fraction versus a setpoint.
  Box height is the distance proxy: bigger box, closer person.
- Too close commands slow reverse, capped at 0.12 m/s and limited to 1.5
  seconds of continuous reverse, after which the robot holds still until
  demand goes non-negative. The robot has no rear sensors; it does not get
  to back across a room blind. Note for tuning: a target that flickers
  fully lost and re-locked resets the reverse timer, so pathological
  flicker can exceed 1.5 s total reverse in segments of at most 18 cm each.
  The mux caps and watchdog still bound this.

## 6. What the live session actually taught us (2026-06-11)

### 6.1 The rosbridge registration bug is per topic

Stage 1 documented rosbridge sometimes half-failing its publisher
registration and silently dropping every command for a session, with the
journal signature "is not a publisher", cured only by restarting the
rosbridge service. Stage 2 learned the harder version: the failure is per
topic registration, not per session. One session passed the gimbal
actuation check (PWMServo registered fine) while /ai/enabled, /ai/status
and /ai/cmd_vel were all dead. The runner believed it was armed, the
dashboard believed it had armed it, and nothing reached the actual ROS
graph. Symptom at the operator level: arm AI and nothing happens.

Protocol that now stands before trusting any live session:

- Actuation check passes (gimbal wiggle changes the camera image).
- /ai/status visibly arrives robot-side (rostopic echo on the robot).
- Arming on the dashboard flips armed in /mux/status (proves /ai/enabled).
- /ai/cmd_vel messages arrive robot-side once the runner is following.

The bug struck twice in one evening. It costs one service restart and two
minutes each time it appears.

### 6.2 The angular sign confusion, and how to never have it again

First armed test: the robot turned away from the operator. The obvious
diagnosis (sign flipped) was wrong, and flipping the sign made it
genuinely wrong. The robot was actually turning toward the operator and
sweeping past: the command-to-stream loop is roughly 0.8 seconds blind, so
the chassis cannot see that it reached center until far past it. A
limit cycle reads as turning away to a human watching the robot.

The deterministic way to settle direction questions, which took two
ten-second tests and should be the template for every future axis:

- Ground truth one: the dashboard A key sends positive angular and the
  robot nose visibly swings left, so the driver follows the standard ROS
  convention (positive z is counter-clockwise).
- Ground truth two: the operator steps to one side and the preview box
  moves the matching way, so the camera is not mirrored.

Given those two facts the correct sign is arithmetic, not experiment. The
original sign was right: target right of center commands negative z,
clockwise, toward the target.

### 6.3 Killing the limit cycle

Stage 1 solved this on the gimbal with move-and-settle. The chassis got
away with continuous control by making the proportional ramp-down land
inside a wider deadband before the latency could carry it through:

- kp angular 2.0 hunted hard. 1.0 still hunted. 0.6 settles.
- Deadband widened from 0.05 to 0.10 of frame width, about 6.5 degrees of
  bearing. Inside that the command is exactly zero.

If a future space makes it hunt again, the next rung is the stage-1
move-and-settle pattern, already proven on this hardware.

### 6.4 Stream latency creeps because the Nano encodes per client

Mid-session the operator noticed control latency growing over time.
Thermals were fine; the cause was CPU saturation: web_video_server
re-encodes the MJPEG stream separately for every client, and full 720p per
client put the four-core Nano at a load average near 7. Saturated encoders
queue, and queues are latency that grows.

Fixes, all server-side downscaling via stream URL parameters (width,
height, quality), no robot changes:

- AI behaviors now request 480p at quality 50 by default (config-level).
  The detector letterboxes to 640 px anyway; 720p was pure waste.
- The dashboard also defaults to the 480p feed, with a new HD button when
  picture quality matters for driving, and a stream on/off switch that
  frees an entire encoder while an AI behavior runs. Operator confirmed
  latency much better with the dashboard stream off.

Correction to a stage-0 belief: the robot serves more than one MJPEG client
at once. The one-client rule from bring-up was a load constraint wearing a
disguise, not a hard limit. Two 480p clients are fine; two 720p clients
are not.

### 6.5 Small quirks logged

- The robot rebooted mid-session (operator power cycle). After that boot
  the driver stopped publishing /voltage even though motion and servo
  commands worked; the dashboard battery widget read stale. Cosmetic;
  heals on the next stack restart; watch the OLED for battery meanwhile.
- Wi-Fi round-trip from the laptop ran around 150 ms this session versus
  60 ms at bring-up. Wireless conditions are part of the control loop;
  there is no software fix for a bad radio day.

## 7. Final constants (as committed in ai/config.py FOLLOW)

| Constant | Value | How it was chosen |
| --- | --- | --- |
| kp angular | 0.6 rad/s per unit offset | live tuned; 2.0 and 1.0 limit-cycled |
| angular deadband | 0.10 of frame width | live tuned with kp to land inside |
| angular sign | +1 | derived from two ground-truth tests, section 6.2 |
| kp linear | 1.2 m/s per unit height error | design value, behaved live |
| height setpoint | 0.8 | operator wants about 1 m follow distance |
| linear deadband | 0.05 | design value |
| smoothing | 0.5 EMA | design value |
| caps fwd / rev / ang | 0.25 / 0.12 / 1.2 | mirror the mux enforcement |
| reverse limit | 1.5 s continuous | design value, never tripped live |
| lost grace | 0.5 s | design value |
| minimum IoU | 0.2 | design value |
| detector input | 640 px letterbox | 12 fps on laptop, above the 10 fps gate |
| AI stream | 480p quality 50 | Nano per-client encoder load, section 6.4 |

## 8. Operator quick reference

Start order matters because of section 6.1:

1. Robot on, wait for the stack. Dashboard connects, drive a meter with W
   to prove the manual path.
2. Start the runner: python -m ai.person_following (add --cap-scale 0.5 in
   a new space, --target-class dog for the dog). Wait for the actuation
   check to pass.
3. Verify the session per the 6.1 protocol if anything seems off.
4. ARM AI on the dashboard. Stand two meters out, in view.
5. SPACE stops everything and disarms. Any WASD key takes over instantly.
   The t key in the preview pauses the behavior; q quits it.
6. DISARM before walking away: a latched armed state will drive the next
   runner the moment it starts.

## 9. Pending work

- Open-space validation at full caps: sustained straight-line follow, wide
  turns, long-range lost-and-reacquire, and a real walk-into-the-robot
  reverse test. The bedroom validated correctness, not performance
  envelope.
- Dashboard-initiated runner start/stop is a candidate quality-of-life
  feature: the browser cannot spawn laptop processes, so it would go
  through a small endpoint on the dashboard's serving script.
- The /voltage dropout and the per-topic registration verification could
  both become automated preflight checks in the runner.
