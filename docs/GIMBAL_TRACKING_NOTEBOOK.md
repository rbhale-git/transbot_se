# Transbot SE — Gimbal Face-Tracking Engineering Notebook (AI Stage 1)

Project: autonomous face tracking on the camera gimbal of a Yahboom Transbot SE
Repository: https://github.com/rbhale-git/transbot_se (ai/ package, commit 391a256)
Period: 2026-06-10, single build-and-tune day with the robot live
Status: complete and operator-accepted; first behavior of the AI roadmap

## 1. Objective and Scope

Stage 1 of the agreed AI roadmap: detect the nearest human face in the robot's
camera and steer the 2-axis camera gimbal to keep it centered. Chassis motion
is explicitly out of scope — this behavior publishes only to the gimbal topic
and is physically incapable of driving the tracks. The stage exists to prove
the full perceive-error-command loop (frames in over Wi-Fi, commands out over
rosbridge) and to surface every infrastructure problem in a harmless setting
before stage 2 puts the same loop in charge of chassis velocity.

Both goals were met. The control loop works and is calibrated against measured
hardware behavior, and the stage surfaced two robot-side landmines (a
command-eating rosbridge failure and a one-client video bottleneck) that would
have been dangerous or maddening to first discover on a moving robot.

## 2. System Context

Everything runs on the laptop in modern Python 3.13; the robot is an unmodified
appliance reached over Wi-Fi. This was a deliberate architecture decision:
iterate at laptop speed, port to the Jetson later only if a behavior must run
untethered.

| Element | Detail |
| --- | --- |
| Frames in | web_video_server MJPEG over HTTP :8080, /usb_cam/image_raw, 1280x720 at ~29 fps |
| Commands out | rosbridge WebSocket :9090 (JSON protocol) via roslibpy 2.0 |
| Actuator | /PWMServo, transbot_msgs/PWMServo = int32 id (1 pan, 2 tilt), int32 angle 0-180 |
| Gimbal home | pan 90, tilt 22 (operator-chosen, shared with dashboard config) |
| Direction signs | positive pan angle moves the view LEFT, positive tilt moves it UP; both axes therefore use sign -1 (servo step opposes image error). Confirmed live |
| Servo feedback | none — the driver never echoes PWM servo positions; commanded state is the only state |
| Detector | OpenCV YuNet (cv2.FaceDetectorYN), ONNX model in ai/models/ (232 KB), score threshold 0.7, detection on a 640-wide downscale |
| Robot stack | factory image: ROS Melodic / Ubuntu 18.04 / Python 2 on a Jetson Nano B01; our headless systemd unit runs bringup + usb_cam + web_video_server + rosbridge |

The absence of servo feedback shapes the whole design: the tracker keeps its
own commanded-angle state, syncs it to reality by driving the gimbal to home at
startup, and uses the camera image itself as the only closed-loop signal.

## 3. Architecture

One process per behavior; shared infrastructure lives in ai/common and is
reused by every later stage.

| Module | Responsibility |
| --- | --- |
| ai/config.py | mirrors the dashboard's verified robot facts (addresses, servo ids, ranges, homes, signs) plus all tracker tuning; single source of truth |
| ai/common/video.py | VideoSource: background thread drains the stream and keeps only the newest frame (a queued MJPEG stream adds seconds of control latency); paces recorded files to real time; frames_differ() view-change test |
| ai/common/ros_client.py | RosClient: roslibpy connection, explicit topic advertise, verified message building, disconnect-vs-close semantics (section 9) |
| ai/common/safety.py | clamp() and RateLimiter — every value leaving the laptop is clamped, command emission is rate-gated |
| ai/face_tracking/detector.py | YuNet wrapper normalizing to Detection bboxes; select_primary() = largest face wins (closest subject) |
| ai/face_tracking/tracker.py | GimbalTracker: the pure control law. No I/O, no hardware, fully unit-tested; maps (face center, frame size) to a list of (servo id, integer angle) commands |
| ai/face_tracking/__main__.py | the runner: wires camera, detector, tracker, ROS sink; preview overlay; dry-run / recorded-video / webcam modes; startup actuation self-check |
| ai/face_tracking/calibrate.py | measures loop latency and per-axis gain on the live robot (section 7) |

Data flow per control step: VideoSource latest frame -> downscale to 640 wide
-> YuNet detect -> select primary face -> scale center back to full frame ->
GimbalTracker.update() -> zero or more /PWMServo commands -> rosbridge.
Detection and preview run at frame rate; control decisions are gated to 10 Hz.

## 4. The Control Problem

The error signal is the face center's offset from the frame center, normalized
per axis to the range -1..+1 (so tuning is resolution-independent). The
actuator accepts absolute integer angles only, with no feedback and no
velocity control: the driver slews each PWM servo to the commanded angle at
its own fixed speed.

What makes this problem interesting is the measurement delay. The path from a
servo command to that motion being visible in a decoded frame on the laptop
crosses: serial servo bus, physical servo travel, camera exposure, MJPEG
encode on a Nano CPU that is already ~80% busy serving the stream, Wi-Fi,
and decode plus detection on the laptop. Measured end to end (section 7) this
is approximately 0.8 seconds — the controller is blind to the consequences of
any command for nearly a second.

A latency that large dominates every other design consideration. A continuous
controller that keeps stepping while it watches a stale image will always
travel past the target before it can see itself arrive, then correct back,
indefinitely — a limit cycle. We rediscovered this empirically (twice)
before restructuring the controller around the latency instead of fighting it.

## 5. Design Iterations — What Failed and Why

Four controller versions reached the robot in one session. The iteration table
is the core engineering record of this stage:

| Version | Control law | Key parameters | Live result | Diagnosis |
| --- | --- | --- | --- | --- |
| v1a | proportional steps at fixed rate | kp 8 deg/err, step cap 4 deg, 10 Hz (40 deg/s slew) | tilt swung up past the face, lost it, camera stuck on the ceiling | latency overshoot; hold-on-lost has no recovery path |
| v1b | PD + lost-target recovery | kp 5, kd 4, cap 2.5 deg (25 deg/s), recenter after 5 s lost | tracked; operator verdict: works but laggy and jittery | the jitter WAS the latency limit cycle, misread as detector noise |
| v2 | faster fine stepping + smoothed measurement | kp 6, kd 6, cap 1.8 deg at 20 Hz (36 deg/s), EMA 0.5 on face center | much worse: overshoots even on a still face, cannot settle | higher slew x same latency = bigger swings; the EMA added measurement lag on top; D-term authority halved per step at the doubled rate |
| v3 | move-and-settle | full-correction gain 20 deg/err, per-move clamp 12 deg, hold fire 0.4 s after each move | better than v2; still occasional overshoot with face loss | settle window (0.4 s) was HALF the real blind time (0.8 s): the tracker re-measured a stale frame and fired the same correction twice |
| v4 (final) | move-and-settle with measured constants | per-axis gains 41/20 (0.85x measured), clamp 20 deg, hold fire 1.1 s | one or two decisive hops, then stillness; operator-accepted | constants measured, not guessed (section 7) |

Three lessons condensed from the table:

- With ~1 s of loop latency, continuous control of a position-commanded servo
  cannot be tuned into stability — to avoid limit cycling, the allowed slew
  would have to be so low (roughly deadband-degrees per latency, about 4 deg/s
  here) that tracking would be useless. The fix is structural, not parametric:
  never command while blind.
- Operator feel is data. The v1b verdict "laggy AND jittery" was the latency
  limit cycle described from the outside; v2 (raising speed to fix lag) made
  the same mechanism worse. The correct response to two failed retunes was to
  stop tuning and measure (section 7).
- Every failure mode needs a recovery path. Hold-position-on-lost-target is
  correct for transient detection dropouts, but after an overshoot it left
  the camera staring at the ceiling forever; the recenter-after-5-s rule
  turned that dead end into a graceful reset.

## 6. Final Control Law (move-and-settle)

The shipped controller, in plain words:

1. Measure: take the newest frame's primary face center. While idle (not
   immediately after a move) the center estimate is exponentially smoothed
   (weight 0.5 on history) so detector bounding-box wobble cannot trigger
   phantom moves.
2. Deadband: if both normalized errors are within 0.06 (about 38 px
   horizontally at 1280), do nothing. A settled tracker is silent on the wire.
3. Move: otherwise command the FULL estimated correction in one move per axis:
   new angle = current + sign x gain x error, with gain 41 deg per error unit
   on pan and 20 on tilt (0.85x the measured true gains, so each move lands
   slightly short rather than past), clamped to 20 deg per move and to the
   0-180 servo range. Angles are kept as floats internally; a command is
   emitted only when the rounded integer angle actually changes.
4. Settle: after any move, hold fire for 11 control updates (~1.1 s at 10 Hz,
   covering the measured 0.8 s blind time plus servo travel margin). The
   smoothing history is also dropped — it describes where the face was in the
   OLD camera pose and would corrupt the next measurement.
5. Re-measure and repeat. Convergence from any offset takes one large hop plus
   at most one small trim.
6. Lost target: hold position (a stationary gimbal is safe and the face
   usually reappears); after ~5 s of continuous loss, drive once back to home
   and wait. Reacquisition resets all estimator state so stale history cannot
   kick the first new move.

Operator controls in the preview window: q or ESC quits; c homes the gimbal
and suppresses tracking for 2 s (without the suppression, tracking instantly
dragged it back off home, which read as "homing is broken"); t toggles
tracking entirely. Status line shows commanded pan/tilt and
TRACKING / NO FACE (n) / PAUSED, plus a DRY RUN tag when not publishing.

A derivative term and a continuous-stepping mode remain in the code (kd and
settle 0 flags) for experimentation, but the shipped configuration uses pure
proportional moves with settle gating — across a ~1 s blind window,
step-to-step derivative memory is meaningless.

## 7. Calibration — Measuring Instead of Guessing

After v3 the two unknowns were measured directly with
python -m ai.face_tracking.calibrate (subject stands still in frame):

1. Find the face (tilt scan upward from home — a close subject's face sits
   above the home view), then roughly center it.
2. For each axis: record the median face center over 5 frames, command a
   known +8 deg step, then sample the face center for 2.5 s. The time until
   the center first shifts beyond 20 px is the loop latency; the stable total
   shift, converted to normalized error units, gives the true
   degrees-per-error-unit gain. Return to start, repeat for the other axis.

| Quantity | Guessed (v3) | Measured | Shipped value |
| --- | --- | --- | --- |
| Loop latency, command to visible | 0.4 s | 0.83 s pan / 0.79 s tilt | settle 11 updates at 10 Hz (~1.1 s) |
| Pan full-correction gain | 20 deg/err | 48.5 deg/err | kp 41 (0.85x) |
| Tilt full-correction gain | 20 deg/err | 23.2 deg/err | kp 20 (0.85x) |

The measured numbers explained both residual symptoms exactly: the settle
window was half the blind time (hence double-fired corrections = overshoot and
face loss), and a single global gain was serving two axes whose true gains
differ by a factor of two (hence pan felt sluggish while tilt ran hot). The
pan/tilt gain ratio is just the camera's horizontal/vertical field-of-view
ratio expressed in servo degrees.

Recalibrate whenever the camera, its resolution, or the gimbal mechanics
change; apply gain x 0.85 to the per-axis kp_deg values in ai/config.py and
make settle_updates comfortably exceed measured latency x control rate.

## 8. Safety Envelope

| Property | Mechanism |
| --- | --- |
| Cannot drive the chassis | the process never publishes /cmd_vel — gimbal topic only; stage 2 chassis work is gated on the agreed command mux + dashboard arm/disarm panel |
| Bounded authority | every angle clamps to the servo range; every move clamps to 20 deg; commands are gated to the control rate |
| Quiet when satisfied | deadband + integer-change suppression: a centered, settled tracker sends nothing |
| Lost target | hold position; recenter once after ~5 s; never hunts |
| State sync | startup drives the gimbal to home because commanded state is the only state |
| Operator authority | e-stop and all manual dashboard controls act on the same driver and override by simply being later commands; q / c / t in the preview |
| Crash and link loss | the robot-side cmd_vel watchdog (bring-up phase 5) is unaffected; a dead tracker process leaves a stationary gimbal, which is safe |

## 9. Incident: the rosbridge Dropped-Session Bug

The single most expensive discovery of the day, found because the very first
live commands did nothing despite a healthy-looking connection.

Mechanism (from the robot's journal): when a websocket client advertises
/PWMServo, the Melodic-era rosbridge (0.11, Python 2) sometimes hits
"Internal error processing topic [/PWMServo]" while building its internal
publisher. The registration with the ROS master succeeds, so the driver
dutifully tries to connect inbound — and rosbridge then refuses it forever
with "[/rosbridge_websocket] is not a publisher of [/PWMServo]". Result:
every publish from that session is dropped, silently — no error ever reaches
the websocket client.

Observed behavior across two boots:

- It hits roughly every other fresh session during the first minutes after
  boot (the Nano is busiest then), and recurred ~25 minutes in.
- The corrupt publisher can OUTLIVE all client disconnects: at one point
  client_count was 0 and five consecutive brand-new sessions were all dead.
  Only sudo systemctl restart rosbridge-dashboard.service cleared it.
- Forensics gotcha: the Nano has no RTC battery and boots believing it is
  August 2021 until NTP syncs, so journalctl --since hides the evidence; use
  journalctl -b and grep for "is not a publisher".

Countermeasures shipped:

- Actuation self-check on every connect (runner and calibrator): wiggle the
  tilt servo 25 deg, wait 1.8 s, and verify the camera image actually changed
  using frames_differ (mean absolute difference on a 160x90 grayscale;
  thresholds calibrated live: static-scene MJPEG noise scores 4-8, real
  gimbal motion ~40). On failure, drop the session and reconnect fresh, up to
  3 attempts, then abort with the journal pointer. The camera is the only
  feedback channel this robot offers, so it is the only honest health check.
- roslibpy gotcha encoded in RosClient: terminate() stops the process-wide
  Twisted reactor, which can never be restarted — any reconnect logic that
  calls it bricks ROS connectivity for the rest of the process. Reconnect
  loops must use disconnect() (websocket close only); terminate only at exit.
- The startup homing doubles as a visible actuation check for the operator:
  if the gimbal does not physically snap to home when the tracker starts,
  the session is bad — quit and restart.

The general lesson mirrors the bring-up watchdog incident: a transport that
reports success is not evidence of actuation. Close the loop through physics.

## 10. Operational Constraints

| Constraint | Consequence |
| --- | --- |
| The Nano serves ONE 720p MJPEG client | a second stream client starves the first; the tracker's reader starts timing out. Close the dashboard video pane during AI runs; never attach diagnostic stream taps while a behavior is running (our own diagnostics polluted our latency measurements until this was understood) |
| ~0.8 s control-loop latency | any new vision behavior must be designed around it from day one; assume it, measure it, never guess it |
| No servo feedback | commanded state must be synced by commanding known poses; the camera is the only sensor of success |
| Robot clock boots in 2021 | journalctl -b for forensics; --since lies until NTP syncs |
| VPN on the laptop | NordVPN must be fully exited during robot work (bring-up finding, still true) |
| Robot reboot mid-session | everything reconnects, but the gimbal wakes at its boot pose and rosbridge re-enters its fragile window — expect the actuation check to earn its keep right after a reboot |

## 11. Test and Verification Record

The package was built test-first throughout (every behavior began as a failing
test), which mattered twice over: the pure control law could be redesigned
three times in one afternoon with zero fear of regression, and live debugging
could always trust that failures were in the robot or the transport, never in
untested control math.

54 tests, all passing at acceptance (python -m pytest ai/tests from repo root):

| Area | Tests | What is locked down |
| --- | --- | --- |
| tracker.py control law | 29 | proportional response and signs, per-axis gain override, deadband, per-move clamp, range clamp and pinned-at-limit silence, sub-degree accumulation, move-and-settle gating (hold fire, resume, deadband does not trigger settle, recenter clears it, raw re-measure after a move), EMA smoothing and its reset rules, derivative damping and its reset, lost-target hold, recenter-after-timeout fires exactly once |
| detector.py | 7 | largest-face selection, bbox center math, model loads, empty results on blank frames, changing input sizes, real-face detection near a known location on a photographic test image |
| video.py | 7 | end-to-end reads from a real generated video file, end-of-stream behavior, open-failure error, real-time pacing of recorded clips, frames_differ thresholds (identical, noise-level, shifted view) |
| ros_client.py | 3 | the verified wire shape of transbot_msgs/PWMServo, angle rounding to int, topic constants |
| safety.py | 7 | clamp edges, rate limiter window semantics (blocked calls do not reset the window) |
| integration | 1 | full pipeline: synthetic 720p clip with a real photographic face pasted off-center -> VideoSource -> YuNet -> tracker -> command stream is monotonic, correctly signed, and clamped |

Live acceptance (operator): decisive lock-on within one or two hops from any
offset, stillness when centered, follows walking in distinct deliberate steps,
auto-recenter after leaving the view, c and t keys behave as described.

## 12. How to Run

| Task | Command |
| --- | --- |
| Install | pip install -r ai/requirements.txt |
| Tests | python -m pytest ai/tests |
| Offline, laptop webcam, no commands | python -m ai.face_tracking --source 0 --dry-run |
| Offline, recorded clip | python -m ai.face_tracking --source clip.avi --dry-run |
| Robot camera, no commands | python -m ai.face_tracking --dry-run |
| Live | python -m ai.face_tracking (add --profile hotspot on the robot AP) |
| Record the stream while tracking | add --record out.avi (builds regression clips) |
| Recalibrate | python -m ai.face_tracking.calibrate |

Tuning flags (defaults in ai/config.py): --kp-pan / --kp-tilt (full-correction
gains), --max-step (per-move clamp), --settle (hold-fire updates; 0 restores
continuous stepping), --deadband, --smoothing, --rate, --lost-recenter,
--pan-sign / --tilt-sign (direction flips, should never be needed again),
--kd (continuous-mode damping). Symptom guide: lands short and needs a third
hop — raise the axis gain toward its measured value; ever overshoots — lower
gain or raise --settle; reacts to nothing — check the actuation self-check
output, then the journal.

## 13. Lessons Learned

| Incident / observation | Lesson |
| --- | --- |
| Three controller versions failed on the same latency in different costumes | with large measurement delay, restructure the controller around the delay (measure-move-wait); do not tune a continuous loop into submission |
| Two retunes made things worse before anything got better | when the second retune fails, stop turning knobs and measure the plant; one 20-line calibration script replaced an afternoon of guessing |
| Settle window half the real latency caused double-fired corrections | a wrong constant inside a correct architecture reproduces the failure the architecture was built to prevent |
| One gain served two axes with a 2:1 true-gain ratio | per-axis calibration is not optional when fields of view differ |
| rosbridge silently ate entire sessions of commands | transport success is not actuation; verify through physics (camera) on every connect |
| Corrupt rosbridge state survived all disconnects | client-side retries have a ceiling; document the server-side remedy and surface it in the error message |
| roslibpy terminate() bricked all reconnects in-process | library teardown semantics are architecture constraints; encode them in the wrapper, not in tribal memory |
| Our own diagnostic stream taps starved the tracker and skewed measurements | observe the observer effect; one camera client at a time on a Nano |
| Operator "laggy and jittery" was a precise description of a limit cycle | treat feel reports as instrument readings, not complaints |

## 14. Stage 2 Outlook

Person following adds chassis motion (/cmd_vel) and is blocked, by prior
agreement, on two prerequisites: a command-priority mux node (e-stop > manual
> AI, with a lower AI speed cap) and a dashboard AI arm/disarm panel with
status readout. Both are buildable and testable against the existing mock
without the robot present. Stage 1 hands stage 2 a calibrated latency number,
per-axis visual-servo gains, the self-healing connection layer, the
one-stream rule, and a tested pattern for latency-tolerant control.

End of notebook.
