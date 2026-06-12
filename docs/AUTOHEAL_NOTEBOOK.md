# Rosbridge Auto-Heal Engineering Notebook

Reliability work for the Transbot SE AI stack. Written 2026-06-12, the day
the feature was designed, built, and live-validated. Companion documents:
the approved design in docs/superpowers/specs/2026-06-12-rosbridge-autoheal-design.md,
the task plan with execution status in
docs/superpowers/plans/2026-06-12-rosbridge-autoheal.md, and the running
lab log in AI_NOTEBOOK.md. The stage notebooks (GIMBAL_TRACKING_NOTEBOOK.md,
PERSON_FOLLOWING_NOTEBOOK.md) record the incidents that motivated this work.

## 1. Goal and scope

The robot's rosbridge has a recurring failure mode that silently discards
commands. Before this work, the cure was a human noticing odd behavior,
diagnosing it, and restarting a service over SSH — sometimes twice in one
evening. After this work, the AI runner cures it by itself before any
behavior starts, the dashboard cures it in one click, and a CLI covers
everything else. A second, smaller item rode along: a target-class picker
on the dashboard AI panel, exposing the runner's existing dog-mode flag.

This was chosen over starting stage 3 (ArUco waypoint navigation) on the
reasoning that every future stage pays the reliability tax, so de-flaking
the platform first makes all of them cheaper.

## 2. The bug this exists for

The robot runs ROS Melodic with rosbridge 0.11. That rosbridge can
half-fail a publisher registration: it accepts the client's advertise
message, but never actually registers itself as a publisher of the topic
on the ROS side. Every message the client then publishes on that topic is
discarded inside the bridge. No error of any kind reaches the websocket
client.

Properties established across the incident history (2026-06-10 and twice
on 2026-06-11):

- It is per topic, not per session. A session can have a perfectly working
  /PWMServo while /ai/cmd_vel and /ai/enabled are dead. The observed
  symptom that finally exposed this was "gimbal tracks fine, arm does
  nothing" during stage-2 bring-up.
- It correlates with boot. It hit roughly every other session near robot
  startup, and the live-validation session in section 9 reproduced it four
  minutes after power-on.
- The corrupt state can outlive every client disconnect. Sometimes a fresh
  websocket session re-registers cleanly; sometimes only a service restart
  clears it.
- The robot-side journal shows "Internal error processing topic" at
  registration time and "is not a publisher of [topic]" at publish time.
  The robot has no RTC and boots believing it is 2021, so journalctl
  --since filters lie; read the whole boot.

The reliable cure for every variant: sudo systemctl restart
rosbridge-dashboard.service on the robot. Passwordless SSH and
passwordless sudo for exactly this were provisioned during bring-up.

One cost makes the cure non-trivial to automate: that service runs the
entire robot stack — bringup (motor and servo drivers), usb_cam,
web_video_server, and rosbridge. A restart therefore drops the camera
stream and every client connection for fifteen to thirty seconds. Any
automation must guarantee the robot is stationary and nothing is
mid-behavior when it fires.

## 3. What existed before, and the gap

Two partial defenses existed before this work, built in response to the
incidents as they happened:

- The dashboard's actuation self-check (dashboard/js/actuation_check.js)
  already did the honest per-topic check: ask rosapi — which runs inside
  the rosbridge process, so it cannot be fooled by a half-registration —
  whether /rosbridge_websocket is registered as a publisher of each
  command topic. On failure it forced up to two fresh sessions, then lit a
  CMD FAULT chip and told the operator to go restart the service by hand.
- The AI runner's preflight (ai/common/connect.py) proved actuation end to
  end: command a tilt move, verify the camera image actually changed. This
  catches a dead /PWMServo but is blind to the per-topic variant — a
  session can pass the gimbal wiggle while the chassis topic is dead,
  which is precisely the stage-2 incident.

So detection existed in two different shapes, neither side could cure
anything, and the runner's detection had a known blind spot. The design
unified them: one shared cure, the dashboard's registration check added to
the runner, and escalation policies suited to each context.

## 4. Design decisions

Three decisions were made by the operator during design, recorded here
with the reasoning:

- The runner heals fully automatically. The preflight runs before arming
  is possible, so the robot is guaranteed stationary; there is no human in
  the loop to protect. Requiring a confirmation would reintroduce a manual
  step at exactly the moment automation should save the evening.
- The dashboard heals only on an explicit button click. A dashboard
  session can be mid-drive; an automatic restart yanking the whole robot
  stack out from under the operator without warning was judged worse than
  one extra click. The server additionally refuses to heal while a
  behavior process is running.
- A robot-side self-heal daemon was rejected. The robot cannot know which
  client topics ought to exist, robot-side deploys are higher friction
  than laptop-side ones, and a daemon would not help the runner decide
  whether to trust a session — which is half the problem.

One structural deviation from the spec was made during planning: the spec
placed the heal logic in tools/heal_rosbridge.py; the implementation put
it in ai/common/heal.py with the tools script as a thin CLI, because the
runner and the test suite import it as a package module. Same surface,
better import hygiene.

## 5. The shared cure

ai/common/heal.py owns the cure and is the only place that knows how to
restart the robot. One function: SSH to the robot (BatchMode so a missing
key fails fast instead of prompting; sudo -n likewise), run the systemctl
restart, then poll the rosbridge TCP port every two seconds until it
accepts connections again or a sixty-second budget expires. Returns a
boolean plus a human-readable detail string that every caller surfaces
verbatim — SSH stderr when the key or host is wrong, a timeout message
when the service restarted but the port never returned.

The command is built as an argument list and run without a shell, so the
host value (which only ever comes from the profile config) could not
inject anything even if it were attacker-controlled. The subprocess call
itself is not unit-tested, per the suite's convention that connection
paths are exercised live; the command shape, failure reporting, and
wait-for-port loop are tested with injected fakes.

Callers, all sharing this one function:

- the runner preflight (section 6), automatically
- POST /api/heal on the dashboard server (section 7), on operator click
- python tools/heal_rosbridge.py, by hand; this also turned out to be the
  one-command cure for the previously known wedged /voltage publisher
  after power cycles

## 6. Runner preflight: the two-phase ladder

connect_with_actuation_check in ai/common/connect.py is the gate every
behavior passes before its control loop starts. It was rewritten from a
single wiggle-with-retries into a two-phase check with an escalation
ladder.

Phase one, registration: for each topic the behavior publishes —
/PWMServo, /ai/cmd_vel, /ai/status — ask rosapi whether
/rosbridge_websocket is registered as a publisher. This is the same check
the dashboard runs and it would have caught both 2026-06-11 incidents.
All three topics are checked even for the gimbal-only face tracker,
because the shared RosClient advertises all three for every behavior, so
the check is truthful regardless of which behavior is running. If rosapi
itself does not answer (the mock server has no rosapi; a dying bridge may
not either), the phase reports unverified and the wiggle phase decides
alone — silence is never treated as a fault, matching the dashboard's
semantics.

Phase two, wiggle: the existing end-to-end proof, unchanged. Tilt the
gimbal twenty-five degrees, wait for the stream to show it, compare
frames. This stays even when registration passes because it proves the
entire chain — bridge, driver, servo, camera, stream — and the driver has
its own ways of being dead.

The escalation ladder, per process run: try a session; on any failure,
disconnect and try a fresh session (fresh sessions usually re-register
cleanly) up to three sessions total; if still failing, spend the single
allowed heal, reopen the video source, and try one final session; then
abort with an error naming exactly which topics were dead or that the
wiggle failed, what was already tried, and where to look on the robot.
One heal per run is a hard rule — a restart that did not cure the problem
will not be fixed by a second restart, and restart storms against a
flailing robot help nobody.

Two details cost real thought:

- The heal kills the MJPEG stream, so VideoSource grew a reopen method
  that retries opening the capture for up to sixty seconds while the
  camera comes back. Review of that change found a genuine latent hazard:
  the reader thread held self dot cap, so a thread surviving the
  two-second join timeout — most likely exactly when the stream just died,
  which is when reopen runs — could read from or release the new capture.
  The pump thread now binds its capture object locally at start, making
  the swap provably safe.
- Sessions are torn down with disconnect, never close: roslibpy shares one
  Twisted reactor per process and a reactor can never be restarted, so
  close mid-retry would brick every future connection attempt in that
  process. This was learned the hard way in stage 2 and is preserved here.

Known limitation, accepted and documented: the ladder only runs once a
session connects. A robot that is unreachable at process start — powered
off, wrong network — fails fast with a connection error and no heal
attempt. The heal cures a sick rosbridge, not an absent robot.

## 7. Dashboard path: endpoint and button

The dashboard cannot SSH from a browser, so the cure lives on the laptop's
static server (tools/serve_dashboard.py), which already ran a
localhost-only behavior API for starting and stopping the runner.

POST /api/heal: refuses with 409 if a behavior process is running (a heal
must never yank the robot stack out from under a live behavior — the
runner has its own heal anyway); validates the requested profile against
the AI config; runs the shared cure; returns ok, the detail string, and
elapsed seconds. The request blocks for the duration — up to about ninety
seconds worst case — which is fine because the server is threaded. Code
review added a non-blocking lock around the heal so two concurrent
requests cannot stack systemctl restarts; the second gets 409 "heal
already in progress". The one-restart-at-a-time invariant now lives in the
server, not just in the button's self-disable.

A race was identified and deliberately accepted: a behavior START arriving
during the thirty-second heal window launches a runner against a
restarting robot. The runner's own preflight ladder absorbs exactly this
(fresh sessions until the bridge is back), so the server does not try to
serialize START against heal.

The button: when the actuation self-check raises CMD FAULT, the chip in
the header grows a RESTART ROSBRIDGE button. It appears only if the
behavior API answers a probe — a statically hosted dashboard keeps the old
manual instructions and never shows a button it cannot honor. Click:
disable, show RESTARTING, post the heal, and on success kick the
websocket; the existing self-check re-runs on reconnect and clears the
chip if the cure took. On failure the button shows HEAL FAILED with the
server's detail in its tooltip for a few seconds.

The button logic went through two review rounds worth recording. The
visibility update awaited the API probe, so a slow probe started during a
fault could resolve after a newer all-clear and resurrect the button on a
healthy link. The first fix added a sequence number that newer fault
probes bump; re-review then caught that the all-clear path did not bump
it, so the specific interleaving fault-then-ok was still open. The final
shape: the ok path hides the button synchronously, bumps the sequence to
invalidate any probe still in flight, and skips probing entirely; only
fault events probe, and only the newest probe may apply its answer. A
small lesson in how async UI handlers on an event stream need explicit
epoch discipline even when each handler looks correct alone.

## 8. Target-class picker

The runner has had --target-class since stage 2 (the YOLO detector is
multi-class; following a dog was always a flag), and the behavior API
already accepted and validated the field. The gap was purely UI: nothing
on the dashboard could set it. The AI panel now has a TARGET dropdown —
person, dog, cat, a curated list in config.js rather than all eighty COCO
classes — sent with the next START, same applies-at-next-start rule as the
preview toggle. The BEHAVIOR readout already showed the running class from
/ai/status, so picker-versus-running mismatches are visible at a glance.

Two caveats, documented rather than solved here:

- The follow distance setpoint was tuned on a standing person's bounding
  box height. A dog fills less frame height, so the robot will read the
  distance as farther and close in. Live dog tuning is its own session.
- The runner matches the class label literally against detector output. A
  typo'd label that passes the server's validation pattern would start
  cleanly and simply never lock onto anything. The picker constrains
  values, so only a hand-edited config can hit this; validating the flag
  against the model's label set at startup is noted as future polish.

## 9. Test record

Unit suite: 97 tests before this work, 118 after. The new coverage, all
fake-driven per the suite's convention that connection paths are exercised
live:

| Area | What is pinned down |
| --- | --- |
| heal helper | command shape (BatchMode, sudo -n, service name), SSH failure reporting, wait-for-port loop, timeout |
| VideoSource.reopen | reopen on a real capture delivers frames again; missing source raises after the budget |
| registration check | all-registered, dead-topic naming, rosapi-silent means unverified not fault |
| ladder | happy path; dead topic to fresh sessions to heal to success; abort names dead topics; unverified falls through to wiggle; wiggle exhaustion spends the heal; failed heal aborts with its detail; heal fires at most once; mixed failure across the heal boundary reports the last symptom plus that a restart was tried; unverified plus dead wiggle still aborts |

The mixed-failure and unverified-plus-dead-wiggle cases were added at a
reviewer's prompting — every original failing test used a uniform failure
mode, which would not have pinned the abort message's behavior when the
failure mode changes across the heal boundary.

Mock-server checks: the dashboard self-check reports unverified against
the mock (no rosapi), so no fault and no button; a heal click on the mock
profile would 400 since the mock is not in the AI config's profiles;
the mock protocol selftest stayed at five of five throughout.

## 10. Live validation, 2026-06-12

Robot on home Wi-Fi, validated about four minutes after boot — prime time
for the bug, as it turned out. All automatable checklist items passed:

- CLI heal with a healthy stack: restart issued, rosbridge port back,
  camera and rosbridge reachable seconds later, service active.
- Runner preflight, person mode: registration silent-pass, wiggle pass,
  actuation check passed on attempt one, runner stable while disarmed.
- The ladder fired for real, unprompted. The dog-mode start hit a genuine
  wiggle failure on its first session — commands not reaching the gimbal,
  the classic dropped-session behavior, with registration having passed —
  and the ladder forced a fresh session and passed on attempt two. Zero
  operator involvement. The feature met its exact target failure within
  two hours of being pushed.
- Dog mode end to end: a second rosbridge subscription on the laptop
  confirmed /ai/status arriving robot-side with state SEARCHING, target
  class dog, at eight to ten detection fps. Verifying status robot-side
  (not just sent) is the stage-2 lesson about per-topic deadness applied
  to validation itself.
- The button's backend, live: /api/heal returned 409 stop-the-runner-first
  while the runner was up; after stopping it, the real call healed the
  robot in thirty-eight seconds and the stack came back healthy.

Still pending, by nature: the literal browser click. It needs a natural
CMD FAULT while the dashboard is open in a browser; everything behind the
click is live-proven. The bug's boot-time habit suggests the wait will not
be long.

## 11. An operational gotcha found during validation

During the live session, the new /api/heal endpoint intermittently
returned 404 from a server that demonstrably had it. Cause: a
serve_dashboard.py instance from earlier that morning — running the
pre-feature module as loaded at its start time — was still alive, and on
Windows a second instance binds the same port anyway (SO_REUSEADDR
semantics differ from Unix), so requests split between old and new code.
Everything the old server also implemented worked; only the new endpoint
404'd, and only sometimes.

Rule going forward: after pulling dashboard-server changes, kill every old
serve_dashboard.py before starting the new one — check which process owns
port 8000 if in doubt. A long-running Python server serves the code it
loaded at start, not the code on disk.

## 12. What this buys, and what is next

Each future stage — ArUco waypoint navigation, pick and place — inherits a
preflight that turns the platform's worst recurring failure from an
evening-killer into a log line. The candidates noted for later: validate
--target-class against the model's label set at startup; consider healing
on connect-refused (the down-at-start gap) if it ever bites in practice;
dog-mode controller tuning; and the browser-click confirmation whenever a
natural CMD FAULT next appears.
