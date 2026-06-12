# Rosbridge Auto-Heal + Target-Class Picker — Design

**Date:** 2026-06-12
**Status:** Approved by operator
**Builds on:** stage 2 person following (`docs/PERSON_FOLLOWING_NOTEBOOK.md`),
the dashboard actuation self-check (`dashboard/js/actuation_check.js`), and
the runner actuation check (`ai/common/connect.py`).

## Problem

The robot's Melodic rosbridge (0.11) recurrently half-fails publisher
registration **per topic**: it accepts the client's advertise but never
becomes a publisher on the ROS side, and every command on that topic vanishes
silently. It hit twice in one evening on 2026-06-11. The only reliable cure
is `sudo systemctl restart rosbridge-dashboard.service` over SSH (passwordless
`sudo -n` is configured and verified).

Current defenses are uneven:

- The **dashboard** detects the bug per topic (rosapi publisher check), forces
  ≤2 fresh sessions, then flags CMD FAULT — but the cure is still a manual
  terminal trip.
- The **runner** only proves `/PWMServo` end-to-end (tilt wiggle +
  `frames_differ`). A session can pass that while `/ai/cmd_vel` and
  `/ai/status` are dead — the exact "arm does nothing" failure seen live.
  The runner never heals; it just gives up after 3 fresh sessions.

Separately, dog mode (`--target-class dog`) exists end-to-end in the runner
and the behavior API, but the dashboard has no way to select it.

## Goals

1. The runner verifies **every topic it publishes** before trusting a
   session, and cures the bug itself — fresh sessions first, SSH restart as
   the last resort — with no operator involvement.
2. The dashboard offers a one-click **RESTART ROSBRIDGE** button when its
   self-check raises CMD FAULT (operator-initiated, never automatic).
3. The AI panel gets a **target-class dropdown** (person default, dog, cat,
   curated list) applied at next runner START.

Operator decisions (2026-06-12): runner heals fully automatically; dashboard
heals only on button click; curated class list (not full COCO).

## Architecture

### 1. Heal helper — `tools/heal_rosbridge.py`

One module owning the cure, importable by the runner and the dashboard
server, and runnable by hand:

- `heal(host, timeout_s=60) -> (ok, detail)`: run
  `ssh -o BatchMode=yes -o ConnectTimeout=5 jetson@<host> sudo -n systemctl
  restart rosbridge-dashboard.service` (list-form `subprocess.run`, no
  shell), then poll the rosbridge websocket port every 2 s until it accepts
  a TCP connection again or the timeout expires. On failure `detail` carries
  the SSH stderr (no key, host down, sudo prompt) or "timed out".
- Host comes from the profile's `rosbridge_url` in `ai/config.py`
  (`RosClient._parse` already extracts host:port — reuse, don't duplicate).
  SSH user `jetson` is a module constant. No new config keys.
- CLI: `python tools/heal_rosbridge.py [--profile home]` for manual use.

The service runs the whole robot stack (bringup + usb_cam + web_video_server
+ rosbridge), so a heal drops the camera stream and all client connections
for ~15–30 s. Both call sites only run it with the robot stationary: the
runner heals before arming is possible, the dashboard heals on an explicit
click (and refuses while a behavior is running, below).

### 2. Runner preflight — per-topic check + escalation ladder

`RosClient` (`ai/common/ros_client.py`) gains `publishers_of(topic)` — a
`/rosapi/publishers` service call with a short timeout, raising on no-answer
so callers can distinguish *unverified* from *dead*.

`connect_with_actuation_check` (`ai/common/connect.py`) becomes a two-phase
ladder. One heal is allowed per process run, spendable by either phase:

- **Registration phase** (no camera needed): for each of `/PWMServo`,
  `/ai/cmd_vel`, `/ai/status`, verify `/rosbridge_websocket` appears in
  `publishers_of(topic)` — the same check `actuation_check.js` does, which
  would have caught both 2026-06-11 incidents. Dead topics → disconnect →
  fresh session (≤2) → heal → fresh session → abort. If rosapi itself does
  not answer (mock server, bridge mid-death), treat as **unverified** and
  fall through to the wiggle phase — never a fault, same semantics as the
  dashboard.
- **Wiggle phase** (existing): tilt +25° → `frames_differ`. Unchanged
  ladder of fresh sessions, but on exhaustion it may spend the heal if the
  registration phase didn't (a restart also bounces the servo driver and
  camera). After the heal: fresh session, retry the full ladder once, then
  abort with the existing journalctl message.

Because a heal kills the MJPEG stream, `VideoSource` (`ai/common/video.py`)
gains `reopen()` and the ladder calls it after every heal before re-reading
frames. `--dry-run` bypasses all of this (unchanged).

Abort messages name the dead topics and what was already tried, so the
operator's next step is obvious.

### 3. Dashboard heal — `POST /api/heal` + CMD FAULT button

`serve_dashboard.py` adds a localhost-only `POST /api/heal`:

- **409 while a behavior is running** ("stop the runner first") — a heal
  must never yank a session out from under a live behavior; the runner has
  its own heal anyway.
- Otherwise call `heal()` for the configured profile (body may carry
  `{"profile": "home"}`, validated against `ai.config.PROFILES`; default
  home) and return `{ok, detail, seconds}`. The request blocks until the
  service is back or timed out — the ThreadingHTTPServer keeps serving.

Dashboard UI: when the actuation self-check emits `fault`, the CMD FAULT
header chip gains a **RESTART ROSBRIDGE** button. Click → POST `/api/heal`
(button disabled, shows "RESTARTING…") → on `ok`, `kickConnection()`; the
existing self-check re-runs automatically on reconnect and clears the chip
if cured. On failure, show the `detail` text and keep the existing "restart
manually" guidance. If the behavior API is unreachable (dashboard not served
by `serve_dashboard.py`), the button hides — the AI panel already probes
this API, reuse that knowledge rather than a second probe.

### 4. Target-class picker

- `dashboard/js/config.js`: `AI.targetClasses = ['person', 'dog', 'cat']`
  (extend in one line; values must satisfy the server's existing
  `TARGET_CLASS_RE`).
- AI panel (05): a small select, default `person`, sent as `target_class`
  in the START body — the API and runner already support it end-to-end.
  Applies at next START, same rule as the PREVIEW toggle.
- The panel shows the active class from `/ai/status` (it already carries
  `target_class`), so "picker says dog, runner still on person" is visible
  at a glance.

Dog-mode caveat (documented, not solved here): `height_setpoint` 0.8 was
tuned for a standing person; a dog fills less frame height, so follow
distance will read closer. Live dog tuning is a separate operator session.

## Failure modes

- **SSH broken (no key / robot unreachable):** heal returns `(False,
  stderr)`; runner aborts with that detail; dashboard shows it on the chip.
  Nothing retries SSH in a loop.
- **Service restarts but rosbridge stays bad:** ladder re-verifies after the
  heal; if topics are still dead the runner aborts (one heal per run — no
  restart storms). Dashboard: chip stays CMD FAULT; operator escalates.
- **Heal mid-drive:** prevented by construction — runner heals only inside
  preflight (before arming), dashboard refuses while a behavior runs and
  otherwise acts only on operator click.
- **rosapi missing (mock server):** registration phase reports unverified
  and the wiggle phase decides — the mock keeps working for offline dev.
- **Wedged `/voltage` publisher after power cycle** (known quirk): the same
  manual CLI heal now cures it in one command — worth a notebook line.

## Testing

- Unit tests, existing fake-based style: ladder ordering (fresh ×2 → heal →
  fresh → abort), heal spent at most once across both phases, unverified ≠
  fault, dead-topic names in the abort message, `publishers_of` parsing,
  heal helper's wait-for-port logic with a fake socket/clock (the ssh
  subprocess call is NOT unit-tested — verified live).
- Mock server: dashboard button hidden (API absent → no /api/heal), check
  reports unverified, START body carries the picked class (selftest).
- Live validation: manual CLI heal once; runner preflight happy path; button
  path by inducing CMD FAULT if the bug obliges (it usually does near boot).

## Out of scope

- Robot-side self-heal daemon (rejected: can't know client topics, deploy
  friction).
- Automatic (clickless) dashboard heal — operator explicitly chose
  button-gated.
- Dog-mode controller tuning and live test; full COCO class list.
- Healing mid-behavior (runner exits → operator restarts via the panel).
