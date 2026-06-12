# Rosbridge Auto-Heal + Target-Class Picker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The AI runner verifies every topic it publishes and cures the rosbridge silent-drop bug itself (fresh sessions → one SSH service restart); the dashboard gets a one-click RESTART ROSBRIDGE button on CMD FAULT; the AI panel gets a target-class dropdown.

**Architecture:** One heal helper (`ai/common/heal.py`: SSH restart + wait-for-port) shared by the runner preflight, the dashboard server's `POST /api/heal`, and a manual CLI (`tools/heal_rosbridge.py`). The runner preflight (`ai/common/connect.py`) becomes a two-phase ladder: rosapi per-topic registration check, then the existing tilt-wiggle check; escalation is fresh sessions (≤2 extra) → ONE heal per run → one final fresh session → abort. `VideoSource.reopen()` recovers the MJPEG stream after a heal (the service restart takes the camera down too).

**Tech Stack:** Python 3 (`pytest`, `roslibpy`, `cv2`), vanilla-JS ES modules (no test harness — mock-server verification), Windows `ssh.exe` with the existing passwordless key to `jetson@robot`.

**Spec:** `docs/superpowers/specs/2026-06-12-rosbridge-autoheal-design.md`

**File map:**

| File | Change |
|---|---|
| `ai/common/heal.py` | NEW — `host_from_url`, `restart_command`, `heal()` |
| `ai/tests/test_heal.py` | NEW — heal helper unit tests (fakes for run/port/clock) |
| `ai/common/video.py` | `VideoSource._open()` refactor + `reopen()` |
| `ai/tests/test_video.py` | reopen tests |
| `ai/common/connect.py` | two-phase ladder + heal escalation (full rewrite) |
| `ai/tests/test_connect.py` | NEW — ladder tests with fake sink/src/heal |
| `ai/common/ros_client.py` | `publishers_of()` |
| `ai/person_following/__main__.py` | `RosSink.publishers_of` passthrough + heal wiring |
| `ai/face_tracking/__main__.py` | same wiring (connect is shared by both behaviors) |
| `tools/heal_rosbridge.py` | NEW — manual CLI |
| `tools/serve_dashboard.py` | `POST /api/heal` (409 while runner active) |
| `dashboard/index.html` | RESTART ROSBRIDGE button next to the CMD FAULT chip |
| `dashboard/js/app.js` | heal button logic in `initActuationWarning()` |
| `dashboard/js/config.js` | `AI.targetClasses` |
| `dashboard/js/ai_panel.js` | target-class select, sent in the START body |
| `dashboard/css/style.css` | `.warning-btn`, `#ai-class-select` rules |
| `docs/AI_NOTEBOOK.md`, `README.md` | auto-heal + picker documentation |

**Conventions (from the existing suite):** run tests with `python -m pytest ai/tests -q` from the repo root. Connection-path code (real roslibpy/ssh calls) is NOT unit-tested — it's exercised live; all decision logic takes injected fakes. `Date.now`-style wall-clock is irrelevant here; tests inject `clock`/`sleep`.

---

### Task 1: Heal helper (`ai/common/heal.py`)

**Files:**
- Create: `ai/common/heal.py`
- Test: `ai/tests/test_heal.py`

- [ ] **Step 1: Write the failing tests**

Create `ai/tests/test_heal.py`:

```python
"""Tests for ai.common.heal — the SSH rosbridge-restart cure.

The ssh subprocess itself is not unit-tested (verified live, like every
connection path in this suite); these tests cover the command shape, the
failure reporting, and the wait-for-port loop with injected fakes.
"""

import types

from ai.common.heal import (
    SERVICE, SSH_USER, heal, host_from_url, restart_command,
)


def proc(returncode=0, stderr="", stdout=""):
    return types.SimpleNamespace(returncode=returncode, stderr=stderr,
                                 stdout=stdout)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


class TestHostFromUrl:
    def test_strips_scheme_and_port(self):
        assert host_from_url("ws://192.168.0.109:9090") == "192.168.0.109"

    def test_no_port(self):
        assert host_from_url("ws://robot.local/") == "robot.local"


class TestRestartCommand:
    def test_non_interactive_ssh_and_sudo(self):
        cmd = restart_command("192.168.0.109")
        assert cmd[0] == "ssh"
        assert "BatchMode=yes" in cmd          # never hang on a password prompt
        assert f"{SSH_USER}@192.168.0.109" in cmd
        assert "sudo" in cmd and "-n" in cmd   # passwordless sudo only
        assert SERVICE in cmd


class TestHeal:
    def test_ssh_failure_reports_stderr(self):
        ok, detail = heal("h", run=lambda *a, **k: proc(255, "Permission denied"),
                          port_open=lambda h, p: True)
        assert ok is False
        assert "Permission denied" in detail

    def test_waits_for_port_to_return(self):
        clock = FakeClock()
        answers = iter([False, False, True])
        ok, detail = heal("h", run=lambda *a, **k: proc(),
                          port_open=lambda h, p: next(answers),
                          sleep=clock.sleep, clock=clock)
        assert ok is True

    def test_times_out_when_port_never_returns(self):
        clock = FakeClock()
        ok, detail = heal("h", timeout_s=10.0, run=lambda *a, **k: proc(),
                          port_open=lambda h, p: False,
                          sleep=clock.sleep, clock=clock)
        assert ok is False
        assert "not back" in detail
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ai/tests/test_heal.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'ai.common.heal'`

- [ ] **Step 3: Implement `ai/common/heal.py`**

```python
"""Cure the robot's rosbridge silent-drop bug from the laptop.

The Melodic rosbridge (0.11) recurrently half-fails publisher registration
per topic and silently discards every command on it; the only reliable cure
is restarting rosbridge-dashboard.service over SSH (passwordless `sudo -n`
is configured and verified — docs/AI_NOTEBOOK.md). The service runs the
WHOLE robot stack (bringup + usb_cam + web_video_server + rosbridge), so a
heal drops the camera and every client connection for ~15-30 s. Callers
must only run it with the robot stationary.

Used by: the behavior preflight (ai/common/connect.py), the dashboard
server's POST /api/heal (tools/serve_dashboard.py), and the manual CLI
(tools/heal_rosbridge.py). It also clears the wedged-/voltage-publisher
quirk seen after power cycles.
"""

import socket
import subprocess
import time

SSH_USER = "jetson"
SERVICE = "rosbridge-dashboard.service"
ROSBRIDGE_PORT = 9090
SSH_TIMEOUT_S = 30
POLL_INTERVAL_S = 2.0


def host_from_url(url):
    """ws://192.168.0.109:9090 -> 192.168.0.109 (dashboard-style URL)."""
    stripped = url.replace("ws://", "").rstrip("/")
    return stripped.partition(":")[0]


def restart_command(host):
    # BatchMode: fail fast instead of hanging on a password prompt if the
    # key is missing; sudo -n likewise (passwordless sudo is set up).
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
            f"{SSH_USER}@{host}",
            "sudo", "-n", "systemctl", "restart", SERVICE]


def _port_open(host, port, timeout_s=2.0):
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def heal(host, timeout_s=60.0, run=subprocess.run, port_open=_port_open,
         sleep=time.sleep, clock=time.monotonic):
    """Restart the robot stack, wait for rosbridge to accept connections.

    Returns (ok, detail). run/port_open/sleep/clock are injection points
    for the unit tests; production callers pass only host.
    """
    try:
        result = run(restart_command(host), capture_output=True, text=True,
                     timeout=SSH_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return False, "ssh timed out"
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        return False, detail or f"ssh exited {result.returncode}"
    deadline = clock() + timeout_s
    while clock() < deadline:
        if port_open(host, ROSBRIDGE_PORT):
            return True, f"{SERVICE} restarted; rosbridge port is back"
        sleep(POLL_INTERVAL_S)
    return False, (f"{SERVICE} restarted but port {ROSBRIDGE_PORT} "
                   f"not back within {timeout_s:.0f}s")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ai/tests/test_heal.py -q`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add ai/common/heal.py ai/tests/test_heal.py
git commit -m "Heal helper: SSH-restart rosbridge-dashboard.service + wait for port"
```

---

### Task 2: `VideoSource.reopen()`

A heal restarts the camera, killing the MJPEG stream; the preflight ladder needs to re-create the capture afterwards.

**Files:**
- Modify: `ai/common/video.py:38-62` (constructor → `_open()` refactor, new `reopen()`)
- Test: `ai/tests/test_video.py` (add to `TestVideoSource`)

- [ ] **Step 1: Write the failing tests**

Add to `class TestVideoSource` in `ai/tests/test_video.py`:

```python
    def test_reopen_reads_frames_again(self, tiny_video):
        # A heal restarts the robot stack and kills the MJPEG stream;
        # reopen() must bring a closed/dead source back to life.
        with VideoSource(tiny_video) as src:
            assert src.read(timeout_s=2.0) is not None
            src.reopen(timeout_s=5.0)
            assert src.alive
            assert src.read(timeout_s=2.0) is not None

    def test_reopen_raises_when_source_stays_gone(self, tiny_video, tmp_path):
        src = VideoSource(tiny_video)
        src.close()
        src._source = str(tmp_path / "missing.avi")
        with pytest.raises(RuntimeError):
            src.reopen(timeout_s=0.2, sleep=lambda s: None)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ai/tests/test_video.py -q`
Expected: 2 FAIL — `AttributeError: 'VideoSource' object has no attribute 'reopen'`

- [ ] **Step 3: Implement**

In `ai/common/video.py`, replace `VideoSource.__init__` with a thin constructor plus `_open()`, and add `reopen()` after `close()`. The class docstring keeps its current text; only the code below changes:

```python
    def __init__(self, source, pace_s=None):
        self._source = source
        self._pace_s = pace_s
        self._lock = threading.Condition()
        self._open()

    def _open(self):
        self._cap = cv2.VideoCapture(self._source)
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open video source: {self._source!r}")
        with self._lock:
            self._frame = None
            self._seq = 0
            self._returned_seq = 0
            self._alive = True
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
```

(`_pump`, `alive`, `read`, `close`, context-manager methods stay exactly as they are — `read()`'s `getattr(self, "_returned_seq", 0)` keeps working since `_open` now initializes it.)

```python
    def reopen(self, timeout_s=60.0, sleep=time.sleep, clock=time.monotonic):
        """Re-create the capture after the robot stack restarted.

        A rosbridge heal (ai/common/heal.py) takes web_video_server down
        with it; the camera comes back ~15-30 s later, so retry until the
        source opens or the timeout expires.
        """
        self.close()
        deadline = clock() + timeout_s
        while True:
            try:
                self._open()
                return
            except RuntimeError:
                if clock() >= deadline:
                    raise RuntimeError(
                        f"video source did not come back within "
                        f"{timeout_s:.0f}s: {self._source!r}")
                sleep(2.0)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ai/tests/test_video.py -q`
Expected: all pass (9 in this file: 4 existing + 2 new VideoSource, 3 FramesDiffer)

- [ ] **Step 5: Commit**

```bash
git add ai/common/video.py ai/tests/test_video.py
git commit -m "VideoSource.reopen(): recover the stream after a robot-stack restart"
```

---

### Task 3: Two-phase preflight ladder (`ai/common/connect.py`)

**Files:**
- Modify: `ai/common/connect.py` (full rewrite, it's 41 lines)
- Test: Create `ai/tests/test_connect.py`

- [ ] **Step 1: Write the failing tests**

Create `ai/tests/test_connect.py`:

```python
"""Tests for ai.common.connect — the two-phase actuation preflight ladder.

Everything is driven through fakes: a FakeSink whose rosapi answers are
scripted, a FakeSrc whose frames either change on command (wiggle passes)
or never change (wiggle fails), and a FakeHeal that records calls. The
real rosapi/wiggle paths are exercised live, per suite convention.
"""

import types

import numpy as np
import pytest

from ai.common.connect import (
    REGISTRATION_TOPICS, connect_with_actuation_check, registration_failures,
)

FRAME_A = np.zeros((48, 64, 3), dtype=np.uint8)
FRAME_B = np.full((48, 64, 3), 200, dtype=np.uint8)
TILT = types.SimpleNamespace(servo_id=2, home_deg=22)
NOSLEEP = lambda s: None  # noqa: E731

ALL_OK = {t: ["/rosbridge_websocket"] for t in REGISTRATION_TOPICS}
DEAD_CMD_VEL = {**ALL_OK, "/ai/cmd_vel": []}


class FakeSink:
    """publishers_by_topic=None scripts a silent rosapi (raises)."""

    def __init__(self, publishers_by_topic):
        self._pubs = publishers_by_topic
        self.sent = []
        self.disconnected = False

    def publishers_of(self, topic, timeout_s=3.0):
        if self._pubs is None:
            raise RuntimeError("rosapi silent")
        return self._pubs.get(topic, [])

    def send(self, servo_id, angle):
        self.sent.append((servo_id, angle))

    def disconnect(self):
        self.disconnected = True


class FakeSrc:
    """Alternates frames when `moving` (wiggle check passes), repeats one
    frame when not (wiggle check fails)."""

    def __init__(self, moving=True):
        self.moving = moving
        self.reopened = 0
        self._flip = False

    def read(self, timeout_s=0.0):
        self._flip = not self._flip
        return FRAME_B if (self.moving and self._flip) else FRAME_A

    def reopen(self, timeout_s=60.0):
        self.reopened += 1


class FakeHeal:
    def __init__(self, ok=True):
        self.ok = ok
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.ok, "fake heal detail"


def factory_of(*sinks):
    it = iter(sinks)
    return lambda: next(it)


class TestRegistrationFailures:
    def test_all_registered(self):
        assert registration_failures(FakeSink(ALL_OK)) == []

    def test_names_dead_topics(self):
        assert registration_failures(FakeSink(DEAD_CMD_VEL)) == ["/ai/cmd_vel"]

    def test_silent_rosapi_is_unverifiable_not_fault(self):
        assert registration_failures(FakeSink(None)) is None


class TestLadder:
    def test_happy_path_first_attempt(self):
        sink = FakeSink(ALL_OK)
        heal = FakeHeal()
        got = connect_with_actuation_check(
            factory_of(sink), FakeSrc(), TILT, heal=heal, sleep=NOSLEEP)
        assert got is sink
        assert not sink.disconnected
        assert heal.calls == 0

    def test_dead_topic_fresh_sessions_then_heal_then_success(self):
        bad = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        good = FakeSink(ALL_OK)
        heal = FakeHeal()
        src = FakeSrc()
        got = connect_with_actuation_check(
            factory_of(*bad, good), src, TILT, attempts=3, heal=heal,
            sleep=NOSLEEP)
        assert got is good
        assert heal.calls == 1
        assert src.reopened == 1
        assert all(s.disconnected for s in bad)

    def test_without_heal_raises_and_names_dead_topics(self):
        sinks = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        with pytest.raises(RuntimeError, match="/ai/cmd_vel"):
            connect_with_actuation_check(
                factory_of(*sinks), FakeSrc(), TILT, attempts=3,
                sleep=NOSLEEP)

    def test_silent_rosapi_falls_back_to_wiggle(self):
        sink = FakeSink(None)  # unverifiable, but the wiggle passes
        got = connect_with_actuation_check(
            factory_of(sink), FakeSrc(moving=True), TILT, sleep=NOSLEEP)
        assert got is sink

    def test_wiggle_failure_spends_heal_then_aborts(self):
        sinks = [FakeSink(ALL_OK) for _ in range(4)]
        heal = FakeHeal()
        src = FakeSrc(moving=False)
        with pytest.raises(RuntimeError, match="service restart"):
            connect_with_actuation_check(
                factory_of(*sinks), src, TILT, attempts=3, heal=heal,
                sleep=NOSLEEP)
        assert heal.calls == 1          # never more than one heal per run
        assert src.reopened == 1

    def test_failed_heal_aborts_with_detail(self):
        sinks = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        with pytest.raises(RuntimeError, match="fake heal detail"):
            connect_with_actuation_check(
                factory_of(*sinks), FakeSrc(), TILT, attempts=3,
                heal=FakeHeal(ok=False), sleep=NOSLEEP)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ai/tests/test_connect.py -q`
Expected: FAIL — `ImportError: cannot import name 'REGISTRATION_TOPICS'`

- [ ] **Step 3: Rewrite `ai/common/connect.py`**

Full new content:

```python
"""Connect to rosbridge and PROVE commands reach the robot.

The robot's old rosbridge (0.11/Melodic) can half-fail publisher
registration PER TOPIC and silently drop every command on it for the
session's lifetime — a session can wiggle the gimbal fine while
/ai/cmd_vel is dead ("arm does nothing", seen live 2026-06-11; docs/
AI_NOTEBOOK.md). No error reaches the websocket client, so each fresh
session is checked in two phases:

1. Registration: ask rosapi (which runs inside the rosbridge process)
   whether /rosbridge_websocket is registered as a publisher of every
   topic behaviors publish — the same check the dashboard runs. If rosapi
   itself does not answer (mock server), the phase is UNVERIFIED and the
   wiggle phase decides alone; never treated as a fault.
2. Wiggle: tilt the gimbal and verify the camera view changed —
   end-to-end proof through the servo driver.

Escalation: fresh sessions re-register cleanly, so reconnect up to
`attempts` times; if still bad, spend the ONE `heal` per run (SSH restart
of rosbridge-dashboard.service — ai/common/heal.py) and try one final
fresh session. The heal restarts the whole robot stack, so the video
source is reopened after it.

Shared by every behavior. The sink_factory must return an object with
send(servo_id, angle), publishers_of(topic), and disconnect();
disconnect() is used between retries because close() would kill the
process-wide Twisted reactor for good.
"""

import time

from ai.common.video import frames_differ

# Every topic RosClient advertises, for any behavior. Mirrors the
# constants in ai/common/ros_client.py — NOT imported from there, because
# behaviors import this module on dry runs where roslibpy may be absent
# (RosClient is deliberately imported lazily inside the sinks).
REGISTRATION_TOPICS = ("/PWMServo", "/ai/cmd_vel", "/ai/status")
ROSBRIDGE_NODE = "/rosbridge_websocket"


def registration_failures(sink, topics=REGISTRATION_TOPICS):
    """Topics rosbridge dropped ([] = all good), or None if unverifiable."""
    dead = []
    for topic in topics:
        try:
            publishers = sink.publishers_of(topic)
        except Exception:
            return None
        if ROSBRIDGE_NODE not in publishers:
            dead.append(topic)
    return dead


def _wiggle_moves_camera(sink, src, tilt_cfg, sleep):
    before = src.read(timeout_s=3.0)
    sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg + 25)
    sleep(1.8)
    after = src.read(timeout_s=3.0)
    sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg)
    sleep(1.0)
    return before is not None and after is not None and frames_differ(before, after)


def connect_with_actuation_check(sink_factory, src, tilt_cfg, attempts=3,
                                 heal=None, sleep=time.sleep):
    """Return a verified sink, or raise RuntimeError naming what failed."""
    healed = False
    budget = attempts
    attempt = 0
    last_fail = "no attempt made"
    while attempt < budget:
        attempt += 1
        sink = sink_factory()
        dead = registration_failures(sink)
        if dead is None:
            print("registration check unverified (rosapi did not answer) - "
                  "relying on the wiggle check alone")
        if not dead:  # [] verified-ok, or None unverified: wiggle decides
            if _wiggle_moves_camera(sink, src, tilt_cfg, sleep):
                print(f"actuation check passed (attempt {attempt})")
                return sink
            last_fail = ("commands are not reaching the gimbal "
                         "(tilt wiggle did not change the camera view)")
        else:
            last_fail = ("rosbridge dropped publisher registration for "
                         + ", ".join(dead))
        print(f"actuation check FAILED on attempt {attempt}: {last_fail} - "
              "reconnecting with a fresh session")
        sink.disconnect()
        if attempt == budget and heal is not None and not healed:
            healed = True
            print("escalating: restarting rosbridge-dashboard.service on "
                  "the robot (drops camera + link ~15-30 s) ...")
            ok, detail = heal()
            if not ok:
                raise RuntimeError(f"auto-heal failed ({detail}); "
                                   f"last actuation failure: {last_fail}")
            print("rosbridge is back - reopening the video stream ...")
            src.reopen()
            budget += 1  # one post-heal session
        sleep(2.0)
    raise RuntimeError(
        f"actuation check failed after {attempt} rosbridge sessions"
        + (" including a service restart" if healed else "")
        + f"; last failure: {last_fail} - check the robot "
        "(journalctl -u rosbridge-dashboard.service)")
```

- [ ] **Step 4: Run the new tests, then the whole suite**

Run: `python -m pytest ai/tests/test_connect.py -q` → 9 passed
Run: `python -m pytest ai/tests -q` → all pass (the old suite had 97 + new heal/video/connect tests; nothing else imports `connect_with_actuation_check`'s removed internals)

- [ ] **Step 5: Commit**

```bash
git add ai/common/connect.py ai/tests/test_connect.py
git commit -m "Preflight ladder: per-topic registration check + one-heal escalation"
```

---

### Task 4: Wire the runners (`publishers_of` passthrough + heal)

**Files:**
- Modify: `ai/common/ros_client.py` (add `publishers_of`)
- Modify: `ai/person_following/__main__.py:62-95,190-196` (RosSink + connect call)
- Modify: `ai/face_tracking/__main__.py:44-61,151-157` (same)

No new unit tests: `publishers_of` is a connection-path method (suite convention: exercised live), and the ladder logic is already covered by Task 3.

- [ ] **Step 1: Add `publishers_of` to `RosClient`**

In `ai/common/ros_client.py`, add constants after `AI_ENABLED_TYPE`:

```python
ROSAPI_PUBLISHERS_SERVICE = "/rosapi/publishers"
ROSAPI_PUBLISHERS_TYPE = "rosapi/Publishers"
```

and the method after `send_status`:

```python
    def publishers_of(self, topic, timeout_s=3.0):
        """Nodes registered as publishers of `topic`, straight from rosapi
        (which runs inside the rosbridge process — the honest check for
        the silent-drop registration bug). Raises on no answer; callers
        treat that as 'unverifiable', never as a fault."""
        service = roslibpy.Service(self._ros, ROSAPI_PUBLISHERS_SERVICE,
                                   ROSAPI_PUBLISHERS_TYPE)
        response = service.call(roslibpy.ServiceRequest({"topic": topic}),
                                timeout=timeout_s)
        return list(response["publishers"])
```

- [ ] **Step 2: Wire `ai/person_following/__main__.py`**

Add to imports (after the `ai.common.connect` import):

```python
from ai.common.heal import heal, host_from_url
```

Add to `RosSink` (after `send_status`):

```python
    def publishers_of(self, topic, timeout_s=3.0):
        return self._client.publishers_of(topic, timeout_s=timeout_s)
```

Change the connect call in `main()` from:

```python
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src, config.GIMBAL_TILT)
```

to:

```python
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src,
                    config.GIMBAL_TILT,
                    heal=lambda: heal(host_from_url(profile["rosbridge_url"])))
```

- [ ] **Step 3: Wire `ai/face_tracking/__main__.py` the same way**

Same import line. Add to its `RosSink` (after `send`):

```python
    def publishers_of(self, topic, timeout_s=3.0):
        return self._client.publishers_of(topic, timeout_s=timeout_s)
```

Change its connect call from:

```python
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src, tilt_cfg)
```

to:

```python
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src, tilt_cfg,
                    heal=lambda: heal(host_from_url(profile["rosbridge_url"])))
```

- [ ] **Step 4: Verify**

Run: `python -m pytest ai/tests -q` → all pass
Run: `python -c "import ai.person_following.__main__, ai.face_tracking.__main__"` → no import errors

- [ ] **Step 5: Commit**

```bash
git add ai/common/ros_client.py ai/person_following/__main__.py ai/face_tracking/__main__.py
git commit -m "Runners: rosapi publishers_of + auto-heal wired into the preflight"
```

---

### Task 5: Manual CLI (`tools/heal_rosbridge.py`)

**Files:**
- Create: `tools/heal_rosbridge.py`

Thin wrapper, no unit tests (one subprocess call already covered; the CLI is verified live in Task 9).

- [ ] **Step 1: Create the file**

```python
"""Manually restart the robot's rosbridge stack from the laptop.

The cure for the Melodic rosbridge silent-drop registration bug (and the
wedged /voltage publisher after a power cycle): restart
rosbridge-dashboard.service over SSH. The runner preflight and the
dashboard's RESTART ROSBRIDGE button trigger the same heal automatically;
this CLI is the by-hand path.

  python tools/heal_rosbridge.py                 # home profile
  python tools/heal_rosbridge.py --profile hotspot

Drops the camera and every client connection for ~15-30 s — only run it
with the robot stationary.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ai import config  # noqa: E402
from ai.common.heal import heal, host_from_url  # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", choices=sorted(config.PROFILES),
                   default=config.DEFAULT_PROFILE)
    args = p.parse_args(argv)
    host = host_from_url(config.PROFILES[args.profile]["rosbridge_url"])
    print(f"restarting rosbridge-dashboard.service on {host} ...")
    ok, detail = heal(host)
    print(detail)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Smoke-test offline**

Run: `python tools/heal_rosbridge.py --help`
Expected: usage text, exit 0. (A real run needs the robot — Task 9.)

- [ ] **Step 3: Commit**

```bash
git add tools/heal_rosbridge.py
git commit -m "CLI: manual rosbridge heal (tools/heal_rosbridge.py)"
```

---

### Task 6: `POST /api/heal` in `serve_dashboard.py`

**Files:**
- Modify: `tools/serve_dashboard.py` (docstring, imports, `do_POST`, new `_heal`)

- [ ] **Step 1: Add imports**

After the `RUNNER_LOG = ...` line (REPO is defined just above):

```python
sys.path.insert(0, REPO)
from ai import config as ai_config  # noqa: E402
from ai.common.heal import heal, host_from_url  # noqa: E402
```

Add to the API summary in the module docstring (after the `/api/behavior/stop` line):

```
  POST /api/heal            -> body {} or {"profile": "home"}; SSH-restarts
                               rosbridge-dashboard.service on the robot and
                               waits for it (409 while a behavior is running)
```

- [ ] **Step 2: Rework `do_POST` and add `_heal`**

Replace the whole `do_POST` method with:

```python
    def do_POST(self):
        if not self.path.startswith("/api/"):
            return self._send_json({"error": "not found"}, 404)
        if not self._api_allowed():
            return self._send_json({"error": "localhost only"}, 403)
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            return self._send_json({"error": "bad json"}, 400)

        if self.path == "/api/behavior/start":
            ok, err = RUNNER.start(body.get("target_class"), body.get("cap_scale"),
                                   preview=bool(body.get("preview")))
        elif self.path == "/api/behavior/stop":
            ok, err = RUNNER.stop()
        elif self.path == "/api/heal":
            return self._heal(body)
        else:
            return self._send_json({"error": "not found"}, 404)
        if not ok:
            return self._send_json({"error": err}, 409)
        return self._send_json(RUNNER.status())

    def _heal(self, body):
        # A heal restarts the WHOLE robot stack — never yank it out from
        # under a live behavior (the runner has its own preflight heal).
        # A START racing in during the ~30 s heal is accepted: the runner's
        # own preflight ladder absorbs a mid-restart connect.
        if RUNNER.status()["running"]:
            return self._send_json({"error": "stop the runner first"}, 409)
        profile = body.get("profile") or ai_config.DEFAULT_PROFILE
        if profile not in ai_config.PROFILES:
            return self._send_json({"error": "unknown profile"}, 400)
        host = host_from_url(ai_config.PROFILES[profile]["rosbridge_url"])
        print(f"heal requested: restarting the robot stack on {host} ...")
        t0 = time.time()
        ok, detail = heal(host)
        print(f"heal {'succeeded' if ok else 'FAILED'}: {detail}")
        return self._send_json(
            {"ok": ok, "detail": detail, "seconds": int(time.time() - t0)},
            200 if ok else 502)
```

- [ ] **Step 3: Verify offline (no robot needed)**

Run: `python tools/serve_dashboard.py 8000` in the background, then:

```powershell
Invoke-RestMethod http://localhost:8000/api/behavior                      # {running: false, ...}
Invoke-WebRequest http://localhost:8000/api/heal -Method POST -Body '{"profile":"nope"}'
```

Expected: the second returns HTTP 400 `{"error": "unknown profile"}`. A heal against the real `home` profile with no robot present returns 502 with an ssh error in `detail` after ~5-10 s (BatchMode + ConnectTimeout fail fast) — acceptable to verify if curious. Stop the server.

- [ ] **Step 4: Commit**

```bash
git add tools/serve_dashboard.py
git commit -m "Behavior API: POST /api/heal (refused while a behavior is running)"
```

---

### Task 7: Dashboard RESTART ROSBRIDGE button

**Files:**
- Modify: `dashboard/index.html:45` (button after the chip)
- Modify: `dashboard/js/app.js:10,88-107` (import + `initActuationWarning`)
- Modify: `dashboard/css/style.css` (rule next to `.warning`, ~line 244)

- [ ] **Step 1: index.html**

After `<span id="actuation-warning" class="warning hidden">CMD FAULT</span>` add:

```html
      <button id="actuation-heal-btn" class="warning-btn hidden"
              title="restart rosbridge-dashboard.service on the robot — drops camera + link ~20 s">RESTART ROSBRIDGE</button>
```

- [ ] **Step 2: style.css**

Next to the `.warning` rule (`/* danger pills */` block):

```css
/* one-click cure for CMD FAULT — only shown when serve_dashboard.py hosts us */
.warning-btn {
  border-radius: 99px;
  border: 1px solid color-mix(in srgb, var(--c-danger) 35%, transparent);
  background: color-mix(in srgb, var(--c-danger) 12%, transparent);
  color: var(--c-danger-bright);
  font-family: var(--display);
  font-size: 10px;
  letter-spacing: 1px;
  padding: 3px 10px;
  cursor: pointer;
}
.warning-btn:disabled { opacity: 0.6; cursor: wait; }
.status-card .warning-btn { margin-top: 6px; }
```

- [ ] **Step 3: app.js**

Add `kickConnection` to the ros.js import:

```js
import { connect, onStatus, kickConnection } from './ros.js';
```

Replace `initActuationWarning()` with:

```js
function initActuationWarning() {
  const chip = $('actuation-warning');
  const healBtn = $('actuation-heal-btn');
  const panelFor = {
    [TOPICS.cmdVel.name]: 'drive-panel',
    [TOPICS.pwmServo.name]: 'gimbal-panel',
    [TOPICS.targetAngle.name]: 'arm-panel',
  };

  // The heal needs serve_dashboard.py (a browser can't SSH); when the
  // behavior API isn't there the chip's manual instructions still stand.
  async function healApiUp() {
    try { return (await fetch('/api/behavior')).ok; } catch { return false; }
  }

  onActuationCheck(async ({ state, dead = [] }) => {
    const fault = state === 'fault';
    chip.classList.toggle('hidden', !fault);
    for (const [topic, panelId] of Object.entries(panelFor)) {
      $(panelId).classList.toggle('alert', fault && dead.includes(topic));
    }
    if (fault) {
      chip.title = `rosbridge dropped publisher registration for ${dead.join(', ')}`
        + ' — commands on these topics are being silently discarded.'
        + ' Restart rosbridge-dashboard.service on the robot.';
    }
    healBtn.classList.toggle('hidden', !(fault && await healApiUp()));
  });

  healBtn.addEventListener('click', async () => {
    healBtn.disabled = true;
    healBtn.textContent = 'RESTARTING…';
    let fail = null;
    try {
      const resp = await fetch('/api/heal', {
        method: 'POST',
        body: JSON.stringify({ profile: $('profile-select').value }),
      });
      const out = await resp.json().catch(() => ({}));
      if (resp.ok) kickConnection(); // reconnect re-runs the self-check
      else fail = out.error ?? out.detail ?? `heal failed (${resp.status})`;
    } catch {
      fail = 'behavior API unreachable';
    }
    if (fail) {
      healBtn.textContent = 'HEAL FAILED';
      healBtn.title = fail;
      setTimeout(() => { healBtn.textContent = 'RESTART ROSBRIDGE'; }, 4000);
    } else {
      healBtn.textContent = 'RESTART ROSBRIDGE';
    }
    healBtn.disabled = false;
    healBtn.blur(); // keep focus free for driving keys
  });
}
```

Notes for the implementer: the spec suggested reusing the AI panel's API-reachability knowledge; this plan deliberately probes `/api/behavior` directly instead — a fault can fire before the panel's first poll completes, and a one-off fetch on the (rare) fault event has no init-order race. The `mock` profile isn't in the server's `ai_config.PROFILES`, so a heal click on mock returns 400 → shown as HEAL FAILED; harmless, and mock never produces a real fault (its check reports `unverified`). During a real heal the link drops and auto-reconnect (max 8 s backoff) brings the session back on its own; the post-fetch `kickConnection()` is belt and braces.

- [ ] **Step 4: Verify against the mock**

Start `python mock/mock_rosbridge.py` and `python tools/serve_dashboard.py`, open `http://localhost:8000`, mock profile:
- No CMD FAULT chip, no button (check reports `unverified` — see console).
- DevTools console: `document.getElementById('actuation-heal-btn').classList` contains `hidden`.

- [ ] **Step 5: Commit**

```bash
git add dashboard/index.html dashboard/js/app.js dashboard/css/style.css
git commit -m "Dashboard: RESTART ROSBRIDGE button on CMD FAULT (via /api/heal)"
```

---

### Task 8: Target-class picker

**Files:**
- Modify: `dashboard/js/config.js` (new `AI` export, after the `ACTUATION` block)
- Modify: `dashboard/js/ai_panel.js:18,47-68,110-125` (import, build select, START body)
- Modify: `dashboard/css/style.css` (select rule, next to `.warning-btn` additions or the panel-button block)

- [ ] **Step 1: config.js**

```js
// ---- AI behaviors -----------------------------------------------------------
// COCO classes offered by the AI panel's target picker. Values must satisfy
// serve_dashboard.py's TARGET_CLASS_RE (lowercase letters/spaces only).
// Applies at the next runner START, like the preview toggle.
export const AI = {
  targetClasses: ['person', 'dog', 'cat'],
};
```

- [ ] **Step 2: ai_panel.js**

Import: change the config import line to

```js
import { TOPICS, TELEMETRY, AI } from './config.js';
```

In `buildRunnerControls()`, before `panel.appendChild(row)`, add:

```js
  const classRow = document.createElement('div');
  classRow.className = 'kv';
  const classLabel = document.createElement('span');
  classLabel.textContent = 'TARGET';
  const classSel = document.createElement('select');
  classSel.id = 'ai-class-select';
  classSel.title = 'COCO class the runner follows — applies at the next START';
  for (const c of AI.targetClasses) {
    const opt = document.createElement('option');
    opt.value = c;
    opt.textContent = c.toUpperCase();
    classSel.appendChild(opt);
  }
  classSel.addEventListener('change', () => classSel.blur()); // keep driving keys free
  classRow.appendChild(classLabel);
  classRow.appendChild(classSel);
  panel.appendChild(classRow);
```

(Insert it after the preview button append so the panel reads: START, PREVIEW, TARGET, RUNNER state.)

In the start branch of the runner-button click handler, change:

```js
      await runnerRequest('/start', { preview: previewWanted });
```

to:

```js
      await runnerRequest('/start', {
        preview: previewWanted,
        target_class: $('ai-class-select').value,
      });
```

- [ ] **Step 3: style.css**

```css
/* AI panel target-class picker */
#ai-class-select {
  background: var(--c-card2);
  color: var(--c-ink-bright);
  border: 1px solid var(--c-line2);
  border-radius: 8px;
  font-size: 11px;
  padding: 2px 6px;
}
```

- [ ] **Step 4: Verify against the mock**

With `mock/mock_rosbridge.py` + `serve_dashboard.py` running, on `http://localhost:8000`:
- AI panel shows the TARGET select with PERSON/DOG/CAT.
- Pick DOG, click START RUNNER, then check the server console line: `behavior started: pid ...  -m ai.person_following --no-preview --target-class dog` (the runner itself exits quickly — the mock has no video stream; that's expected). Also confirm `runner.log` first line: `video source: ...  target class: dog`. Click STOP if still running.
- BEHAVIOR readout renders `state target_class fps` from `/ai/status` (existing code; nothing to change).

- [ ] **Step 5: Commit**

```bash
git add dashboard/js/config.js dashboard/js/ai_panel.js dashboard/css/style.css
git commit -m "AI panel: target-class picker (person/dog/cat), applied at next START"
```

---

### Task 9: Docs + final verification

**Files:**
- Modify: `docs/AI_NOTEBOOK.md` (new section), `README.md` (AI / dashboard feature mentions)

- [ ] **Step 1: docs/AI_NOTEBOOK.md**

Append a dated section (match the notebook's existing tone; remember `tools/md_to_docx.py` takes no bold/backticks/code-fences if a .docx is regenerated — headings, tables, bullets only in any converted copy):

```markdown
## Auto-heal for the rosbridge registration bug (2026-06-12)

The per-topic silent-drop bug now cures itself. Three layers, one shared
cure (tools-side SSH restart of rosbridge-dashboard.service, passwordless
sudo -n, then wait for the rosbridge port):

- Runner preflight (ai/common/connect.py): every fresh session is checked
  in two phases — rosapi publisher registration for /PWMServo, /ai/cmd_vel
  and /ai/status (the check that would have caught both 2026-06-11
  incidents), then the tilt-wiggle end-to-end proof. Escalation: 2 extra
  fresh sessions, then ONE heal per run, then one final session, then
  abort naming the dead topics. rosapi silent (mock server) = unverified,
  wiggle decides alone. The heal restarts the whole robot stack, so the
  video source is reopened afterwards (VideoSource.reopen retries up to
  60 s while the camera comes back).
- Dashboard: CMD FAULT now carries a RESTART ROSBRIDGE button (only when
  served by tools/serve_dashboard.py — a browser cannot SSH). POST
  /api/heal refuses (409) while a behavior process is running.
- By hand: python tools/heal_rosbridge.py [--profile hotspot]. Also clears
  the wedged /voltage publisher seen after power cycles.

Blast radius of any heal: camera + all connections drop ~15-30 s (the
service runs bringup + usb_cam + web_video_server + rosbridge). All paths
fire only with the robot stationary: the preflight runs before arming is
possible, the button is operator-clicked and runner-gated.

Also new: the AI panel has a TARGET picker (person/dog/cat, config.js
AI.targetClasses), passed as --target-class at the next START. Dog-mode
caveat: height_setpoint 0.8 was tuned on a standing person, so the follow
distance reads closer on a dog — live dog tuning is its own session.
```

- [ ] **Step 2: README.md**

Find the dashboard/AI feature list (the section updated in commit `1eb48ee`) and add two bullets in the matching style: one for the CMD FAULT RESTART ROSBRIDGE button + `/api/heal` + `tools/heal_rosbridge.py`, one for the TARGET picker and the runner preflight auto-heal.

- [ ] **Step 3: Full suite + mock selftest**

Run: `python -m pytest ai/tests -q` → all pass
Run: `python mock/selftest.py` → 5/5 (unchanged — confirms no dashboard regression at the rosbridge-protocol level)

- [ ] **Step 4: Commit**

```bash
git add docs/AI_NOTEBOOK.md README.md
git commit -m "Docs: rosbridge auto-heal layers + target-class picker"
```

---

### Task 10: Live validation (operator, robot powered on)

No code. Checklist for the next robot session — record outcomes in docs/AI_NOTEBOOK.md:

- [ ] `python tools/heal_rosbridge.py` with the robot healthy: prints restart + "port is back" within ~30 s; dashboard reconnects by itself; gimbal/drive/arm work after.
- [ ] `START RUNNER` from the panel: runner.log shows the registration check passing (or healing) before the wiggle check; behavior runs normally.
- [ ] If/when CMD FAULT appears naturally (it likes boots): click RESTART ROSBRIDGE, confirm the chip clears after reconnect without touching a terminal.
- [ ] Dog mode smoke test: TARGET=DOG, START, confirm `/ai/status` shows `dog` on the BEHAVIOR readout (live dog *tuning* remains out of scope).
