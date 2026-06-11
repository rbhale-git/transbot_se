# Person Following (AI Stage 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Robot base follows a person (or dog) using YOLO detection, with a robot-side command-priority mux and dashboard AI arm/disarm panel as safety prerequisites.

**Architecture:** Three components built in safety-first order: (1) `robot/cmd_vel_mux.py` — sole publisher to `/cmd_vel`, manual-wins arbitration with AI speed caps, deployed like the existing watchdog; (2) dashboard — drive moves to `/manual/cmd_vel`, new AI panel arms/disarms via `/ai/enabled`; (3) `ai/person_following/` — laptop-side YOLO nano detector + lock-at-arm IoU tracker + P-controller publishing `/ai/cmd_vel` at 10 Hz.

**Tech Stack:** Python 3.13 laptop-side (OpenCV DNN, roslibpy, pytest), Python 2 robot-side (rospy/Melodic), vanilla JS dashboard, YOLO11n ONNX.

**Spec:** `docs/superpowers/specs/2026-06-10-person-following-design.md`

---

## Execution status (updated 2026-06-11)

- Tasks 1-9: DONE (commits ba01849..ef998bc, subagent-implemented, two-stage reviewed, final integration review GO).
- Task 10 (offline validation): DONE — lock/stickiness/lost-stop verified on a recorded walk clip; caps and reverse limit confirmed in command stats.
- Task 11 (deploy + mux verify): DONE — mux live on the robot, manual-through-mux drive, e-stop, arm semantics, and watchdog regression all pass.
- Task 12 (live follow): DONE FOR INDOOR SCOPE — half caps, bedroom only. Live findings: angular sign +1 confirmed correct (first "turns away" was the latency limit cycle); kp_ang 0.6 + deadband_x 0.10 settle reliably; height_setpoint 0.8 ≈ 1 m follow distance; per-client 720p encoding saturates the Nano so both dashboard and AI now use downscaled streams (dashboard got STREAM on/off + SD/HD controls). Rosbridge half-registration bug bit twice more — it is PER TOPIC (a session can pass the gimbal actuation check while /ai/cmd_vel-/ai/enabled registrations are dead); verify mux armed flips + /ai/cmd_vel flows robot-side before trusting a session.
- **Task 12b (PENDING): open-space validation.** Bedroom testing left untested: full caps (`--cap-scale 1.0`), sustained straight-line following at walking pace, wide turns, longer lost-reacquire distances, and the reverse time limit under a real walk-into-robot. Run the Task 12 checklist in a larger room / cleared garage / outdoors-flat before calling stage 2 operator-accepted.
- Task 13 (docs): notebook + README written 2026-06-11; amend after Task 12b.

---

## File structure

| File | Action | Responsibility |
| --- | --- | --- |
| `robot/cmd_vel_mux.py` | Create | `MuxCore` (pure arbitration logic, py2/3) + rospy plumbing |
| `robot/transbot_dashboard.launch` | Modify | add mux node |
| `tools/deploy_robot.ps1` | Modify | also scp watchdog + mux into `transbot_bringup/scripts/` |
| `ai/tests/test_mux.py` | Create | MuxCore unit tests (loads module by path) |
| `dashboard/js/config.js` | Modify | drive topic → `/manual/cmd_vel`; add AI topics |
| `dashboard/js/ai_panel.js` | Create | ARM/DISARM toggle + mux/behavior status readout |
| `dashboard/index.html` | Modify | AI panel section |
| `dashboard/js/app.js` | Modify | import + init AI panel |
| `mock/mock_rosbridge.py` | Modify | `/manual/cmd_vel`, `/ai/enabled`, simulated `/mux/status` |
| `ai/common/detection.py` | Create | shared `Detection` dataclass + `select_largest` (moved from face detector) |
| `ai/face_tracking/detector.py` | Modify | import Detection from common (re-export for compat) |
| `ai/common/ros_client.py` | Modify | add `/ai/cmd_vel`, `/ai/status` publishers, `/ai/enabled` subscriber |
| `ai/common/connect.py` | Create | shared `connect_with_actuation_check` (moved from face `__main__`) |
| `ai/face_tracking/__main__.py` | Modify | use shared connect helper |
| `ai/models/yolo11n.onnx` | Create | exported YOLO11n model (~10 MB, committed like the YuNet model) |
| `ai/person_following/__init__.py` | Create | package marker |
| `ai/person_following/detector.py` | Create | `YoloDetector` (letterbox, decode, NMS, COCO labels) |
| `ai/person_following/tracker.py` | Create | `TargetTracker` lock state machine (IoU association) |
| `ai/person_following/controller.py` | Create | `FollowController` (P-control, caps, reverse time limit) |
| `ai/person_following/benchmark.py` | Create | detector FPS benchmark |
| `ai/person_following/__main__.py` | Create | runner (args, sinks, control loop, preview) |
| `ai/config.py` | Modify | add `FOLLOW` config dict |
| `ai/tests/test_yolo_detector.py` | Create | detector tests on real images |
| `ai/tests/test_target_tracker.py` | Create | tracker state machine tests |
| `ai/tests/test_follow_controller.py` | Create | controller tests |
| `ai/tests/test_ros_client.py` | Modify | add `twist_message` test |
| `ai/tests/data/people.jpg`, `ai/tests/data/dog.jpg` | Create | downloaded test stills |

Conventions to follow (from existing code): config-driven values, pure logic separated from I/O, injectable clocks for tests, module docstrings explaining *why*.

---

### Task 1: Mux arbitration core (TDD)

**Files:**
- Create: `robot/cmd_vel_mux.py` (core class only in this task)
- Create: `ai/tests/test_mux.py`

- [ ] **Step 1: Write the failing tests**

`ai/tests/test_mux.py`:

```python
"""MuxCore arbitration tests.

cmd_vel_mux.py runs on the robot under Python 2 / rospy, so its pure
arbitration core is loaded here by file path (robot/ is not a package) and
driven with explicit `now` timestamps - no rospy, no clocks.
"""

import importlib.util
from pathlib import Path

_PATH = Path(__file__).resolve().parents[2] / "robot" / "cmd_vel_mux.py"
_spec = importlib.util.spec_from_file_location("cmd_vel_mux", _PATH)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
MuxCore = _mod.MuxCore


def make():
    return MuxCore()  # defaults: window 1.0s, ai timeout 0.5s, caps 0.25/0.12/1.2


def test_manual_always_forwarded_unchanged():
    core = make()
    assert core.on_manual(0.45, -2.0, now=0.0) == (0.45, -2.0)
    core.armed = True  # arming must not affect manual
    assert core.on_manual(-0.3, 1.0, now=0.1) == (-0.3, 1.0)


def test_ai_blocked_when_disarmed():
    core = make()
    assert core.on_ai(0.1, 0.0, now=0.0) is None


def test_ai_blocked_during_manual_window():
    core = make()
    core.set_armed(True, now=0.0)
    core.on_manual(0.2, 0.0, now=1.0)
    assert core.on_ai(0.1, 0.0, now=1.5) is None      # 0.5s after manual: blocked
    assert core.on_ai(0.1, 0.0, now=2.1) == (0.1, 0.0)  # window expired: forwarded


def test_ai_clamped_to_caps():
    core = make()
    core.set_armed(True, now=0.0)
    assert core.on_ai(1.0, 5.0, now=0.0) == (0.25, 1.2)
    assert core.on_ai(-1.0, -5.0, now=0.1) == (-0.12, -1.2)


def test_disarm_zeroes_only_if_ai_was_driving():
    core = make()
    core.set_armed(True, now=0.0)
    assert core.set_armed(False, now=0.1) is False    # AI never drove: no zero
    core.set_armed(True, now=0.2)
    core.on_ai(0.1, 0.0, now=0.3)
    assert core.set_armed(False, now=0.4) is True     # AI was driving: halt
    assert core.on_ai(0.1, 0.0, now=0.5) is None      # and now disarmed


def test_manual_takeover_clears_ai_source():
    core = make()
    core.set_armed(True, now=0.0)
    core.on_ai(0.1, 0.0, now=0.0)
    assert core.status(now=0.1)["source"] == "ai"
    core.on_manual(0.2, 0.0, now=0.2)
    assert core.status(now=0.3)["source"] == "manual"
    # disarm right after a manual takeover must not zero (manual is driving)
    assert core.set_armed(False, now=0.4) is False


def test_ai_silence_timeout_fires_once():
    core = make()
    core.set_armed(True, now=0.0)
    core.on_ai(0.1, 0.0, now=0.0)
    assert core.check_timeout(now=0.3) is False   # still fresh
    assert core.check_timeout(now=0.6) is True    # >0.5s silent: zero once
    assert core.check_timeout(now=0.7) is False   # fired already


def test_status_shape():
    core = make()
    s = core.status(now=0.0)
    assert s == {
        "source": "none",
        "armed": False,
        "caps": {"fwd": 0.25, "rev": 0.12, "ang": 1.2},
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ai/tests/test_mux.py -v`
Expected: FAIL — `FileNotFoundError` or `AttributeError` (module/class doesn't exist yet)

- [ ] **Step 3: Write the implementation**

`robot/cmd_vel_mux.py`:

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""cmd_vel command-priority mux - manual always wins, AI is capped and gated.

Stage 2 of the AI roadmap puts an autonomous publisher on the chassis for the
first time. ROS does not arbitrate between publishers, so this node becomes
the ONLY publisher to /cmd_vel and arbitrates its two inputs:

  /manual/cmd_vel  dashboard (keyboard/gamepad/e-stop). Always forwarded;
                   suppresses AI for MANUAL_WINDOW_S after the last message,
                   so touching the joystick instantly takes over.
  /ai/cmd_vel      AI behaviors. Forwarded only while /ai/enabled (latched
                   Bool from the dashboard panel) is true and manual is
                   quiet, then clamped to the AI caps below - a buggy AI
                   process cannot drive fast, ever.

On disarm while AI was driving, and on AI silence > AI_TIMEOUT_S while AI
was driving, one zero Twist is published (halt, don't coast). /mux/status
(JSON String, 2 Hz) tells the dashboard who is driving.

The existing cmd_vel_watchdog is unchanged and remains the last line of
defense on /cmd_vel itself.

Runs on the robot under transbot_dashboard.launch (ROS Melodic, py2). The
MuxCore class is pure logic, py2/3 compatible, unit-tested on the laptop
(ai/tests/test_mux.py) before deployment.
"""

import json
import threading

MANUAL_WINDOW_S = 1.0   # manual suppresses AI for this long after last msg
AI_TIMEOUT_S = 0.5      # AI silence while driving => publish one zero
AI_CAP_FWD = 0.25       # m/s   (manual cap is 0.45 dashboard-side)
AI_CAP_REV = 0.12       # m/s   (reverse is blind - tighter cap)
AI_CAP_ANG = 1.2        # rad/s (manual cap is 2.0)
STATUS_HZ = 2.0
CHECK_HZ = 10.0


class MuxCore(object):
    """Pure arbitration logic. All methods take explicit `now` seconds."""

    def __init__(self, manual_window_s=MANUAL_WINDOW_S, ai_timeout_s=AI_TIMEOUT_S,
                 ai_cap_fwd=AI_CAP_FWD, ai_cap_rev=AI_CAP_REV, ai_cap_ang=AI_CAP_ANG):
        self.manual_window_s = manual_window_s
        self.ai_timeout_s = ai_timeout_s
        self.ai_cap_fwd = ai_cap_fwd
        self.ai_cap_rev = ai_cap_rev
        self.ai_cap_ang = ai_cap_ang
        self.armed = False
        self._last_manual = None
        self._last_ai = None
        self._ai_active = False   # AI is the current source of /cmd_vel

    def _manual_recent(self, now):
        return (self._last_manual is not None
                and (now - self._last_manual) < self.manual_window_s)

    def on_manual(self, lin, ang, now):
        """Manual input: always forwarded, becomes the active source."""
        self._last_manual = now
        self._ai_active = False
        return (lin, ang)

    def on_ai(self, lin, ang, now):
        """AI input: (lin, ang) clamped to caps, or None if blocked."""
        if not self.armed or self._manual_recent(now):
            return None
        self._last_ai = now
        self._ai_active = True
        lin = min(self.ai_cap_fwd, max(-self.ai_cap_rev, lin))
        ang = min(self.ai_cap_ang, max(-self.ai_cap_ang, ang))
        return (lin, ang)

    def set_armed(self, armed, now):
        """Returns True if a zero Twist must be published (halt AI motion)."""
        was_active = self._ai_active
        self.armed = bool(armed)
        if not self.armed and was_active:
            self._ai_active = False
            return True
        return False

    def check_timeout(self, now):
        """True once when AI goes silent while it was driving."""
        if (self._ai_active and self._last_ai is not None
                and (now - self._last_ai) > self.ai_timeout_s):
            self._ai_active = False
            return True
        return False

    def status(self, now):
        if self._manual_recent(now):
            source = "manual"
        elif self._ai_active:
            source = "ai"
        else:
            source = "none"
        return {"source": source, "armed": self.armed,
                "caps": {"fwd": self.ai_cap_fwd, "rev": self.ai_cap_rev,
                         "ang": self.ai_cap_ang}}


def run_node():
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String

    rospy.init_node("cmd_vel_mux")
    core = MuxCore()
    lock = threading.Lock()   # rospy callbacks arrive on separate threads
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    status_pub = rospy.Publisher("/mux/status", String, queue_size=1)

    def now_s():
        return rospy.Time.now().to_sec()

    def on_manual(msg):
        with lock:
            core.on_manual(msg.linear.x, msg.angular.z, now_s())
        pub.publish(msg)   # forwarded verbatim - manual is never modified

    def on_ai(msg):
        with lock:
            out = core.on_ai(msg.linear.x, msg.angular.z, now_s())
        if out is None:
            return
        fwd = Twist()
        fwd.linear.x, fwd.angular.z = out
        pub.publish(fwd)

    def on_enabled(msg):
        with lock:
            halt = core.set_armed(msg.data, now_s())
        rospy.loginfo("ai %s", "ARMED" if msg.data else "disarmed")
        if halt:
            pub.publish(Twist())   # all zeros

    def on_check(_event):
        with lock:
            halt = core.check_timeout(now_s())
        if halt:
            rospy.logwarn("ai input went silent while driving - stopping")
            pub.publish(Twist())

    def on_status(_event):
        with lock:
            s = core.status(now_s())
        status_pub.publish(String(json.dumps(s)))

    rospy.Subscriber("/manual/cmd_vel", Twist, on_manual, queue_size=5)
    rospy.Subscriber("/ai/cmd_vel", Twist, on_ai, queue_size=5)
    rospy.Subscriber("/ai/enabled", Bool, on_enabled, queue_size=5)
    rospy.Timer(rospy.Duration(1.0 / CHECK_HZ), on_check)
    rospy.Timer(rospy.Duration(1.0 / STATUS_HZ), on_status)
    rospy.loginfo("cmd_vel mux up: manual window %.1fs, ai caps %.2f/%.2f/%.1f",
                  core.manual_window_s, core.ai_cap_fwd, core.ai_cap_rev,
                  core.ai_cap_ang)
    rospy.spin()


if __name__ == "__main__":
    run_node()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ai/tests/test_mux.py -v`
Expected: 8 passed

- [ ] **Step 5: Run the full suite (no regressions)**

Run: `python -m pytest`
Expected: all pass (54 existing + 8 new)

- [ ] **Step 6: Commit**

```powershell
git add robot/cmd_vel_mux.py ai/tests/test_mux.py
git commit -m "Robot-side cmd_vel priority mux: manual wins, AI capped and gated"
```

---

### Task 2: Launch file + deploy script

**Files:**
- Modify: `robot/transbot_dashboard.launch` (after the watchdog node, line 54)
- Modify: `tools/deploy_robot.ps1`

- [ ] **Step 1: Add the mux node to the launch file**

In `robot/transbot_dashboard.launch`, after the `cmd_vel_watchdog` node element, add:

```xml
  <!-- 6. SAFETY: command-priority mux. Sole publisher to /cmd_vel: forwards
       /manual/cmd_vel (dashboard) always, /ai/cmd_vel only while /ai/enabled
       and manual is quiet, clamped to the AI speed caps. Deployed into
       transbot_bringup/scripts/ like the watchdog. -->
  <node name="cmd_vel_mux" pkg="transbot_bringup" type="cmd_vel_mux.py"
        output="screen" respawn="true" respawn_delay="5" />
```

- [ ] **Step 2: Extend the deploy script**

In `tools/deploy_robot.ps1`, after the existing scp block, add:

```powershell
Write-Host "Copying safety nodes into transbot_bringup/scripts..."
scp (Join-Path $repoRoot "robot\cmd_vel_watchdog.py") `
    (Join-Path $repoRoot "robot\cmd_vel_mux.py") `
    "${target}:~/transbot_ws/src/transbot_bringup/scripts/"
if ($LASTEXITCODE -ne 0) { Write-Host "scp failed" -ForegroundColor Red; exit 1 }
ssh $target "chmod +x ~/transbot_ws/src/transbot_bringup/scripts/cmd_vel_watchdog.py ~/transbot_ws/src/transbot_bringup/scripts/cmd_vel_mux.py"
if ($LASTEXITCODE -ne 0) { Write-Host "chmod failed" -ForegroundColor Red; exit 1 }
```

- [ ] **Step 3: Commit**

```powershell
git add robot/transbot_dashboard.launch tools/deploy_robot.ps1
git commit -m "Launch + deploy: cmd_vel_mux ships with the robot stack"
```

Note: actual deployment to the robot is Task 11 (needs the robot powered on). After Task 3 lands, the dashboard publishes `/manual/cmd_vel`, so a robot running the OLD stack will not respond to dashboard driving until Task 11 deploys the mux. Mock mode keeps working throughout.

---

### Task 3: Dashboard config + mock extension

**Files:**
- Modify: `dashboard/js/config.js`
- Modify: `mock/mock_rosbridge.py`

- [ ] **Step 1: Update config.js topics**

In `dashboard/js/config.js`, replace the `cmdVel` entry:

```js
  // Drive. Published to /manual/cmd_vel; the robot-side cmd_vel_mux forwards
  // it to /cmd_vel (manual always wins over AI). Requires the stage-2 robot
  // stack — an old robot image without the mux will not hear the dashboard.
  cmdVel: { name: '/manual/cmd_vel', type: 'geometry_msgs/Twist', verified: true },
```

and add to `TOPICS` (after `cameraInfo`):

```js
  // AI arbitration (stage 2). aiEnabled is latched robot-side by the mux.
  aiEnabled: { name: '/ai/enabled', type: 'std_msgs/Bool', verified: true },
  muxStatus: { name: '/mux/status', type: 'std_msgs/String', verified: true },
  aiStatus: { name: '/ai/status', type: 'std_msgs/String', verified: true },
```

- [ ] **Step 2: Update the mock**

In `mock/mock_rosbridge.py`:

(a) Add to `state` dict: `"ai_armed": False,`

(b) In `handle_publish`, change the `/cmd_vel` branch condition to accept both names, and add an `/ai/enabled` branch:

```python
    if topic in ("/cmd_vel", "/manual/cmd_vel"):
        lin = float(msg.get("linear", {}).get("x", 0.0))
        ang = float(msg.get("angular", {}).get("z", 0.0))
        state["last_cmd"] = {"linear": lin, "angular": ang, "at": time.monotonic()}
        log("RECV", f"{topic:<16} lin={lin:+.3f} m/s  ang={ang:+.3f} rad/s")
    elif topic == "/ai/enabled":
        state["ai_armed"] = bool(msg.get("data"))
        log("RECV", f"/ai/enabled     armed={state['ai_armed']}")
```

(c) Add a `/mux/status` generator (after `gen_camera_info`) and register it:

```python
def gen_mux_status() -> dict:
    # Mirrors robot/cmd_vel_mux.py status: who is driving + armed + AI caps.
    cmd = state["last_cmd"]
    fresh = (time.monotonic() - cmd["at"]) < 1.0
    payload = {
        "source": "manual" if fresh else "none",
        "armed": state["ai_armed"],
        "caps": {"fwd": 0.25, "rev": 0.12, "ang": 1.2},
    }
    return {"data": json.dumps(payload)}
```

In `TELEMETRY`, add: `"/mux/status": (gen_mux_status, 0.5),`

- [ ] **Step 3: Manual verification against the mock**

Run in two terminals: `python mock/mock_rosbridge.py` and `python tools/serve_dashboard.py`. Open http://localhost:8000 (MOCK profile). Hold W — the mock console must log `RECV /manual/cmd_vel ... lin=+0.200`. Release — zero burst logged.

Expected: drive commands arrive on `/manual/cmd_vel`; everything else unchanged.

- [ ] **Step 4: Commit**

```powershell
git add dashboard/js/config.js mock/mock_rosbridge.py
git commit -m "Dashboard drives /manual/cmd_vel; mock simulates mux status + arm state"
```

---

### Task 4: Dashboard AI panel

**Files:**
- Create: `dashboard/js/ai_panel.js`
- Modify: `dashboard/index.html` (after the settings panel section)
- Modify: `dashboard/js/app.js`

- [ ] **Step 1: Add the panel markup**

In `dashboard/index.html`, after the closing `</section>` of `settings-panel`, add:

```html
      <section id="ai-panel" class="panel" aria-label="ai">
        <div class="panel-title"><span class="num">05</span>AI</div>
        <button id="ai-arm-btn" class="panel-btn" title="allow AI behaviors to drive the chassis (robot-side mux enforces caps; joystick always overrides)">&#9655; ARM AI</button>
        <div class="kv"><span>DRIVE SRC</span><code id="ai-source" class="dim">--</code></div>
        <div class="kv"><span>AI ARMED</span><code id="ai-armed" class="dim">--</code></div>
        <div class="kv"><span>BEHAVIOR</span><code id="ai-behavior" class="dim small">--</code></div>
      </section>
```

- [ ] **Step 2: Write the panel module**

`dashboard/js/ai_panel.js`:

```js
// ============================================================================
// ai_panel.js — ARM/DISARM switch for AI chassis control + arbitration status.
//
// Publishes /ai/enabled (Bool). The robot-side cmd_vel_mux is the enforcement
// point — this panel is just the operator's switch and readout. Policy:
// disarmed on every page load and on every (re)connect; e-stop disarms.
// Status comes from /mux/status (who is driving) and /ai/status (behavior
// state), both JSON-in-String, marked stale after TELEMETRY.staleAfterMs.
// ============================================================================

import { TOPICS, TELEMETRY } from './config.js';
import { publish, subscribe, onStatus } from './ros.js';
import { onEStop } from './keyboard.js';

const $ = (id) => document.getElementById(id);

let armed = false;
let lastMux = 0;
let lastAi = 0;

function render() {
  const btn = $('ai-arm-btn');
  btn.innerHTML = armed ? '&#9632; DISARM AI' : '&#9655; ARM AI';
  btn.classList.toggle('armed', armed);
}

function setArmed(next) {
  armed = next;
  publish(TOPICS.aiEnabled, { data: armed });
  render();
}

export function initAiPanel() {
  $('ai-arm-btn').addEventListener('click', () => setArmed(!armed));
  onEStop(() => setArmed(false));   // e-stop also pulls the AI switch

  onStatus((status) => {
    if (status !== 'connected') return;
    // Fresh link: declare disarmed (page-load default) and (re)subscribe.
    setArmed(false);
    subscribe(TOPICS.muxStatus, (msg) => {
      lastMux = Date.now();
      try {
        const s = JSON.parse(msg.data);
        $('ai-source').textContent = s.source.toUpperCase();
        $('ai-armed').textContent = s.armed ? 'YES' : 'no';
      } catch { /* malformed status: leave last value, staleness will catch it */ }
    });
    subscribe(TOPICS.aiStatus, (msg) => {
      lastAi = Date.now();
      try {
        const s = JSON.parse(msg.data);
        $('ai-behavior').textContent =
          `${s.state} ${s.target_class ?? ''} ${s.fps != null ? s.fps + 'fps' : ''}`.trim();
      } catch { /* ignore */ }
    });
  });

  setInterval(() => {
    const now = Date.now();
    if (now - lastMux > TELEMETRY.staleAfterMs) {
      $('ai-source').textContent = '--';
      $('ai-armed').textContent = '--';
    }
    if (now - lastAi > TELEMETRY.staleAfterMs) {
      $('ai-behavior').textContent = '--';
    }
  }, 1000);

  render();
}
```

- [ ] **Step 3: Wire into app.js**

In `dashboard/js/app.js`: add import (with the other imports):

```js
import { initAiPanel } from './ai_panel.js';
```

and add `initAiPanel();` to the init call list, after `initSettingsPanel();`.

- [ ] **Step 4: Manual verification against the mock**

With `python mock/mock_rosbridge.py` + `python tools/serve_dashboard.py` running, on http://localhost:8000:
1. AI panel shows DRIVE SRC `NONE`, AI ARMED `no` (from simulated /mux/status).
2. Click ARM AI → mock logs `RECV /ai/enabled armed=True`; AI ARMED flips to `YES` within ~1 s.
3. Hold W → DRIVE SRC shows `MANUAL`; release → back to `NONE`.
4. Press SPACE (e-stop) → mock logs `armed=False`, button returns to ARM AI.
5. Stop the mock → readouts fall back to `--` within ~3 s.

- [ ] **Step 5: Commit**

```powershell
git add dashboard/index.html dashboard/js/ai_panel.js dashboard/js/app.js
git commit -m "Dashboard AI panel: arm/disarm switch + mux/behavior status readout"
```

---

### Task 5: Shared Detection + RosClient chassis support (TDD)

**Files:**
- Create: `ai/common/detection.py`
- Modify: `ai/face_tracking/detector.py`
- Modify: `ai/common/ros_client.py`
- Modify: `ai/tests/test_ros_client.py`

- [ ] **Step 1: Create the shared detection module**

`ai/common/detection.py`:

```python
"""Shared detection bbox type for every AI behavior.

Originally lived in ai/face_tracking/detector.py; promoted here when stage 2
added a second detector (YOLO). `label` is the class name for multi-class
detectors; single-class detectors (YuNet faces) leave it empty.
"""

from dataclasses import dataclass


@dataclass
class Detection:
    x: float
    y: float
    w: float
    h: float
    score: float
    label: str = ""

    @property
    def area(self):
        return self.w * self.h

    @property
    def center(self):
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)


def select_largest(detections):
    """The detection to act on: largest bbox (closest), or None."""
    if not detections:
        return None
    return max(detections, key=lambda d: d.area)
```

- [ ] **Step 2: Re-point the face detector at it**

In `ai/face_tracking/detector.py`, delete the `@dataclass class Detection` block and the `select_primary` function, and replace with:

```python
from ai.common.detection import Detection, select_largest

# Backwards-compatible alias: stage-1 code and tests use select_primary.
select_primary = select_largest
```

(Keep the `YuNetDetector` class unchanged; remove the now-unused `from dataclasses import dataclass` import.)

- [ ] **Step 3: Run existing tests — must still pass**

Run: `python -m pytest ai/tests/test_detector.py ai/tests/test_tracker.py -v`
Expected: all pass (imports resolve via the alias)

- [ ] **Step 4: Write the failing test for twist_message**

Add to `ai/tests/test_ros_client.py`:

```python
def test_twist_message_shape():
    from ai.common.ros_client import twist_message
    msg = twist_message(0.25, -1.2)
    assert msg == {
        "linear": {"x": 0.25, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": -1.2},
    }
```

Run: `python -m pytest ai/tests/test_ros_client.py -v`
Expected: FAIL — `ImportError: cannot import name 'twist_message'`

- [ ] **Step 5: Extend RosClient**

Replace `ai/common/ros_client.py` with:

```python
"""Thin roslibpy wrapper for AI behaviors.

Wire shapes were verified against the live robot (FINDINGS.md). This module
owns the websocket connection and topic advertising; behaviors only ever
hand it plain dicts built by the pure helpers below (which is what the unit
tests cover — the connection path is exercised live).

Stage 2 additions: /ai/cmd_vel (chassis, arbitrated by the robot-side mux),
/ai/status (behavior state for the dashboard), /ai/enabled subscription
(defense in depth — behaviors stop publishing motion when disarmed, even
though the mux already enforces it).
"""

import json
import time

import roslibpy

PWM_SERVO_TOPIC = "/PWMServo"
PWM_SERVO_TYPE = "transbot_msgs/PWMServo"
AI_CMD_VEL_TOPIC = "/ai/cmd_vel"
AI_CMD_VEL_TYPE = "geometry_msgs/Twist"
AI_STATUS_TOPIC = "/ai/status"
AI_STATUS_TYPE = "std_msgs/String"
AI_ENABLED_TOPIC = "/ai/enabled"
AI_ENABLED_TYPE = "std_msgs/Bool"

# Pause between advertising and first publish. The robot's old rosbridge
# (0.11/Melodic) can hit "Internal error processing topic" if its publisher
# registration races traffic — seen live 2026-06-10, where it silently
# dropped every command for that session (docs/AI_NOTEBOOK.md).
ADVERTISE_SETTLE_S = 1.0


def pwm_servo_message(servo_id, angle_deg):
    """Verified shape: transbot_msgs/PWMServo = {int32 id, int32 angle}."""
    return {"id": int(servo_id), "angle": int(round(angle_deg))}


def twist_message(linear_x, angular_z):
    """geometry_msgs/Twist dict; only the two driven fields are non-zero."""
    return {
        "linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
    }


class RosClient:
    """Connection + publishers for one AI behavior process."""

    def __init__(self, url):
        # roslibpy wants host/port split; accept the dashboard-style ws:// URL.
        host, port = self._parse(url)
        self._ros = roslibpy.Ros(host=host, port=port)
        self._pwm = roslibpy.Topic(self._ros, PWM_SERVO_TOPIC, PWM_SERVO_TYPE)
        self._twist = roslibpy.Topic(self._ros, AI_CMD_VEL_TOPIC, AI_CMD_VEL_TYPE)
        self._status = roslibpy.Topic(self._ros, AI_STATUS_TOPIC, AI_STATUS_TYPE)
        self._enabled = roslibpy.Topic(self._ros, AI_ENABLED_TOPIC, AI_ENABLED_TYPE)
        self._pubs = (self._pwm, self._twist, self._status)

    @staticmethod
    def _parse(url):
        stripped = url.replace("ws://", "").rstrip("/")
        host, _, port = stripped.partition(":")
        return host, int(port or 9090)

    @property
    def connected(self):
        return self._ros.is_connected

    def connect(self, timeout_s=10):
        self._ros.run(timeout=timeout_s)
        if not self._ros.is_connected:
            raise RuntimeError("rosbridge connection failed")
        # Advertise up front instead of lazily on first publish, then let the
        # registration settle before any traffic.
        for topic in self._pubs:
            topic.advertise()
        time.sleep(ADVERTISE_SETTLE_S)

    def send_pwm_servo(self, servo_id, angle_deg):
        self._pwm.publish(roslibpy.Message(pwm_servo_message(servo_id, angle_deg)))

    def send_twist(self, linear_x, angular_z):
        self._twist.publish(roslibpy.Message(twist_message(linear_x, angular_z)))

    def send_status(self, status_dict):
        self._status.publish(roslibpy.Message({"data": json.dumps(status_dict)}))

    def on_ai_enabled(self, callback):
        """callback(bool) fires on every /ai/enabled message (latched, so the
        current state arrives immediately after subscribing)."""
        self._enabled.subscribe(lambda msg: callback(bool(msg["data"])))

    def disconnect(self):
        """Close the websocket but keep the process's event loop running.

        roslibpy shares one Twisted reactor per process and a reactor can
        never be restarted — terminate() here would make every future
        RosClient in this process fail. Use this between reconnect attempts;
        close() only at process exit.
        """
        for topic in self._pubs:
            try:
                topic.unadvertise()
            except Exception:
                pass
        try:
            self._enabled.unsubscribe()
        except Exception:
            pass
        self._ros.close()

    def close(self):
        for topic in self._pubs:
            try:
                topic.unadvertise()
            except Exception:
                pass
        try:
            self._ros.terminate()
        except Exception:
            # roslibpy teardown is unreliable after a disconnect/reconnect
            # cycle (its event-loop manager may lack a _thread). Only ever
            # called at process exit, so swallowing this is safe.
            pass
```

- [ ] **Step 6: Run the suite**

Run: `python -m pytest`
Expected: all pass

- [ ] **Step 7: Commit**

```powershell
git add ai/common/detection.py ai/face_tracking/detector.py ai/common/ros_client.py ai/tests/test_ros_client.py
git commit -m "Shared Detection type; RosClient learns /ai/cmd_vel, /ai/status, /ai/enabled"
```

---

### Task 6: YOLO model + detector (TDD) + benchmark

**Files:**
- Create: `ai/models/yolo11n.onnx` (exported)
- Create: `ai/tests/data/people.jpg`, `ai/tests/data/dog.jpg` (downloaded)
- Create: `ai/person_following/__init__.py`, `ai/person_following/detector.py`, `ai/person_following/benchmark.py`
- Create: `ai/tests/test_yolo_detector.py`

- [ ] **Step 1: Export the model and fetch test images**

```powershell
pip install ultralytics
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='onnx', imgsz=640, opset=12)"
Move-Item yolo11n.onnx ai\models\yolo11n.onnx
Remove-Item yolo11n.pt
curl.exe -L -o ai\tests\data\people.jpg https://ultralytics.com/images/bus.jpg
curl.exe -L -o ai\tests\data\dog.jpg https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg
```

Notes: `ultralytics` is export-only tooling — do NOT add it to `ai/requirements.txt` (runtime is pure OpenCV DNN, consistent with YuNet). The ~10 MB ONNX is committed, same policy as the YuNet model. `people.jpg` (the Ultralytics bus scene) contains 4 people; `dog.jpg` is a Samoyed with no people.

- [ ] **Step 2: Write the failing tests**

`ai/tests/test_yolo_detector.py`:

```python
"""YoloDetector tests against real images (model + stills are in the repo)."""

from pathlib import Path

import cv2
import pytest

from ai.person_following.detector import YoloDetector, COCO_CLASSES

MODEL = Path(__file__).resolve().parents[1] / "models" / "yolo11n.onnx"
DATA = Path(__file__).parent / "data"


@pytest.fixture(scope="module")
def detector():
    return YoloDetector(str(MODEL))


def test_coco_has_the_classes_we_follow():
    assert "person" in COCO_CLASSES
    assert "dog" in COCO_CLASSES
    assert len(COCO_CLASSES) == 80


def test_detects_people(detector):
    img = cv2.imread(str(DATA / "people.jpg"))
    dets = [d for d in detector.detect(img) if d.label == "person"]
    assert len(dets) >= 2          # scene contains 4; require 2 to be robust
    h, w = img.shape[:2]
    for d in dets:
        assert d.score >= 0.5
        assert -5 <= d.x <= w and -5 <= d.y <= h   # boxes land on the image
        assert 0 < d.w <= w and 0 < d.h <= h


def test_detects_dog(detector):
    img = cv2.imread(str(DATA / "dog.jpg"))
    labels = [d.label for d in detector.detect(img)]
    assert "dog" in labels


def test_no_person_in_dog_image(detector):
    img = cv2.imread(str(DATA / "dog.jpg"))
    assert [d for d in detector.detect(img) if d.label == "person"] == []
```

Run: `python -m pytest ai/tests/test_yolo_detector.py -v`
Expected: FAIL — `ModuleNotFoundError: ai.person_following`

- [ ] **Step 3: Write the detector**

`ai/person_following/__init__.py`: empty file.

`ai/person_following/detector.py`:

```python
"""YOLO11n object detector via OpenCV DNN.

Chosen in the stage-2 design: nano-size (~10 MB ONNX), 80 COCO classes (so
the follow target is a config choice — person, dog, ...), runs on the laptop
CPU well above the 10 Hz control rate, and the same model serves stages 3/4
(find-and-approach for pick and place). Runtime is cv2.dnn only — same stack
as YuNet, no new dependencies; `ultralytics` was used once, offline, to
export the ONNX.

Decode notes (YOLOv8/11 ONNX, 640x640 input): output is (1, 84, 8400) —
4 box coords (cx,cy,w,h in input pixels) + 80 class scores per anchor, no
objectness term. Frames are letterboxed top-left (pad right/bottom with the
conventional gray 114) so mapping back to frame coords is a single divide.
"""

import cv2
import numpy as np

from ai.common.detection import Detection

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


class YoloDetector:
    def __init__(self, model_path, score_threshold=0.5, nms_threshold=0.45,
                 input_size=640):
        self._net = cv2.dnn.readNetFromONNX(str(model_path))
        self._score = score_threshold
        self._nms = nms_threshold
        self._size = input_size

    def detect(self, frame_bgr):
        """All detections above threshold, in frame coordinates."""
        h, w = frame_bgr.shape[:2]
        scale = min(self._size / w, self._size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        canvas = np.full((self._size, self._size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(frame_bgr, (nw, nh))
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0,
                                     (self._size, self._size),
                                     swapRB=True, crop=False)
        self._net.setInput(blob)
        out = self._net.forward()[0].T          # (8400, 84)
        class_ids = np.argmax(out[:, 4:], axis=1)
        scores = out[np.arange(len(out)), 4 + class_ids]
        keep = scores >= self._score
        out, class_ids, scores = out[keep], class_ids[keep], scores[keep]
        boxes = [[float(cx - bw / 2), float(cy - bh / 2), float(bw), float(bh)]
                 for cx, cy, bw, bh in out[:, :4]]
        idxs = cv2.dnn.NMSBoxes(boxes, [float(s) for s in scores],
                                self._score, self._nms)
        dets = []
        for i in np.asarray(idxs).flatten():
            x, y, bw, bh = boxes[i]
            dets.append(Detection(
                x=x / scale, y=y / scale, w=bw / scale, h=bh / scale,
                score=float(scores[i]), label=COCO_CLASSES[class_ids[i]]))
        return dets
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ai/tests/test_yolo_detector.py -v`
Expected: 4 passed

- [ ] **Step 5: Write and run the benchmark**

`ai/person_following/benchmark.py`:

```python
"""Detector throughput check — the design gate is >= 10 fps on the laptop.

Run: python -m ai.person_following.benchmark
"""

import time
from pathlib import Path

import cv2

from ai.person_following.detector import YoloDetector

MODEL = Path(__file__).resolve().parents[1] / "models" / "yolo11n.onnx"
IMAGE = Path(__file__).resolve().parents[1] / "tests" / "data" / "people.jpg"
RUNS = 30


def main():
    det = YoloDetector(str(MODEL))
    frame = cv2.imread(str(IMAGE))
    det.detect(frame)  # warm-up (first inference pays one-time init)
    t0 = time.perf_counter()
    for _ in range(RUNS):
        det.detect(frame)
    dt = (time.perf_counter() - t0) / RUNS
    print(f"{dt * 1000:.1f} ms/frame  ->  {1 / dt:.1f} fps")
    print("PASS (>= 10 fps)" if 1 / dt >= 10 else "FAIL: below the 10 fps design gate")


if __name__ == "__main__":
    main()
```

Run: `python -m ai.person_following.benchmark`
Expected: prints ms/frame and `PASS (>= 10 fps)`. **If FAIL:** retry with `input_size=416` in the constructor default and re-run tests + benchmark; record the chosen size in `ai/config.py` (Task 8 `FOLLOW["input_size"]`).

- [ ] **Step 6: Commit**

```powershell
git add ai/models/yolo11n.onnx ai/tests/data/people.jpg ai/tests/data/dog.jpg ai/person_following/ ai/tests/test_yolo_detector.py
git commit -m "YOLO11n detector via cv2.dnn: letterbox decode, COCO labels, benchmark"
```

---

### Task 7: Lock tracker (TDD)

**Files:**
- Create: `ai/person_following/tracker.py`
- Create: `ai/tests/test_target_tracker.py`

- [ ] **Step 1: Write the failing tests**

`ai/tests/test_target_tracker.py`:

```python
"""TargetTracker: lock-at-arm / stop-on-loss state machine (no re-ID)."""

from ai.common.detection import Detection
from ai.person_following.tracker import TargetTracker, iou


def box(x, y, w=100, h=200, label="person"):
    return Detection(x=x, y=y, w=w, h=h, score=0.9, label=label)


def test_iou_identical_and_disjoint():
    assert iou(box(0, 0), box(0, 0)) == 1.0
    assert iou(box(0, 0), box(500, 500)) == 0.0


def test_locks_largest_on_first_sighting():
    t = TargetTracker()
    small, big = box(0, 0, w=50, h=100), box(300, 0, w=120, h=240)
    assert t.update([small, big], now=0.0) is big
    assert t.state == "FOLLOWING"


def test_sticks_to_locked_target_not_largest():
    t = TargetTracker()
    locked = box(100, 100)
    t.update([locked], now=0.0)
    moved = box(120, 105)                       # same person, slight motion
    intruder = box(600, 100, w=200, h=400)      # bigger box, elsewhere
    assert t.update([intruder, moved], now=0.1) is moved


def test_no_match_returns_none_within_grace():
    t = TargetTracker(lost_grace_s=0.5)
    t.update([box(100, 100)], now=0.0)
    assert t.update([], now=0.2) is None        # stopped, still FOLLOWING
    assert t.state == "FOLLOWING"


def test_lost_after_grace_then_relocks_largest():
    t = TargetTracker(lost_grace_s=0.5)
    t.update([box(100, 100)], now=0.0)
    t.update([], now=0.6)
    assert t.state == "LOST"
    newcomer = box(400, 50, w=80, h=160)
    assert t.update([newcomer], now=1.0) is newcomer
    assert t.state == "FOLLOWING"


def test_reappearance_within_grace_reassociates():
    t = TargetTracker(lost_grace_s=0.5)
    t.update([box(100, 100)], now=0.0)
    t.update([], now=0.2)                       # one missed frame
    back = box(110, 102)
    assert t.update([back], now=0.3) is back
```

Run: `python -m pytest ai/tests/test_target_tracker.py -v`
Expected: FAIL — module doesn't exist

- [ ] **Step 2: Write the tracker**

`ai/person_following/tracker.py`:

```python
"""Lock-at-arm target tracker — stop on loss, no re-identification.

Design decision (spec): appearance re-ID silently follows the wrong person
when it mismatches — the worst failure shape for a follower. This tracker
only associates frame-to-frame by box overlap (IoU); any loss longer than a
short grace makes the robot STOP, and re-acquisition is simply "largest
detection of the target class" (the operator accepts that semantics).

States: SEARCHING (never had a target) / FOLLOWING / LOST. SEARCHING and
LOST behave identically (lock largest when something appears); they are
distinct only so the dashboard/preview can tell "never saw anyone" from
"had someone and lost them".
"""

SEARCHING = "SEARCHING"
FOLLOWING = "FOLLOWING"
LOST = "LOST"


def iou(a, b):
    """Intersection-over-union of two Detections."""
    x1, y1 = max(a.x, b.x), max(a.y, b.y)
    x2 = min(a.x + a.w, b.x + b.w)
    y2 = min(a.y + a.h, b.y + b.h)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = a.area + b.area - inter
    return inter / union if union > 0 else 0.0


class TargetTracker:
    def __init__(self, min_iou=0.2, lost_grace_s=0.5):
        self._min_iou = min_iou
        self._grace = lost_grace_s
        self.state = SEARCHING
        self._box = None
        self._last_seen = None

    def update(self, detections, now):
        """Feed this control step's detections (already class-filtered).

        Returns the target Detection to steer toward, or None (=> the
        controller must command zero motion).
        """
        if self.state == FOLLOWING:
            best, best_iou = None, 0.0
            for d in detections:
                v = iou(d, self._box)
                if v > best_iou:
                    best, best_iou = d, v
            if best is not None and best_iou >= self._min_iou:
                self._box = best
                self._last_seen = now
                return best
            if now - self._last_seen > self._grace:
                self.state = LOST
                self._box = None
            return None

        # SEARCHING or LOST: lock the largest target-class detection.
        if detections:
            self._box = max(detections, key=lambda d: d.area)
            self._last_seen = now
            self.state = FOLLOWING
            return self._box
        return None
```

- [ ] **Step 3: Run tests to verify they pass**

Run: `python -m pytest ai/tests/test_target_tracker.py -v`
Expected: 6 passed

- [ ] **Step 4: Commit**

```powershell
git add ai/person_following/tracker.py ai/tests/test_target_tracker.py
git commit -m "Lock-at-arm target tracker: IoU association, stop-on-loss, relock largest"
```

---

### Task 8: Follow controller (TDD) + FOLLOW config

**Files:**
- Create: `ai/person_following/controller.py`
- Create: `ai/tests/test_follow_controller.py`
- Modify: `ai/config.py`

- [ ] **Step 1: Add the FOLLOW config**

Append to `ai/config.py`:

```python
# ---- Person following (stage 2) ---------------------------------------------
# Caps MIRROR robot/cmd_vel_mux.py — the mux is the enforcement point, these
# are defense in depth. angular_sign/setpoints are starting values; the live
# tuning session adjusts them (same process as the gimbal tracker's).
FOLLOW = {
    "target_class": "person",   # any COCO class: "person", "dog", ...
    "score_threshold": 0.5,
    "input_size": 640,          # YOLO letterbox size (drop to 416 if <10 fps)
    "control_rate_hz": 10.0,
    "status_rate_hz": 2.0,
    # tracker
    "min_iou": 0.2,
    "lost_grace_s": 0.5,
    # controller — angular: P on horizontal offset (err_x in [-0.5, 0.5])
    "kp_ang": 2.0,              # rad/s per unit err_x
    "deadband_x": 0.05,         # |err_x| below this is "centered"
    "angular_sign": 1,          # flip to -1 if the robot turns AWAY (live check)
    # controller — linear: P on bbox-height fraction vs setpoint
    "height_setpoint": 0.55,    # bbox h / frame h at the desired follow distance
    "deadband_h": 0.05,
    "kp_lin": 1.2,              # m/s per unit height error
    "smoothing": 0.5,           # EMA weight on previous (cx, h) estimate
    # caps (mirror the mux) + blind-reverse limit
    "cap_fwd": 0.25, "cap_rev": 0.12, "cap_ang": 1.2,
    "reverse_limit_s": 1.5,     # max continuous blind reverse, then hold
    # gimbal is FIXED during following; chassis does the turning
    "gimbal_follow": {"pan": 90, "tilt": 22},
}
```

- [ ] **Step 2: Write the failing tests**

`ai/tests/test_follow_controller.py`:

```python
"""FollowController: P-control with caps, deadbands, and the reverse limit."""

import dataclasses

from ai.common.detection import Detection
from ai.person_following.controller import FollowController

FRAME = (1280, 720)


def cfg(**over):
    base = dict(kp_ang=2.0, deadband_x=0.05, angular_sign=1,
                height_setpoint=0.55, deadband_h=0.05, kp_lin=1.2,
                smoothing=0.0,   # EMA off in unit tests: pure P response
                cap_fwd=0.25, cap_rev=0.12, cap_ang=1.2, reverse_limit_s=1.5)
    base.update(over)
    return base


def target(cx_frac, h_frac):
    """A Detection centered at cx_frac of frame width with given height."""
    w, h = FRAME
    bw, bh = 100.0, h_frac * h
    return Detection(x=cx_frac * w - bw / 2, y=100.0, w=bw, h=bh,
                     score=0.9, label="person")


def test_centered_at_distance_is_still():
    c = FollowController(cfg())
    assert c.update(target(0.5, 0.55), FRAME, now=0.0) == (0.0, 0.0)


def test_no_target_is_zeros():
    c = FollowController(cfg())
    assert c.update(None, FRAME, now=0.0) == (0.0, 0.0)


def test_target_right_turns_right():
    c = FollowController(cfg())
    lin, ang = c.update(target(0.8, 0.55), FRAME, now=0.0)
    assert ang < 0           # right of center => clockwise (negative z)
    lin, ang = c.update(target(0.2, 0.55), FRAME, now=0.1)
    assert ang > 0


def test_angular_sign_flip():
    c = FollowController(cfg(angular_sign=-1))
    _, ang = c.update(target(0.8, 0.55), FRAME, now=0.0)
    assert ang > 0


def test_far_drives_forward_close_reverses_capped():
    c = FollowController(cfg())
    lin, _ = c.update(target(0.5, 0.20), FRAME, now=0.0)   # small box = far
    assert 0 < lin <= 0.25
    lin, _ = c.update(target(0.5, 0.95), FRAME, now=0.1)   # huge box = close
    assert -0.12 <= lin < 0


def test_reverse_time_limited_then_recovers():
    c = FollowController(cfg())
    too_close = target(0.5, 0.95)
    assert c.update(too_close, FRAME, now=0.0)[0] < 0
    assert c.update(too_close, FRAME, now=1.0)[0] < 0      # still within 1.5s
    assert c.update(too_close, FRAME, now=2.0)[0] == 0.0   # limit hit: hold
    assert c.update(too_close, FRAME, now=3.0)[0] == 0.0   # still holding
    far = target(0.5, 0.20)
    assert c.update(far, FRAME, now=4.0)[0] > 0            # forward resets it
    assert c.update(too_close, FRAME, now=5.0)[0] < 0      # reverse allowed again


def test_deadbands():
    c = FollowController(cfg())
    lin, ang = c.update(target(0.52, 0.57), FRAME, now=0.0)  # inside both bands
    assert (lin, ang) == (0.0, 0.0)
```

Run: `python -m pytest ai/tests/test_follow_controller.py -v`
Expected: FAIL — module doesn't exist

- [ ] **Step 3: Write the controller**

`ai/person_following/controller.py`:

```python
"""Follow controller: bbox -> (linear, angular) chassis command.

Angular: P on the target's horizontal offset from frame center (the gimbal
is fixed during following — the chassis does the turning). Linear: P on
bbox-height fraction vs the follow-distance setpoint; height is the distance
proxy (bigger box = closer). Both have deadbands so a centered, at-distance
target commands exact zeros (no idle creep).

Safety (spec): outputs clamped to the same caps the mux enforces (defense in
depth); reverse is additionally time-limited — the robot has no rear
sensors, so after reverse_limit_s of continuous backing it holds still until
the demand goes non-negative (target stepped back / moved away).

Latency note: the command->stream loop is ~0.8 s blind (measured, stage 1).
Gains here start LOW; if the chassis limit-cycles live, the tuning ladder is
EMA smoothing -> lower gains -> stage-1 move-and-settle pattern.
"""


class FollowController:
    def __init__(self, cfg):
        self._cfg = cfg
        self._ema = None            # smoothed (cx_px, h_px)
        self._reverse_since = None  # when continuous reverse began

    def reset(self):
        self._ema = None
        self._reverse_since = None

    def update(self, target, frame_size, now):
        """target: Detection or None. Returns (linear m/s, angular rad/s)."""
        cfg = self._cfg
        if target is None:
            self.reset()
            return (0.0, 0.0)

        w, h = frame_size
        cx, bh = target.center[0], target.h
        a = cfg["smoothing"]
        if self._ema is not None and a > 0:
            cx = a * self._ema[0] + (1 - a) * cx
            bh = a * self._ema[1] + (1 - a) * bh
        self._ema = (cx, bh)

        # Angular: err_x in [-0.5, 0.5]; target right of center => negative z
        # (clockwise) for a forward-facing camera. angular_sign flips it if
        # the live check disagrees.
        err_x = cx / w - 0.5
        ang = 0.0
        if abs(err_x) >= cfg["deadband_x"]:
            ang = -cfg["kp_ang"] * err_x * cfg["angular_sign"]
        ang = min(cfg["cap_ang"], max(-cfg["cap_ang"], ang))

        # Linear: positive height error = too far = drive forward.
        err_h = cfg["height_setpoint"] - bh / h
        lin = 0.0
        if abs(err_h) >= cfg["deadband_h"]:
            lin = cfg["kp_lin"] * err_h
        lin = min(cfg["cap_fwd"], max(-cfg["cap_rev"], lin))

        # Blind-reverse time limit.
        if lin < 0:
            if self._reverse_since is None:
                self._reverse_since = now
            elif now - self._reverse_since > cfg["reverse_limit_s"]:
                lin = 0.0   # hold; _reverse_since stays set until demand >= 0
        else:
            self._reverse_since = None

        return (lin, ang)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ai/tests/test_follow_controller.py -v`
Expected: 7 passed

- [ ] **Step 5: Run the full suite and commit**

Run: `python -m pytest` — all pass.

```powershell
git add ai/person_following/controller.py ai/tests/test_follow_controller.py ai/config.py
git commit -m "Follow controller: P-control, mux-mirrored caps, blind-reverse time limit"
```

---

### Task 9: Shared connect helper + runner

**Files:**
- Create: `ai/common/connect.py`
- Modify: `ai/face_tracking/__main__.py`
- Create: `ai/person_following/__main__.py`

- [ ] **Step 1: Extract the actuation-check helper**

`ai/common/connect.py`:

```python
"""Connect to rosbridge and PROVE commands reach the servos.

The robot's old rosbridge can half-fail its publisher registration and
silently drop every command for the session's lifetime (seen on two boots;
docs/AI_NOTEBOOK.md). No error reaches the websocket client, so the only
honest check is end-to-end: wiggle the tilt servo and verify the camera view
changed. A fresh session re-registers cleanly, so on failure we reconnect
and try again.

Shared by every behavior (extracted from face_tracking in stage 2). The
sink_factory must return an object with send(servo_id, angle) and
disconnect(); disconnect() is used between retries because close() would
kill the process-wide Twisted reactor for good.
"""

import time

from ai.common.video import frames_differ


def connect_with_actuation_check(sink_factory, src, tilt_cfg, attempts=3):
    for attempt in range(1, attempts + 1):
        sink = sink_factory()
        before = src.read(timeout_s=3.0)
        sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg + 25)
        time.sleep(1.8)
        after = src.read(timeout_s=3.0)
        sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg)
        time.sleep(1.0)
        if before is not None and after is not None and frames_differ(before, after):
            print(f"actuation check passed (attempt {attempt})")
            return sink
        print(f"actuation check FAILED on attempt {attempt}: commands are not "
              "reaching the gimbal (known rosbridge dropped-session bug) - "
              "reconnecting with a fresh session")
        sink.disconnect()
        time.sleep(2.0)
    raise RuntimeError(
        "gimbal did not respond after %d fresh rosbridge sessions - "
        "check the robot (journalctl -u rosbridge-dashboard.service)" % attempts)
```

- [ ] **Step 2: Re-point face_tracking at it**

In `ai/face_tracking/__main__.py`: delete the local `connect_with_actuation_check` function and the now-unused `frames_differ` import; add import and adapt the one call site:

```python
from ai.common.connect import connect_with_actuation_check
```

```python
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src, tilt_cfg)
```

Run: `python -m pytest` — all pass (the helper itself is exercised live; existing integration tests must not break).

- [ ] **Step 3: Write the runner**

`ai/person_following/__main__.py`:

```python
"""Person following — stage 2 of the AI roadmap.

Locks onto the largest target-class detection (person by default, dog via
--target-class) and drives the chassis to keep it centered at the follow
distance. Publishes /ai/cmd_vel — the robot-side mux forwards it only while
the dashboard's AI switch is ARMED and the joystick is quiet, clamped to the
AI caps. Lost target => zeros (and the mux + watchdog back that up).

Run modes (test offline first, per the plan):
  python -m ai.person_following --source clip.avi --dry-run   # recorded video
  python -m ai.person_following --source 0 --dry-run          # laptop webcam
  python -m ai.person_following --dry-run                     # robot camera, no commands
  python -m ai.person_following                               # live (must ARM on dashboard)
  python -m ai.person_following --cap-scale 0.5               # first live run: half caps

Keys in the preview window: q/ESC quit, t toggle a local pause (publishes
zeros while paused).
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from ai import config
from ai.common.connect import connect_with_actuation_check
from ai.common.safety import RateLimiter
from ai.common.video import VideoSource
from ai.person_following.controller import FollowController
from ai.person_following.detector import YoloDetector
from ai.person_following.tracker import TargetTracker

DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "models" / "yolo11n.onnx"


class DryRunSink:
    """Prints commands instead of publishing; always 'armed'."""

    armed = True

    def send(self, servo_id, angle):
        print(f"[dry-run] /PWMServo id={servo_id} angle={angle}")

    def send_twist(self, lin, ang):
        if abs(lin) > 1e-6 or abs(ang) > 1e-6:
            print(f"[dry-run] /ai/cmd_vel lin={lin:+.3f} ang={ang:+.3f}")

    def send_status(self, status):
        pass

    def close(self):
        pass


class RosSink:
    """Publishes via RosClient; tracks the /ai/enabled switch (defense in
    depth — the mux enforces it anyway)."""

    def __init__(self, url):
        from ai.common.ros_client import RosClient

        self._client = RosClient(url)
        self.armed = False
        print(f"connecting to rosbridge at {url} ...")
        self._client.connect()
        self._client.on_ai_enabled(self._on_enabled)
        print("rosbridge connected")

    def _on_enabled(self, armed):
        if armed != self.armed:
            print(f"AI {'ARMED' if armed else 'disarmed'} (dashboard)")
        self.armed = armed

    def send(self, servo_id, angle):
        self._client.send_pwm_servo(servo_id, angle)

    def send_twist(self, lin, ang):
        self._client.send_twist(lin, ang)

    def send_status(self, status):
        self._client.send_status(status)

    def disconnect(self):
        self._client.disconnect()

    def close(self):
        self._client.close()


def parse_args(argv):
    f = config.FOLLOW
    p = argparse.ArgumentParser(prog="ai.person_following", description=__doc__)
    p.add_argument("--profile", choices=sorted(config.PROFILES), default=config.DEFAULT_PROFILE)
    p.add_argument("--source", default=None,
                   help="video override: webcam index, file path, or URL")
    p.add_argument("--dry-run", action="store_true",
                   help="print commands instead of publishing")
    p.add_argument("--no-preview", action="store_true")
    p.add_argument("--record", default=None, metavar="OUT.AVI")
    p.add_argument("--model", default=str(DEFAULT_MODEL))
    p.add_argument("--target-class", default=f["target_class"],
                   help='COCO class to follow (e.g. "person", "dog")')
    p.add_argument("--cap-scale", type=float, default=1.0,
                   help="scale ALL output caps (0.5 for the first live run)")
    p.add_argument("--kp-ang", type=float, default=f["kp_ang"])
    p.add_argument("--kp-lin", type=float, default=f["kp_lin"])
    p.add_argument("--height-setpoint", type=float, default=f["height_setpoint"])
    p.add_argument("--angular-sign", type=int, choices=(-1, 1),
                   default=f["angular_sign"])
    p.add_argument("--smoothing", type=float, default=f["smoothing"])
    return p.parse_args(argv)


def draw_overlay(frame, dets, target, tracker, lin, ang, armed, dry_run, paused):
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 24, 1)
    for d in dets:
        color = (0, 255, 0) if d is target else (128, 128, 128)
        x, y = int(d.x), int(d.y)
        cv2.rectangle(frame, (x, y), (x + int(d.w), y + int(d.h)), color, 2)
        cv2.putText(frame, f"{d.label} {d.score:.2f}", (x, max(12, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    mode = "PAUSED" if paused else tracker.state
    arm_txt = "DRY RUN" if dry_run else ("ARMED" if armed else "disarmed")
    status = f"{mode}  lin {lin:+.2f}  ang {ang:+.2f}  [{arm_txt}]"
    cv2.putText(frame, status, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 255), 1, cv2.LINE_AA)


def main(argv=None):
    args = parse_args(argv)
    f = dict(config.FOLLOW)
    f.update(kp_ang=args.kp_ang, kp_lin=args.kp_lin,
             height_setpoint=args.height_setpoint,
             angular_sign=args.angular_sign, smoothing=args.smoothing,
             cap_fwd=f["cap_fwd"] * args.cap_scale,
             cap_rev=f["cap_rev"] * args.cap_scale,
             cap_ang=f["cap_ang"] * args.cap_scale)
    profile = config.PROFILES[args.profile]

    source = profile["video_url"] if args.source is None else args.source
    if isinstance(source, str) and source.isdigit():
        source = int(source)

    detector = YoloDetector(args.model, score_threshold=f["score_threshold"],
                            input_size=f["input_size"])
    tracker = TargetTracker(min_iou=f["min_iou"], lost_grace_s=f["lost_grace_s"])
    controller = FollowController(f)
    control_gate = RateLimiter(min_interval_s=1.0 / f["control_rate_hz"])
    status_gate = RateLimiter(min_interval_s=1.0 / f["status_rate_hz"])

    pace_s = None
    if isinstance(source, str) and Path(source).is_file():
        probe = cv2.VideoCapture(source)
        fps = probe.get(cv2.CAP_PROP_FPS) or 30.0
        probe.release()
        pace_s = 1.0 / fps

    recorder = None
    sink = None
    lin = ang = 0.0
    fps_t0, fps_n, det_fps = time.monotonic(), 0, 0.0
    print(f"video source: {source}  target class: {args.target_class}")

    try:
        with VideoSource(source, pace_s=pace_s) as src:
            if args.dry_run:
                sink = DryRunSink()
            else:
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src, config.GIMBAL_TILT)
                # Park the gimbal at the follow pose: chassis does the turning.
                sink.send(config.GIMBAL_PAN.servo_id, f["gimbal_follow"]["pan"])
                time.sleep(0.2)  # pace the two servo commands (driver quirk)
                sink.send(config.GIMBAL_TILT.servo_id, f["gimbal_follow"]["tilt"])

            paused = False
            while True:
                frame = src.read(timeout_s=1.0)
                if frame is None:
                    if not src.alive:
                        print("stream ended")
                        break
                    print("waiting for frames ...")
                    continue

                dets = [d for d in detector.detect(frame)
                        if d.label == args.target_class]
                fps_n += 1
                if time.monotonic() - fps_t0 >= 2.0:
                    det_fps = fps_n / (time.monotonic() - fps_t0)
                    fps_t0, fps_n = time.monotonic(), 0

                h, w = frame.shape[:2]
                target = None
                if control_gate.ready():
                    now = time.monotonic()
                    target = tracker.update(dets, now) if not paused else None
                    lin, ang = controller.update(target, (w, h), now)
                    # Publish continuously while armed: a steady stream (zeros
                    # included) keeps the mux/watchdog state machines quiet.
                    if sink.armed:
                        sink.send_twist(lin, ang)

                if status_gate.ready():
                    sink.send_status({
                        "state": "PAUSED" if paused else tracker.state,
                        "target_class": args.target_class,
                        "fps": round(det_fps, 1),
                    })

                if recorder is None and args.record:
                    recorder = cv2.VideoWriter(
                        args.record, cv2.VideoWriter_fourcc(*"MJPG"), 15.0, (w, h))
                if recorder is not None:
                    recorder.write(frame)

                if not args.no_preview:
                    draw_overlay(frame, dets, target, tracker, lin, ang,
                                 getattr(sink, "armed", False), args.dry_run, paused)
                    cv2.imshow("transbot person following", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("t"):
                        paused = not paused
    finally:
        if sink is not None:
            try:
                sink.send_twist(0.0, 0.0)   # parting zero, belt and braces
            except Exception:
                pass
        if recorder is not None:
            recorder.release()
        cv2.destroyAllWindows()
        if sink is not None:
            sink.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Smoke-test offline**

Run: `python -m ai.person_following --source ai/tests/data/people.jpg --dry-run --no-preview`
(a still image plays as a 1-frame stream) — Expected: prints `video source: ...`, possibly one `[dry-run] /ai/cmd_vel ...` line, then `stream ended`, exit 0. Also run `python -m pytest` — all pass.

- [ ] **Step 5: Commit**

```powershell
git add ai/common/connect.py ai/face_tracking/__main__.py ai/person_following/__main__.py
git commit -m "Person-following runner + shared rosbridge actuation-check helper"
```

---

### Task 10: Offline validation on recorded video (operator)

No new files — a verification gate before anything touches the robot.

- [ ] **Step 1: Record a walk-around clip** (with the robot on and stationary; or reuse any prior recording):

```powershell
python -m ai.face_tracking --dry-run --record walk_test.avi --no-preview
```

Walk around the room in view of the robot camera for ~60 s (vary distance: close, far, leave frame, return; have a second person cross through if possible). Ctrl+C / q to stop.

- [ ] **Step 2: Run the follower against the clip**

```powershell
python -m ai.person_following --source walk_test.avi --dry-run
```

Verify in the preview + console:
1. Your body gets a green (locked) box; state FOLLOWING while visible.
2. Walking left/right of center prints `ang` commands of the expected sign (target right of center → negative ang with default sign).
3. Walking away prints positive lin (forward); walking close prints negative lin, magnitude ≤ 0.12.
4. Leaving the frame → state LOST within ~1 s, commands go silent (zeros are not printed by the dry-run sink).
5. A second person crossing does NOT steal the lock while you stay visible.

- [ ] **Step 3: Record acceptance** — note any gain/setpoint adjustments made (CLI flags map 1:1 to `FOLLOW` keys; persist tuned values into `ai/config.py` with a comment, same convention as `TRACKER`). Commit any tuning:

```powershell
git add ai/config.py
git commit -m "Follow gains tuned against recorded walk-around clip"
```

---

### Task 11: Robot deployment + mux live verification (operator, robot on)

Prereq: robot powered, on home Wi-Fi (192.168.0.109), NordVPN off.

- [ ] **Step 1: Deploy**

```powershell
.\tools\deploy_robot.ps1 -JetsonIp 192.168.0.109 -User jetson
ssh jetson@192.168.0.109 "sudo systemctl restart rosbridge-dashboard.service"
```

Wait ~60 s for the stack to come up.

- [ ] **Step 2: Verify the mux is alive**

```powershell
ssh jetson@192.168.0.109 "bash -lc 'source /opt/ros/melodic/setup.bash; rostopic list | grep -E \"mux|manual|ai\"'"
```

Expected: `/ai/cmd_vel`, `/ai/enabled`, `/manual/cmd_vel`, `/mux/status` all listed.

```powershell
ssh jetson@192.168.0.109 "bash -lc 'source /opt/ros/melodic/setup.bash; rostopic echo -n1 /mux/status'"
```

Expected: `data: "{\"source\": \"none\", \"armed\": false, ...}"`.

If the mux node is missing: `journalctl -u rosbridge-dashboard.service -b | grep -i mux` (remember the robot clock may lie about dates — use `-b`, not `--since`).

- [ ] **Step 3: Manual drive THROUGH the mux** (robot on a stand or floor with space)

Dashboard on HOME WIFI profile:
1. Hold W — robot drives; DRIVE SRC shows `MANUAL`. (This proves `/manual/cmd_vel` → mux → `/cmd_vel` end-to-end.)
2. Release — robot stops (deadman unchanged).
3. SPACE e-stop mid-drive — instant stop.
4. ARM AI with no AI process running — AI ARMED `YES` on the panel; robot stays still; W still drives (manual wins). DISARM.

- [ ] **Step 4: Watchdog regression check** — drive forward, then kill the dashboard tab mid-drive: robot must stop within ~0.5 s (watchdog still guards `/cmd_vel`).

- [ ] **Step 5: Commit any fixes found, and record results** in the session (they feed the notebook in Task 13).

---

### Task 12: First live follow + tuning (operator, open space)

Prereqs: Tasks 10–11 passed. Open floor space, operator at the dashboard with a hand on the keyboard (SPACE = e-stop, any WASD = instant manual takeover via the mux window).

- [ ] **Step 1: Dry-run against the live camera** (no commands):

```powershell
python -m ai.person_following --dry-run
```

Confirm detection/lock/state behavior on the live stream looks like Task 10.

- [ ] **Step 2: First powered run at half caps:**

```powershell
python -m ai.person_following --cap-scale 0.5
```

1. Wait for the actuation check to pass and the gimbal to park at the follow pose.
2. ARM AI on the dashboard. Step in front of the robot at ~2 m.
3. Verify, in order: (a) rotate-to-face — if it turns AWAY, quit, rerun with `--angular-sign -1`, and persist the flip in `ai/config.py`; (b) walk away slowly — it follows; (c) walk toward it — it backs up, then holds after ~1.5 s of reverse; (d) step out of view — it stops within ~1 s; (e) grab W mid-follow — manual takeover is instant, AI resumes ~1 s after release; (f) DISARM — instant halt.

- [ ] **Step 3: Tune** (kp/setpoint/smoothing via flags; if it limit-cycles on stream latency, lower `kp_ang`/`kp_lin` first, raise `smoothing` second; the stage-1 move-and-settle pattern is the documented fallback). Persist final values in `ai/config.py` with a MEASURED/tuned comment, then full caps run (`--cap-scale 1.0`).

- [ ] **Step 4: Commit tuned config:**

```powershell
git add ai/config.py
git commit -m "Person following live-tuned: gains, setpoint, angular sign confirmed"
```

---

### Task 13: Docs

**Files:**
- Modify: `README.md`
- Create: `docs/PERSON_FOLLOWING_NOTEBOOK.md` (+ generated .docx)

- [ ] **Step 1: README** — in the Status list, mark AI stage 2 done; in the "AI behaviors" section add the run line `python -m ai.person_following` (+ `--target-class dog`) and mention the mux + AI panel; in "Extending this", note the mux is now live (any new autonomous publisher goes through `/ai/cmd_vel`).

- [ ] **Step 2: Notebook** — write `docs/PERSON_FOLLOWING_NOTEBOOK.md` in the established notebook style (problem → architecture → what was measured → what failed → final constants), from the Task 10–12 session results. Convert: `python tools/md_to_docx.py docs/PERSON_FOLLOWING_NOTEBOOK.md` — REMEMBER the converter takes NO bold/backticks/code-fences (headings, tables, bullets only).

- [ ] **Step 3: Commit**

```powershell
git add README.md docs/PERSON_FOLLOWING_NOTEBOOK.md docs/PERSON_FOLLOWING_NOTEBOOK.docx
git commit -m "Stage 2 docs: README status + person-following engineering notebook"
```

---

## Self-review (done at write time)

- **Spec coverage:** mux rules/caps/status → Tasks 1–2, 11; dashboard topic move + panel → Tasks 3–4; YOLO detector + benchmark gate → Task 6; lock tracker → Task 7; controller incl. reverse cap + time limit + deadbands → Task 8; defense-in-depth arming + continuous-publish + actuation check + fixed gimbal pose → Task 9; recorded-video gate → Task 10; deploy + manual-through-mux + watchdog regression → Task 11; live tuning ladder + angular-sign check → Task 12; notebook/README → Task 13. No spec items without a task.
- **Known judgment calls:** mux forwards manual messages verbatim (never modified); AI silence timeout lives in the mux *and* the watchdog still guards `/cmd_vel` (intentional redundancy); `select_primary` kept as alias so stage-1 tests/files don't churn.
- **Type consistency check:** `Detection(label=...)` added in Task 5 and used by Tasks 6–9; `MuxCore` method names match between Tasks 1 and 2's plumbing; `FOLLOW` keys referenced in Tasks 8–9 all exist in the Task 8 config block; `sink_factory` signature in Task 9 matches both call sites.
