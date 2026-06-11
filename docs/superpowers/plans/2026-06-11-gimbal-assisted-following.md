# Gimbal-Assisted Person Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The gimbal tracks the locked person fast (move-and-settle); the chassis steers on total bearing (pan offset + image offset) so the robot squares up to the person without losing them off the frame edge.

**Architecture:** Reuse stage 1's `GimbalTracker` unchanged as the fast inner loop. `FollowController` gains an optional `pan_offset_deg` input and computes its angular error as bearing-fused `err_total = err_x + pan_sign * pan_offset / deg_per_errx` (invariant to gimbal moves, so the loops cannot fight). Forward/reverse magnitude is scaled by `cos(bearing)`, floored at 0. A `--fixed-gimbal` flag restores today's parked-gimbal behavior.

**Tech Stack:** Python 3, OpenCV, roslibpy (existing `ai/` package). Spec: `docs/superpowers/specs/2026-06-11-gimbal-assisted-following-design.md`.

**Key existing facts (verified in code, do not rediscover):**
- `GimbalTracker` (`ai/face_tracking/tracker.py`) is target-agnostic: `update(center_or_None, (w, h))` returns a list of `(servo_id, int_angle)` commands; `start_commands()`/`recenter()` drive both axes to `home_deg`; `pan_deg` property exposes the current pan estimate. Lost-target handling (hold, then recenter after `lost_recenter_after` updates) is built in.
- `AxisConfig` is a frozen-style dataclass — override `home_deg` with `dataclasses.replace`.
- Pan sign is -1 (hardware-confirmed): a person RIGHT of center (err_x > 0) drives pan BELOW 90, so `pan_offset_deg` is NEGATIVE for a person on the right. `pan_sign * pan_offset` maps it back to image-error convention (positive = person right).
- `deg_per_errx` = 97.0: measured pan full-correction gain is 48.5 deg per unit of half-frame-normalized error (range ±1); `err_x` spans ±0.5, so degrees per err_x unit = 2 × 48.5.
- The servo driver can drop back-to-back commands — when sending two `/PWMServo` commands in one tick, sleep 0.15 s between them (the move-and-settle window makes this rare and absorbs the delay).
- Tests run with `python -m pytest ai/tests -q` from the repo root (54+ tests, all green before starting).

---

### Task 1: Bearing fusion + cos scaling in FollowController

**Files:**
- Modify: `ai/person_following/controller.py`
- Test: `ai/tests/test_follow_controller.py`

- [ ] **Step 1: Write the failing tests**

In `ai/tests/test_follow_controller.py`:

Add `import pytest` at the top (below the existing imports). Then add the two new keys to the `cfg()` helper's base dict:

```python
def cfg(**over):
    base = dict(kp_ang=2.0, deadband_x=0.05, angular_sign=1,
                height_setpoint=0.55, deadband_h=0.05, kp_lin=1.2,
                smoothing=0.0,   # EMA off in unit tests: pure P response
                cap_fwd=0.25, cap_rev=0.12, cap_ang=1.2, reverse_limit_s=1.5,
                deg_per_errx=97.0, pan_sign=-1)
    base.update(over)
    return base
```

Append these tests at the end of the file:

```python
def test_pan_offset_alone_turns_chassis():
    # Camera panned toward a person on the right (pan sign -1: offset is
    # negative), image centered: the chassis must still turn right (ang < 0)
    # to square up — "always re-center" per the spec.
    c = FollowController(cfg())
    _, ang = c.update(target(0.5, 0.55), FRAME, now=0.0, pan_offset_deg=-20.0)
    assert ang < 0


def test_bearing_invariant_to_gimbal_moves():
    # The same physical bearing split two ways must command the same turn:
    # all in the image (gimbal centered) vs partly absorbed by a 20 deg pan.
    c1 = FollowController(cfg())
    _, ang_image = c1.update(target(0.8, 0.55), FRAME, now=0.0)
    c2 = FollowController(cfg())
    _, ang_split = c2.update(target(0.8 - 20.0 / 97.0, 0.55), FRAME, now=0.0,
                             pan_offset_deg=-20.0)
    assert ang_split == pytest.approx(ang_image)


def test_deadband_applies_to_total_bearing():
    # Small pan offset + centered image = "facing them": no turn.
    # 4 deg / 97 = 0.041 err units, inside the 0.05 deadband.
    c = FollowController(cfg())
    _, ang = c.update(target(0.5, 0.55), FRAME, now=0.0, pan_offset_deg=-4.0)
    assert ang == 0.0


def test_forward_scaled_down_at_large_bearing():
    # Far target square ahead vs the same target 60 deg off to the side:
    # forward speed must drop (cos scaling) but not necessarily to zero.
    c = FollowController(cfg())
    lin_square, _ = c.update(target(0.5, 0.20), FRAME, now=0.0)
    c.reset()
    lin_offaxis, _ = c.update(target(0.5, 0.20), FRAME, now=1.0,
                              pan_offset_deg=-60.0)
    assert 0 <= lin_offaxis < lin_square


def test_no_drive_past_90_deg_bearing():
    # cos floor: beyond 90 deg of bearing, no forward AND no reverse.
    c = FollowController(cfg())
    lin, _ = c.update(target(0.5, 0.20), FRAME, now=0.0, pan_offset_deg=-97.0)
    assert lin == 0.0
    c.reset()
    lin, _ = c.update(target(0.5, 0.95), FRAME, now=1.0, pan_offset_deg=-97.0)
    assert lin == 0.0
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `python -m pytest ai/tests/test_follow_controller.py -q`
Expected: the 5 new tests FAIL with `TypeError: update() got an unexpected keyword argument 'pan_offset_deg'`; the 7 existing tests still PASS (their `cfg()` gained keys the controller doesn't read yet — harmless).

- [ ] **Step 3: Implement bearing fusion in the controller**

In `ai/person_following/controller.py`:

Add `import math` at the top of the file (first code line after the docstring).

Update the module docstring's first paragraph to reflect the new angular source — replace:

```
Angular: P on the target's horizontal offset from frame center (the gimbal
is fixed during following — the chassis does the turning). Linear: P on
```

with:

```
Angular: P on the target's total bearing — image offset plus the gimbal's
pan offset from its follow-pose home. The sum is invariant to gimbal moves
(a pan toward the person shrinks the image error by exactly what the pan
offset grows), so the chassis loop is decoupled from the gimbal's settle
cycle. With the gimbal parked (--fixed-gimbal) it reduces to plain P on the
image offset, today's live-tuned behavior. Linear: P on
```

Replace the `update` signature and the angular/linear blocks. The method becomes:

```python
    def update(self, target, frame_size, now, pan_offset_deg=0.0):
        """target: Detection or None. pan_offset_deg: gimbal pan minus its
        follow-pose home (0 when the gimbal is parked).
        Returns (linear m/s, angular rad/s)."""
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

        # Angular: err in [-0.5, 0.5]-ish units; target right of center =>
        # negative z (clockwise) for a forward-facing camera. pan_sign maps
        # the servo offset back into image-error convention (pan sign -1:
        # person right => pan below home => negative offset => positive err).
        # angular_sign flips the output if the live check disagrees.
        err_x = cx / w - 0.5
        err_total = err_x + cfg["pan_sign"] * pan_offset_deg / cfg["deg_per_errx"]
        ang = 0.0
        if abs(err_total) >= cfg["deadband_x"]:
            ang = -cfg["kp_ang"] * err_total * cfg["angular_sign"]
        ang = min(cfg["cap_ang"], max(-cfg["cap_ang"], ang))

        # Linear: positive height error = too far = drive forward. Scaled by
        # cos(bearing), floored at 0 — never drive forward/reverse at speed
        # while the person stands far off-axis (the robot has no side
        # sensors and isn't looking where it's going).
        err_h = cfg["height_setpoint"] - bh / h
        lin = 0.0
        if abs(err_h) >= cfg["deadband_h"]:
            lin = cfg["kp_lin"] * err_h
        lin *= max(0.0, math.cos(math.radians(err_total * cfg["deg_per_errx"])))
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

(Only the docstring/signature, the `err_total` lines, and the `lin *=` line are new; everything else is the existing body, shown for context.)

- [ ] **Step 4: Run the full controller test file to verify all pass**

Run: `python -m pytest ai/tests/test_follow_controller.py -q`
Expected: 12 passed (7 existing + 5 new). The existing tests pass unchanged because `pan_offset_deg` defaults to 0.0 (the pan term vanishes and `cos(err_x * 97deg)` only scales `lin` when off-center — `test_far_drives_forward_close_reverses_capped` and `test_reverse_time_limited_then_recovers` use centered targets, so their `lin` is unscaled).

- [ ] **Step 5: Run the whole suite (regression check)**

Run: `python -m pytest ai/tests -q`
Expected: all pass. If `test_integration.py` calls `controller.update` positionally, the default arg keeps it green.

- [ ] **Step 6: Commit**

```bash
git add ai/person_following/controller.py ai/tests/test_follow_controller.py
git commit -m "Follow controller: bearing fusion (image + gimbal pan) + cos drive scaling"
```

---

### Task 2: Config — gimbal-track block and bearing constants

**Files:**
- Modify: `ai/config.py` (the `FOLLOW` dict, lines ~64-94)

- [ ] **Step 1: Add the new FOLLOW keys**

In `ai/config.py`, inside `FOLLOW`, replace the last entry and its comment:

```python
    # gimbal is FIXED during following; chassis does the turning
    "gimbal_follow": {"pan": 90, "tilt": 22},
```

with:

```python
    # Follow pose: the gimbal tracker's HOME while following (and the park
    # pose under --fixed-gimbal). Same values as the axis-config home today,
    # but sourced here so they can diverge.
    "gimbal_follow": {"pan": 90, "tilt": 22},
    # Bearing fusion (spec 2026-06-11): chassis steers on total bearing =
    # image err_x + pan_sign * pan_offset / deg_per_errx. deg_per_errx
    # converts servo degrees to err_x units: measured pan calibration is
    # 48.5 deg per half-frame error unit (range +/-1); err_x spans +/-0.5,
    # so 2 x 48.5 = 97 deg per err_x unit.
    "deg_per_errx": 97.0,
    "pan_sign": GIMBAL_PAN.sign,
    # GimbalTracker params while following — stage-1 move-and-settle values
    # (TRACKER block above), rehomed to the follow pose by the runner.
    "gimbal_track": {
        "kd_deg": 0.0,
        "deadband": 0.06,
        "max_step_deg": 20.0,
        "smoothing": 0.5,
        "settle_updates": 11,        # hold fire ~1.1 s after each move
        "lost_recenter_after": 50,   # ~5 s lost at 10 Hz -> back to follow pose
    },
```

- [ ] **Step 2: Run the suite**

Run: `python -m pytest ai/tests -q`
Expected: all pass (config is additive).

- [ ] **Step 3: Commit**

```bash
git add ai/config.py
git commit -m "Config: FOLLOW gimbal_track block + bearing-fusion constants"
```

---

### Task 3: Runner integration — gimbal tracking in the follow loop

**Files:**
- Modify: `ai/person_following/__main__.py`

- [ ] **Step 1: Imports and CLI flag**

Add `import dataclasses` to the stdlib import block, and the tracker import below the existing `ai.` imports:

```python
from ai.face_tracking.tracker import GimbalTracker
```

(Cross-package import is the established pattern — `ai/config.py` already imports `AxisConfig` from there.)

In `parse_args`, after the `--cap-scale` argument, add:

```python
    p.add_argument("--fixed-gimbal", action="store_true",
                   help="park the gimbal at the follow pose instead of "
                        "tracking the person with it (pre-spec behavior)")
```

- [ ] **Step 2: Instantiate the gimbal tracker in main()**

In `main()`, right after `controller = FollowController(f)`, add:

```python
    # Fast inner loop: the gimbal keeps the person in frame between chassis
    # corrections. Rehomed to the follow pose; stage-1 move-and-settle params.
    gt = f["gimbal_track"]
    gimbal = GimbalTracker(
        pan=dataclasses.replace(config.GIMBAL_PAN,
                                home_deg=f["gimbal_follow"]["pan"]),
        tilt=dataclasses.replace(config.GIMBAL_TILT,
                                 home_deg=f["gimbal_follow"]["tilt"]),
        kp_deg=config.TRACKER["kp_deg"], kd_deg=gt["kd_deg"],
        deadband=gt["deadband"], max_step_deg=gt["max_step_deg"],
        lost_recenter_after=gt["lost_recenter_after"],
        smoothing=gt["smoothing"], settle_updates=gt["settle_updates"])
```

- [ ] **Step 3: Replace the manual gimbal park with the tracker's start sync**

Replace these lines in the live branch:

```python
                # Park the gimbal at the follow pose: chassis does the turning.
                sink.send(config.GIMBAL_PAN.servo_id, f["gimbal_follow"]["pan"])
                time.sleep(0.2)  # pace the two servo commands (driver quirk)
                sink.send(config.GIMBAL_TILT.servo_id, f["gimbal_follow"]["tilt"])
```

with a helper call placed AFTER the `if args.dry_run: ... else: ...` block (so it runs in both modes — dry-run just prints):

```python
            # Sync: drive the gimbal to the follow pose so tracker state
            # matches reality (the driver does not echo servo positions).
            send_servo_commands(sink, gimbal.start_commands())
```

and add this helper at module level (above `draw_overlay`):

```python
def send_servo_commands(sink, commands):
    """Send /PWMServo commands, paced — the driver drops back-to-back sends."""
    for i, (servo_id, angle) in enumerate(commands):
        if i:
            time.sleep(0.15)
        sink.send(servo_id, angle)
```

- [ ] **Step 4: Wire the gimbal into the control tick**

Replace the control-gate block:

```python
                target = None
                if control_gate.ready():
                    now = time.monotonic()
                    target = tracker.update(dets, now) if not paused else None
                    lin, ang = controller.update(target, (w, h), now)
```

with:

```python
                target = None
                if control_gate.ready():
                    now = time.monotonic()
                    target = tracker.update(dets, now) if not paused else None
                    if not paused and not args.fixed_gimbal:
                        # Gimbal tracks whenever a target is locked, armed or
                        # not — arming gates chassis motion only. On loss it
                        # holds (best relock odds), then recenters to the
                        # follow pose after ~5 s (GimbalTracker built-in).
                        center = target.center if target is not None else None
                        send_servo_commands(sink, gimbal.update(center, (w, h)))
                    lin, ang = controller.update(
                        target, (w, h), now,
                        pan_offset_deg=gimbal.pan_deg - f["gimbal_follow"]["pan"])
```

Notes for the implementer:
- With `--fixed-gimbal`, `gimbal.update` is never called, so `gimbal.pan_deg` stays at home and `pan_offset_deg` is exactly 0.0 — today's behavior, no special-casing needed.
- `paused` skips `gimbal.update` entirely (spec: pause freezes the gimbal — no lost-counter ticking toward a surprise recenter while paused).

- [ ] **Step 5: Show the gimbal pose in the overlay**

Change the `draw_overlay` signature from:

```python
def draw_overlay(frame, dets, target, tracker, lin, ang, armed, dry_run, paused):
```

to:

```python
def draw_overlay(frame, dets, target, tracker, gimbal, lin, ang, armed,
                 dry_run, paused):
```

and change its status line from:

```python
    status = f"{mode}  lin {lin:+.2f}  ang {ang:+.2f}  [{arm_txt}]"
```

to:

```python
    status = (f"{mode}  lin {lin:+.2f}  ang {ang:+.2f}  "
              f"pan {gimbal.pan_deg:5.1f} tilt {gimbal.tilt_deg:5.1f}  [{arm_txt}]")
```

Update the call site to match:

```python
                    draw_overlay(frame, dets, target, tracker, gimbal, lin, ang,
                                 getattr(sink, "armed", False), args.dry_run, paused)
```

- [ ] **Step 6: Update the module docstring**

In the docstring at the top of `__main__.py`, after the line "Locks onto the largest target-class detection (person by default, dog via --target-class) and drives the chassis to keep it centered at the follow distance.", insert:

```
The gimbal tracks the person too (fast, move-and-settle) and the chassis
steers on the total bearing — see the 2026-06-11 spec. --fixed-gimbal
restores the parked-gimbal behavior for A/B comparison.
```

- [ ] **Step 7: Run the full suite**

Run: `python -m pytest ai/tests -q`
Expected: all pass.

- [ ] **Step 8: Dry-run smoke test (no robot needed)**

Run: `python -m ai.person_following --source 0 --dry-run` (laptop webcam; any recorded clip path works too). Stand in view, drift sideways.
Expected: preview shows the lock box plus `pan`/`tilt` in the status line; `[dry-run] /PWMServo ...` lines appear as you move (at most every ~1.1 s — move-and-settle), pan value in the overlay walks toward you; `/ai/cmd_vel` lines show `ang` responding to the combined bearing. Press `t`: servo prints stop. Press `q`: clean exit.
Then: `python -m ai.person_following --source 0 --dry-run --fixed-gimbal`
Expected: no `/PWMServo` prints after the two startup park commands; behavior identical to the pre-change runner.

- [ ] **Step 9: Commit**

```bash
git add ai/person_following/__main__.py
git commit -m "Runner: gimbal tracks the person; chassis steers on fused bearing"
```

---

### Task 4: Live validation notes (operator-in-the-loop, not automatable)

**Files:** none (checklist for the live session; results go to `docs/PERSON_FOLLOWING_NOTEBOOK.md` afterwards)

- [ ] **Step 1: Preflight** — dashboard up, robot on home Wi-Fi, AI panel visible. The runner's existing `connect_with_actuation_check` covers `/PWMServo` (tilt wiggle); remember the per-topic registration bug — verify `/ai/status` arrives robot-side and arming flips `/mux/status` to armed before trusting the session (restart `rosbridge-dashboard.service` if not).
- [ ] **Step 2: Disarmed sentry check** — start the runner live (not dry-run), do NOT arm. The gimbal alone should track you around the room. This validates lock + gimbal + signs with zero chassis risk.
- [ ] **Step 3: First armed run** — `--cap-scale 0.5`. Walk a slow arc around the robot: gimbal leads, chassis follows until it faces you, gimbal returns to ~90. Watch for hunting (tuning ladder: lower `kp_ang` → raise `gimbal_track.settle_updates` → `--fixed-gimbal`).
- [ ] **Step 4: A/B** — same arc with `--fixed-gimbal`: confirm the new mode holds lock through motion that breaks the old one (that's the acceptance test for this feature).
- [ ] **Step 5: Full caps + notebook** — if stable, drop `--cap-scale`, then record findings (tuned values, sign confirmations, surprises) in `docs/PERSON_FOLLOWING_NOTEBOOK.md` and commit.
