# Gimbal-Assisted Person Following — Design

**Date:** 2026-06-11
**Status:** Approved by operator
**Builds on:** stage 2 person following (live-validated 2026-06-11, `docs/PERSON_FOLLOWING_NOTEBOOK.md`) and stage 1 gimbal tracking (`docs/GIMBAL_TRACKING_NOTEBOOK.md`).

## Problem

Today the gimbal is parked at the follow pose (pan 90 / tilt 22) and the
chassis is the only thing keeping the person in the camera's field of view.
Chassis rotation is slow (capped, latency-limited), so a person moving
sideways can slip out of the frame edge before the chassis catches up — lock
is lost and following stops.

## Goal

Let the gimbal do the fast "keep the person in frame" work (it is quicker
than chassis rotation and safe — no wheel motion), while the chassis steers
to squarely face the person. Lock survives lateral person motion that would
previously have escaped the frame.

Operator decisions:

- **Chassis role:** always re-center — the chassis continuously steers to
  bring the gimbal back to pan 90, so the robot ends up facing the person.
- **Gimbal axes:** pan + tilt. Tilt offset is absorbed by the gimbal alone
  (the chassis has nothing to hand a tilt error to).

## Architecture

Two loops sharing one truth. The person's bearing relative to the robot body
is `pan_offset + image_offset`, and this sum is **invariant to gimbal
moves**: when the gimbal pans 15° toward the person, the image error shrinks
by exactly what the pan offset grows. The chassis loop therefore does not
care what the gimbal is doing mid-correction; the loops cannot fight each
other. Only gain-calibration error leaks between them.

### Gimbal loop (fast, inner)

Reuse stage 1's `GimbalTracker` (`ai/face_tracking/tracker.py`) unchanged —
it is target-agnostic (takes any center point). It tracks the locked
person's bbox center on both axes with the proven move-and-settle pattern
and the measured constants: pan kp 41, tilt kp 20, settle ~1.1 s, per-move
clamp 20°, deadband 0.06. Home for this behavior is the follow pose
(`FOLLOW["gimbal_follow"]`, pan 90 / tilt 22 — currently the same values as
the axis-config home, but sourced from the follow config so they can diverge).

### Chassis angular (slower, outer)

In `FollowController`, the angular error changes from raw image error to
total bearing, expressed in the same err_x units the live-tuned gains
already speak:

```
err_total = err_x + pan_sign * (pan_deg - 90) / deg_per_errx
ang = -kp_ang * err_total * angular_sign      (existing deadband, cap)
```

- `deg_per_errx` comes from the measured pan calibration: 48.5°/unit of
  half-frame-normalized error → ~97° per err_x unit (err_x spans ±0.5).
- `pan_sign` is `GIMBAL_PAN.sign` (-1, hardware-confirmed) — it maps a pan
  offset back into image-error convention.
- When the gimbal is centered this reduces to exactly today's live-tuned
  behavior; kp_ang 0.6 and deadband_x 0.10 keep their meaning.

### Chassis linear

Unchanged P on bbox-height fraction, with one addition: command magnitude is
scaled by `cos(bearing)`, floored at 0 — the robot must not drive straight
ahead (or reverse) at speed while the person stands far off to the side. At
small bearings cos ≈ 1, so normal following is untouched.

### Sign conventions

Taken from the existing axis configs. The existing `angular_sign` live-check
procedure still applies, with the notebook's two ground truths (dashboard A
key = +ang = nose-left; step-sideways = box moves the same way = not
mirrored) to settle any dispute on the robot.

## Runner integration (`ai/person_following/__main__.py`)

- Instantiate a `GimbalTracker` with the follow pose as home.
- Each control tick: `TargetTracker` picks the target →
  `GimbalTracker.update(target.center, frame_size)` → send resulting
  `/PWMServo` commands, **paced 150 ms apart** when both axes move (driver
  drops back-to-back servo commands) → `FollowController.update(...)` now
  also receives `gimbal.pan_deg`.
- The gimbal tracks **whenever a target is locked, regardless of arm
  state** — arming gates chassis motion only. A disarmed robot that watches
  the operator with its camera doubles as a pre-flight check that lock
  works.
- Pause (`t` key) freezes the gimbal too (no servo commands while paused).
- Lost target: chassis zeros immediately (unchanged); gimbal holds pointing
  where the person vanished (best relock odds), then recenters to the follow
  pose after ~5 s lost (stage-1 pattern).
- Startup: park at the follow pose (existing behavior, now via the tracker's
  recenter so internal state matches reality).

### Escape hatch

A `--fixed-gimbal` CLI flag restores today's parked-gimbal behavior, for
live A/B comparison and as a fallback if the coupled tuning fights us.

## Config

New `FOLLOW["gimbal_track"]` block in `ai/config.py`: settle, deadband,
max step, smoothing, lost-recenter (starting at stage-1 `TRACKER` values)
plus `deg_per_errx` (97, derived from the measured calibration). Existing
FOLLOW gains untouched.

## Failure modes

- **Gimbal hits its pan limit (0/180°):** image error keeps growing, so the
  bearing keeps growing and the chassis keeps turning — degrades gracefully.
- **rosbridge per-topic registration bug:** the runner now needs `/PWMServo`
  live again — the existing actuation check (`connect_with_actuation_check`)
  already verifies exactly that topic via the tilt wiggle. No new preflight.
- **Coupled oscillation:** the bearing-invariance argument says the loops
  should not fight; if live behavior hunts anyway, the tuning ladder is:
  lower `kp_ang` → lengthen gimbal settle → `--fixed-gimbal`.

## Testing

- Unit tests for the new controller math: bearing fusion reduces to old
  behavior at pan 90; sign conventions; cos scaling; the invariance
  property (moving error from image offset to pan offset leaves `ang`
  unchanged).
- Offline: recorded clip / laptop webcam with `--dry-run` (gimbal commands
  print).
- Live: first run at `--cap-scale 0.5`, A/B against `--fixed-gimbal`.

## Out of scope

- Continuous (non-settle) gimbal control — disproved live in stage 1
  (limit-cycles on the 0.8 s blind loop).
- Open-space full-cap validation (pending task 12b) — separate session.
- Target re-identification after loss, dog mode, dashboard target picker.
