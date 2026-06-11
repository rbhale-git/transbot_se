"""Tests for ai.face_tracking.tracker — the pure P-control gimbal tracker.

The tracker is deliberately hardware-free: it maps (face center, frame size)
to a list of (servo_id, int angle) commands. All safety behavior lives here:
deadband, per-update step cap, range clamping, and hold-on-lost-target.
"""

from ai.face_tracking.tracker import AxisConfig, GimbalTracker

FRAME = (1280, 720)
CENTER = (640.0, 360.0)


def make_tracker(**overrides):
    """Symmetric test rig: both axes home at 90, full 0-180 range.

    sign=-1 matches the real robot (positive servo angle moves the view
    away from positive image error on both axes — see dashboard config).
    """
    kwargs = dict(
        pan=AxisConfig(servo_id=1, min_deg=0, max_deg=180, home_deg=90, sign=-1),
        tilt=AxisConfig(servo_id=2, min_deg=0, max_deg=180, home_deg=90, sign=-1),
        kp_deg=10.0,
        deadband=0.05,
        max_step_deg=5.0,
    )
    kwargs.update(overrides)
    return GimbalTracker(**kwargs)


class TestStart:
    def test_start_commands_home_both_axes(self):
        t = make_tracker()
        assert t.start_commands() == [(1, 90), (2, 90)]


class TestProportionalResponse:
    def test_centered_face_sends_nothing(self):
        t = make_tracker()
        assert t.update(CENTER, FRAME) == []

    def test_face_right_of_center_steps_pan_negative(self):
        t = make_tracker()
        # ex = (768-640)/640 = +0.2 -> delta = sign * kp * ex = -2 deg
        assert t.update((768.0, 360.0), FRAME) == [(1, 88)]

    def test_face_left_of_center_steps_pan_positive(self):
        t = make_tracker()
        assert t.update((512.0, 360.0), FRAME) == [(1, 92)]

    def test_face_below_center_steps_tilt_negative(self):
        t = make_tracker()
        # ey = (432-360)/360 = +0.2 -> delta = -2 deg
        assert t.update((640.0, 432.0), FRAME) == [(2, 88)]

    def test_diagonal_error_commands_both_axes(self):
        t = make_tracker()
        cmds = dict(t.update((768.0, 432.0), FRAME))
        assert cmds == {1: 88, 2: 88}

    def test_error_within_deadband_sends_nothing(self):
        t = make_tracker()
        # ex = 0.03 < deadband 0.05
        assert t.update((659.2, 360.0), FRAME) == []

    def test_steps_accumulate_across_updates(self):
        t = make_tracker()
        assert t.update((768.0, 360.0), FRAME) == [(1, 88)]
        assert t.update((768.0, 360.0), FRAME) == [(1, 86)]


class TestSafetyEnvelope:
    def test_large_error_is_capped_to_max_step(self):
        t = make_tracker(kp_deg=30.0)
        # ex = 1.0 -> raw delta -30, capped at -5
        assert t.update((1280.0, 360.0), FRAME) == [(1, 85)]

    def test_angle_clamps_to_axis_min(self):
        t = make_tracker(
            pan=AxisConfig(servo_id=1, min_deg=0, max_deg=180, home_deg=2, sign=-1),
            kp_deg=30.0,
        )
        # raw delta -5 (capped) would take pan to -3; clamps to 0 instead.
        assert t.update((1280.0, 360.0), FRAME) == [(1, 0)]
        assert t.pan_deg == 0.0

    def test_no_command_when_pinned_at_limit(self):
        t = make_tracker(
            pan=AxisConfig(servo_id=1, min_deg=0, max_deg=180, home_deg=2, sign=-1),
            kp_deg=30.0,
        )
        assert t.update((1280.0, 360.0), FRAME) == [(1, 0)]
        # Still pushing past the limit: angle can't move, so stay silent.
        assert t.update((1280.0, 360.0), FRAME) == []

    def test_subdegree_changes_accumulate_until_a_whole_degree(self):
        t = make_tracker(deadband=0.01)
        # ex = 0.04 -> delta -0.4 deg/update: no command until the rounded
        # angle actually changes.
        face = (665.6, 360.0)
        assert t.update(face, FRAME) == []           # 89.6 -> rounds to 90
        assert t.update(face, FRAME) == [(1, 89)]    # 89.2 -> rounds to 89


class TestPerAxisGain:
    def test_axis_kp_overrides_global(self):
        # Measured on the robot 2026-06-10: pan needs ~48 deg per unit
        # error, tilt ~23 — one global gain cannot serve both.
        t = make_tracker(
            pan=AxisConfig(servo_id=1, min_deg=0, max_deg=180, home_deg=90,
                           sign=-1, kp_deg=40.0),
            kp_deg=10.0, max_step_deg=30.0,
        )
        # ex = ey = 0.5: pan uses its own 40 -> -20; tilt falls back to 10 -> -5.
        assert dict(t.update((960.0, 540.0), FRAME)) == {1: 70, 2: 85}


class TestMoveAndSettle:
    """Latency-robust mode: command the full correction once, then hold
    fire for settle_updates so the laggy stream can catch up — measured
    live, continuous control just limit-cycles around the face."""

    def test_commands_full_correction_in_one_move(self):
        t = make_tracker(kp_deg=20.0, max_step_deg=12.0, settle_updates=4)
        # ex = 0.5 -> one decisive move of -10, not a 2-degree nibble.
        assert t.update((960.0, 360.0), FRAME) == [(1, 80)]

    def test_holds_fire_while_settling_then_resumes(self):
        t = make_tracker(settle_updates=3)
        assert t.update((960.0, 360.0), FRAME) != []
        for _ in range(3):
            assert t.update((960.0, 360.0), FRAME) == []
        assert t.update((960.0, 360.0), FRAME) != []

    def test_deadband_updates_do_not_trigger_settle(self):
        t = make_tracker(settle_updates=5)
        assert t.update(CENTER, FRAME) == []
        # Still ready to move immediately when the face drifts off.
        assert t.update((960.0, 360.0), FRAME) != []

    def test_recenter_starts_settle_window(self):
        # recenter() IS a move: the stream still shows the old pose for ~0.8 s,
        # so the first post-recenter update must be held (not executed).
        t = make_tracker(settle_updates=2)
        t.update((960.0, 360.0), FRAME)  # move to start a settle
        t.recenter()                      # another move — must start its own settle
        assert t.update((960.0, 360.0), FRAME) == []  # still settling

    def test_settle_mode_measures_raw_center_after_each_move(self):
        # The pre-move smoothing history describes where the face WAS in the
        # old camera pose; blending it in after the camera moved would
        # corrupt the next measurement.
        t = make_tracker(settle_updates=1, smoothing=0.9, kp_deg=10.0)
        t.update((960.0, 360.0), FRAME)   # move; history must be dropped
        t.update((960.0, 360.0), FRAME)   # settling
        # ex measured raw = 0.2 -> -2 (heavy smoothing would give ~-4.4)
        assert t.update((768.0, 360.0), FRAME) == [(1, 83)]


class TestCenterSmoothing:
    def test_first_sighting_uses_raw_center(self):
        t = make_tracker(smoothing=0.5)
        assert t.update((768.0, 360.0), FRAME) == [(1, 88)]  # ex=.2 -> -2

    def test_center_estimate_is_exponentially_smoothed(self):
        t = make_tracker(smoothing=0.5)
        t.update((768.0, 360.0), FRAME)
        # Raw center jumps back to frame center; smoothed = .5*768+.5*640
        # = 704 -> ex=.1 -> step -1, instead of no step at all.
        assert t.update((640.0, 360.0), FRAME) == [(1, 87)]

    def test_smoothing_resets_after_lost_target(self):
        t = make_tracker(smoothing=0.9)
        t.update((768.0, 360.0), FRAME)  # pan -> 88, history at 768
        t.update(None, FRAME)
        # Reacquired at 512: raw ex=-0.2 -> +2 -> 90. With stale history the
        # estimate would still sit right of center and step down instead.
        assert t.update((512.0, 360.0), FRAME) == [(1, 90)]


class TestDerivativeDamping:
    def test_static_error_behaves_like_pure_p(self):
        t = make_tracker(kd_deg=5.0)
        t.update((768.0, 360.0), FRAME)  # ex=0.2, first sighting: derr=0
        # Second update, same error: derr=0 again -> plain P step of -2.
        assert t.update((768.0, 360.0), FRAME) == [(1, 86)]

    def test_shrinking_error_damps_the_step(self):
        t = make_tracker(kd_deg=5.0)
        t.update((768.0, 360.0), FRAME)        # ex=0.2 -> 88
        # Error shrank to 0.1: derr=-0.1, step = -(10*0.1 + 5*-0.1) = -0.5
        # instead of the undamped -1.0 -> 87.5 rounds to 88, so silent.
        assert t.update((704.0, 360.0), FRAME) == []

    def test_damping_resets_after_lost_target(self):
        t = make_tracker(kd_deg=100.0)
        t.update((768.0, 360.0), FRAME)
        t.update(None, FRAME)
        # Reacquired: stale prev error must not produce a huge D kick.
        assert t.update((768.0, 360.0), FRAME) == [(1, 86)]


class TestLostRecenter:
    def test_recenters_once_after_timeout_then_stays_silent(self):
        t = make_tracker(lost_recenter_after=3)
        t.update((1280.0, 360.0), FRAME)  # drift pan off home
        assert t.update(None, FRAME) == []
        assert t.update(None, FRAME) == []
        assert t.update(None, FRAME) == [(1, 90), (2, 90)]  # fires once
        assert t.pan_deg == 90.0
        assert t.update(None, FRAME) == []  # does not repeat

    def test_disabled_by_default(self):
        t = make_tracker()
        t.update((1280.0, 360.0), FRAME)
        for _ in range(500):
            assert t.update(None, FRAME) == []


class TestRecenter:
    def test_recenter_resets_state_and_commands_home(self):
        t = make_tracker()
        t.update((1280.0, 720.0), FRAME)  # drift both axes off home
        assert t.recenter() == [(1, 90), (2, 90)]
        assert t.pan_deg == 90.0
        assert t.tilt_deg == 90.0


class TestLostTarget:
    def test_no_face_sends_nothing_and_holds_state(self):
        t = make_tracker()
        t.update((768.0, 360.0), FRAME)
        assert t.update(None, FRAME) == []
        assert t.pan_deg == 88.0

    def test_frames_since_face_counts_and_resets(self):
        t = make_tracker()
        t.update(None, FRAME)
        t.update(None, FRAME)
        assert t.frames_since_face == 2
        t.update(CENTER, FRAME)
        assert t.frames_since_face == 0


class TestRecenterSettle:
    """recenter() starts a settle window so the first post-recenter update
    does not measure the face against the old gimbal pose."""

    def test_recenter_with_settle_holds_fire_on_immediate_update(self):
        # After recenter, the stream still shows the pre-recenter pose; an
        # immediate off-center update must return [] (hold fire).
        t = make_tracker(settle_updates=2)
        t.recenter()
        assert t.update((960.0, 360.0), FRAME) == []

    def test_recenter_with_settle_resumes_after_settle_updates(self):
        # After settle_updates ticks the tracker must move again.
        t = make_tracker(settle_updates=2)
        t.recenter()
        assert t.update((960.0, 360.0), FRAME) == []   # settling (1 of 2)
        assert t.update((960.0, 360.0), FRAME) == []   # settling (2 of 2)
        assert t.update((960.0, 360.0), FRAME) != []   # settled — moves

    def test_settling_property_reflects_settle_window(self):
        # settle_updates=2: recenter sets _settle_remaining=2; each update()
        # call with a face decrements it, so it hits 0 after 2 updates.
        t = make_tracker(settle_updates=2)
        assert not t.settling
        t.recenter()
        assert t.settling           # 2 remaining
        t.update((960.0, 360.0), FRAME)
        assert t.settling           # 1 remaining
        t.update((960.0, 360.0), FRAME)
        assert not t.settling       # 0 remaining — settled
        assert t.update((960.0, 360.0), FRAME) != []   # actually moves now

    def test_settle_window_survives_lost_frames(self):
        # recenter() fires DURING a lost streak; the settle window must not
        # be wiped by subsequent lost-target ticks before reacquisition.
        t = make_tracker(settle_updates=2, lost_recenter_after=1)
        # First update: face visible so it moves (sets settle)
        t.update((960.0, 360.0), FRAME)
        # Two lost ticks: first one triggers recenter (lost_recenter_after=1),
        # starting a fresh settle window; second one must not clear it.
        t.update(None, FRAME)   # triggers recenter -> settle_remaining = 2
        assert t.settling
        t.update(None, FRAME)   # lost again — settle must survive
        assert t.settling
        # Reacquire: still inside the settle window.
        assert t.update((960.0, 360.0), FRAME) == []
