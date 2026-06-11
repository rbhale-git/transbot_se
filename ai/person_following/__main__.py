"""Person following — stage 2 of the AI roadmap.

Locks onto the largest target-class detection (person by default, dog via
--target-class) and drives the chassis to keep it centered at the follow
distance. Publishes /ai/cmd_vel — the robot-side mux forwards it only while
the dashboard's AI switch is ARMED and the joystick is quiet, clamped to the
AI caps. Lost target => zeros (and the mux + watchdog back that up).
The gimbal tracks the person too (fast, move-and-settle) and the chassis
steers on the total bearing — see the 2026-06-11 spec. --fixed-gimbal
restores the parked-gimbal behavior for A/B comparison.

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
import dataclasses
import sys
import time
from pathlib import Path

import cv2

from ai import config
from ai.common.connect import connect_with_actuation_check
from ai.common.safety import RateLimiter, ServoPacer
from ai.common.video import VideoSource
from ai.face_tracking.tracker import GimbalTracker
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
    p.add_argument("--fixed-gimbal", action="store_true",
                   help="park the gimbal at the follow pose; chassis steers "
                        "on image error alone (linear keeps cos scaling)")
    p.add_argument("--kp-ang", type=float, default=f["kp_ang"])
    p.add_argument("--kp-lin", type=float, default=f["kp_lin"])
    p.add_argument("--height-setpoint", type=float, default=f["height_setpoint"])
    p.add_argument("--angular-sign", type=int, choices=(-1, 1),
                   default=f["angular_sign"])
    p.add_argument("--smoothing", type=float, default=f["smoothing"])
    return p.parse_args(argv)


def draw_overlay(frame, dets, target, tracker, gimbal, lin, ang, armed,
                 dry_run, paused):
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
    status = (f"{mode}  lin {lin:+.2f}  ang {ang:+.2f}  "
              f"pan {gimbal.pan_deg:5.1f} tilt {gimbal.tilt_deg:5.1f}  [{arm_txt}]")
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
    control_gate = RateLimiter(min_interval_s=1.0 / f["control_rate_hz"])
    status_gate = RateLimiter(min_interval_s=1.0 / f["status_rate_hz"])
    pacer = ServoPacer(min_interval_s=0.15)

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

            # Sync: drive the gimbal to the follow pose so tracker state
            # matches reality (the driver does not echo servo positions).
            pacer.send(sink, gimbal.start_commands())

            # fusion_pan tracks the pan pose visible to the current frame
            # (frames lag moves by ~0.8 s; updated at each move, not settle).
            fusion_pan = f["gimbal_follow"]["pan"]
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
                    if not paused and not args.fixed_gimbal:
                        # Gimbal tracks whenever a target is locked, armed or
                        # not — arming gates chassis motion only. On loss it
                        # holds (best relock odds), then recenters to the
                        # follow pose after ~5 s (GimbalTracker built-in).
                        # Aim above bbox center (face/chest): the center of a
                        # close, edge-clipped person never leaves the tilt
                        # deadband, so the gimbal would pan but never tilt.
                        aim = (target.point(0.5, gt["aim_y"])
                               if target is not None else None)
                        pan_before = gimbal.pan_deg
                        moves = gimbal.update(aim, (w, h))
                        if moves:
                            # Frames stay blind to this move for ~0.8 s; fuse
                            # the bearing with the pose the frames DO show.
                            fusion_pan = pan_before
                        pacer.send(sink, moves)
                    if not gimbal.settling:
                        fusion_pan = gimbal.pan_deg
                    lin, ang = controller.update(
                        target, (w, h), now,
                        pan_offset_deg=fusion_pan - f["gimbal_follow"]["pan"])
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
                    draw_overlay(frame, dets, target, tracker, gimbal, lin, ang,
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
