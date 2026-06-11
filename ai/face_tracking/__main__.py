"""Face tracking behavior — stage 1 of the AI roadmap.

Detects the largest face in the camera image and P-controls the gimbal
(/PWMServo) to keep it centered. Gimbal only — never touches /cmd_vel.

Run modes (test offline first, per the plan):
  python -m ai.face_tracking --source 0 --dry-run          # laptop webcam, prints commands
  python -m ai.face_tracking --source clip.avi --dry-run   # recorded video
  python -m ai.face_tracking --dry-run                     # robot camera, no commands
  python -m ai.face_tracking                               # live: robot camera + gimbal

Keys in the preview window: q/ESC quit, c home the gimbal (tracking pauses
briefly so it stays homed), t toggle tracking on/off.
"""

import argparse
import dataclasses
import sys
import time
from pathlib import Path

import cv2

from ai import config
from ai.common.connect import connect_with_actuation_check
from ai.common.safety import RateLimiter
from ai.common.video import VideoSource
from ai.face_tracking.detector import YuNetDetector, select_primary
from ai.face_tracking.tracker import GimbalTracker

DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "models" / "face_detection_yunet_2023mar.onnx"


class DryRunSink:
    """Prints commands instead of publishing them."""

    def send(self, servo_id, angle):
        print(f"[dry-run] /PWMServo id={servo_id} angle={angle}")

    def close(self):
        pass


class RosSink:
    def __init__(self, url):
        from ai.common.ros_client import RosClient  # import only when needed

        self._client = RosClient(url)
        print(f"connecting to rosbridge at {url} ...")
        self._client.connect()
        print("rosbridge connected")

    def send(self, servo_id, angle):
        self._client.send_pwm_servo(servo_id, angle)

    def disconnect(self):
        """Drop the session but keep the event loop usable for a retry."""
        self._client.disconnect()

    def close(self):
        self._client.close()


def parse_args(argv):
    p = argparse.ArgumentParser(prog="ai.face_tracking", description=__doc__)
    p.add_argument("--profile", choices=sorted(config.PROFILES), default=config.DEFAULT_PROFILE)
    p.add_argument("--source", default=None,
                   help="video override: webcam index, file path, or URL "
                        "(default: the profile's robot stream)")
    p.add_argument("--dry-run", action="store_true",
                   help="print /PWMServo commands instead of publishing")
    p.add_argument("--no-preview", action="store_true")
    p.add_argument("--record", default=None, metavar="OUT.AVI",
                   help="also record the raw stream (for offline tuning)")
    p.add_argument("--model", default=str(DEFAULT_MODEL))
    p.add_argument("--kp-pan", type=float, default=config.GIMBAL_PAN.kp_deg,
                   help="pan full-correction gain, deg per unit error")
    p.add_argument("--kp-tilt", type=float, default=config.GIMBAL_TILT.kp_deg,
                   help="tilt full-correction gain, deg per unit error")
    p.add_argument("--kd", type=float, default=config.TRACKER["kd_deg"])
    p.add_argument("--deadband", type=float, default=config.TRACKER["deadband"])
    p.add_argument("--max-step", type=float, default=config.TRACKER["max_step_deg"])
    p.add_argument("--lost-recenter", type=int, default=config.TRACKER["lost_recenter_after"],
                   metavar="STEPS", help="recenter after this many lost control steps (0=never)")
    p.add_argument("--smoothing", type=float, default=config.TRACKER["smoothing"],
                   help="EMA weight on previous face-center estimate (0=off)")
    p.add_argument("--settle", type=int, default=config.TRACKER["settle_updates"],
                   metavar="UPDATES", help="hold-fire updates after each move "
                   "(0 = continuous stepping instead of move-and-settle)")
    p.add_argument("--rate", type=float, default=config.TRACKER["control_rate_hz"])
    p.add_argument("--pan-sign", type=int, choices=(-1, 1), default=config.GIMBAL_PAN.sign)
    p.add_argument("--tilt-sign", type=int, choices=(-1, 1), default=config.GIMBAL_TILT.sign)
    return p.parse_args(argv)


def draw_overlay(frame, face, tracker, sending, paused):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 24, 1)
    if face is not None:
        x, y, fw, fh = (int(v) for v in (face.x, face.y, face.w, face.h))
        cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
        fcx, fcy = (int(v) for v in face.center)
        cv2.line(frame, (cx, cy), (fcx, fcy), (0, 255, 0), 1)
    if paused:
        state = "PAUSED (t to resume)"
    elif face is not None:
        state = "TRACKING"
    else:
        state = f"NO FACE ({tracker.frames_since_face})"
    status = (f"pan {tracker.pan_deg:5.1f}  tilt {tracker.tilt_deg:5.1f}  "
              f"{state}{'' if sending else '  [DRY RUN]'}")
    cv2.putText(frame, status, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 255), 1, cv2.LINE_AA)


def main(argv=None):
    args = parse_args(argv)
    profile = config.PROFILES[args.profile]

    source = profile["video_url"] if args.source is None else args.source
    if isinstance(source, str) and source.isdigit():
        source = int(source)  # local webcam index

    pan_cfg = dataclasses.replace(config.GIMBAL_PAN, sign=args.pan_sign,
                                  kp_deg=args.kp_pan)
    tilt_cfg = dataclasses.replace(config.GIMBAL_TILT, sign=args.tilt_sign,
                                   kp_deg=args.kp_tilt)
    tracker = GimbalTracker(pan=pan_cfg, tilt=tilt_cfg,
                            kp_deg=config.TRACKER["kp_deg"],
                            kd_deg=args.kd, deadband=args.deadband,
                            max_step_deg=args.max_step,
                            lost_recenter_after=args.lost_recenter,
                            smoothing=args.smoothing,
                            settle_updates=args.settle)
    detector = YuNetDetector(args.model, score_threshold=config.TRACKER["score_threshold"])
    control_gate = RateLimiter(min_interval_s=1.0 / args.rate)

    recorder = None
    sink = None
    print(f"video source: {source}")

    # Recorded files need pacing or OpenCV plays them at CPU speed.
    pace_s = None
    if isinstance(source, str) and Path(source).is_file():
        probe = cv2.VideoCapture(source)
        fps = probe.get(cv2.CAP_PROP_FPS) or 30.0
        probe.release()
        pace_s = 1.0 / fps

    try:
        with VideoSource(source, pace_s=pace_s) as src:
            if args.dry_run:
                sink = DryRunSink()
            else:
                sink = connect_with_actuation_check(
                    lambda: RosSink(profile["rosbridge_url"]), src, tilt_cfg)

            # Sync: drive the gimbal to home so tracker state matches reality.
            for servo_id, angle in tracker.start_commands():
                sink.send(servo_id, angle)

            paused = False
            hold_until = 0.0  # tracking suppressed until this time after 'c'
            while True:
                frame = src.read(timeout_s=1.0)
                if frame is None:
                    if not src.alive:
                        print("stream ended")
                        break
                    print("waiting for frames ...")
                    continue

                h, w = frame.shape[:2]
                scale = config.TRACKER["detect_width"] / w
                small = cv2.resize(frame, None, fx=scale, fy=scale) if scale < 1.0 else frame
                face = select_primary(detector.detect(small))
                if face is not None and small is not frame:
                    face = dataclasses.replace(
                        face, x=face.x / scale, y=face.y / scale,
                        w=face.w / scale, h=face.h / scale)

                # Control runs at a fixed rate; detection/preview run per frame.
                tracking_active = not paused and time.monotonic() >= hold_until
                if tracking_active and control_gate.ready():
                    center = face.center if face is not None else None
                    for servo_id, angle in tracker.update(center, (w, h)):
                        sink.send(servo_id, angle)

                if recorder is None and args.record:
                    recorder = cv2.VideoWriter(
                        args.record, cv2.VideoWriter_fourcc(*"MJPG"), 15.0, (w, h))
                if recorder is not None:
                    recorder.write(frame)

                if not args.no_preview:
                    draw_overlay(frame, face, tracker,
                                 sending=not args.dry_run, paused=paused)
                    cv2.imshow("transbot face tracking", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("t"):
                        paused = not paused
                    if key == ord("c"):
                        for servo_id, angle in tracker.recenter():
                            sink.send(servo_id, angle)
                        # Let it stay home long enough to see, instead of
                        # immediately tracking away again.
                        hold_until = time.monotonic() + 2.0
    finally:
        if recorder is not None:
            recorder.release()
        cv2.destroyAllWindows()
        if sink is not None:
            sink.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
