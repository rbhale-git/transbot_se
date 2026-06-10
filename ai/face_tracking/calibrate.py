"""Gimbal calibration: measure the real control-loop latency and the
degrees-per-error gain of each axis.

Method: with a roughly still face in view, command a known servo delta and
sample the detected face center at frame rate. Time until the face starts
shifting = end-to-end latency (command -> servo -> camera -> stream ->
detection). Total shift once stable = the true full-correction gain.

These two numbers ARE the tracker tuning (see docs/GIMBAL_TRACKING_NOTEBOOK.md):
  - per-axis kp_deg in ai/config.py = measured gain x ~0.85
  - settle_updates = ceil(measured latency x control_rate_hz) + margin

Run (someone must stand in front of the robot, reasonably still):
  python -m ai.face_tracking.calibrate [--profile home] [--delta 8]

Includes the same actuation self-check as the tracker: the robot's rosbridge
can silently drop a whole session (notebook section on the dropped-session
bug), so every connection is verified by wiggling the tilt servo and watching
the camera image actually change.
"""

import argparse
import sys
import time
from pathlib import Path

from ai import config
from ai.common.ros_client import RosClient
from ai.common.video import VideoSource, frames_differ
from ai.face_tracking.detector import YuNetDetector, select_primary

DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "models" / "face_detection_yunet_2023mar.onnx"


def parse_args(argv):
    p = argparse.ArgumentParser(prog="ai.face_tracking.calibrate", description=__doc__)
    p.add_argument("--profile", choices=sorted(config.PROFILES), default=config.DEFAULT_PROFILE)
    p.add_argument("--delta", type=int, default=8, help="commanded degrees per probe")
    p.add_argument("--model", default=str(DEFAULT_MODEL))
    return p.parse_args(argv)


class Calibrator:
    def __init__(self, ros, detector, src, delta):
        self.ros = ros
        self.detector = detector
        self.src = src
        self.delta = delta
        self.pose = {config.GIMBAL_PAN.servo_id: config.GIMBAL_PAN.home_deg,
                     config.GIMBAL_TILT.servo_id: config.GIMBAL_TILT.home_deg}

    def send(self, servo_id, angle):
        angle = max(0, min(180, int(round(angle))))
        self.pose[servo_id] = angle
        self.ros.send_pwm_servo(servo_id, angle)

    def face_center(self, timeout=2.0):
        frame = self.src.read(timeout_s=timeout)
        if frame is None:
            return None, None
        det = select_primary(self.detector.detect(frame))
        return (det.center if det else None), frame.shape[:2]

    def median_center(self, n=5):
        pts, shape = [], None
        for _ in range(n * 4):
            c, s = self.face_center()
            if c:
                pts.append(c)
                shape = s
            if len(pts) >= n:
                break
        if len(pts) < n:
            return None, shape
        xs = sorted(p[0] for p in pts)
        ys = sorted(p[1] for p in pts)
        return (xs[len(xs) // 2], ys[len(ys) // 2]), shape

    def find_face(self):
        """Tilt scan: a close subject's face is often above the home view."""
        tilt_id = config.GIMBAL_TILT.servo_id
        for tilt in (config.GIMBAL_TILT.home_deg, 40, 60, 80, 100):
            self.send(tilt_id, tilt)
            time.sleep(1.2)
            c, _ = self.face_center()
            if c:
                print(f"face found at tilt {tilt}: ({c[0]:.0f},{c[1]:.0f})")
                return True
        return False

    def rough_center(self, iters=3):
        for _ in range(iters):
            c, shape = self.median_center(n=3)
            if c is None:
                return
            h, w = shape
            ex = (c[0] - w / 2) / (w / 2)
            ey = (c[1] - h / 2) / (h / 2)
            if abs(ex) < 0.10 and abs(ey) < 0.10:
                return
            pan, tilt = config.GIMBAL_PAN, config.GIMBAL_TILT
            self.send(pan.servo_id, self.pose[pan.servo_id] + pan.sign * pan.kp_deg * ex)
            self.send(tilt.servo_id, self.pose[tilt.servo_id] + tilt.sign * tilt.kp_deg * ey)
            time.sleep(1.2)

    def probe(self, name, servo_id, axis_idx):
        base, shape = self.median_center(n=5)
        if base is None:
            print(f"--- {name}: face lost, skipping")
            return None, None
        h, w = shape
        start = self.pose[servo_id]
        print(f"--- {name}: baseline=({base[0]:.0f},{base[1]:.0f}), servo {servo_id} at {start}")
        t0 = time.monotonic()
        self.send(servo_id, start + self.delta)
        samples = []
        while time.monotonic() - t0 < 2.5:
            c, _ = self.face_center(timeout=0.5)
            if c:
                samples.append((time.monotonic() - t0, c[axis_idx]))
        self.send(servo_id, start)
        time.sleep(1.2)
        if not samples:
            print("    no samples (face lost during probe)")
            return None, None
        moved = [(t, v) for t, v in samples if abs(v - base[axis_idx]) > 20]
        latency = moved[0][0] if moved else None
        tail = [v for t, v in samples if t > samples[-1][0] - 0.5]
        shift_px = (sum(tail) / len(tail)) - base[axis_idx]
        half = (w / 2.0) if axis_idx == 0 else (h / 2.0)
        err_units = shift_px / half
        gain = abs(self.delta / err_units) if err_units else float("nan")
        lat_s = f"{latency:.2f}" if latency is not None else ">2.5"
        print(f"    latency {lat_s} s | shift {shift_px:+.0f} px ({err_units:+.3f} err)"
              f" | gain {gain:.1f} deg/err-unit | {len(samples)} samples")
        return latency, gain


def connect_checked(url, src, attempts=5):
    """Fresh-session loop with the camera-verified actuation check."""
    tilt = config.GIMBAL_TILT
    for attempt in range(1, attempts + 1):
        ros = RosClient(url)
        ros.connect()
        before = src.read(timeout_s=3.0)
        ros.send_pwm_servo(tilt.servo_id, tilt.home_deg + 25)
        time.sleep(1.8)
        after = src.read(timeout_s=3.0)
        ros.send_pwm_servo(tilt.servo_id, tilt.home_deg)
        time.sleep(1.0)
        if before is not None and after is not None and frames_differ(before, after):
            print(f"actuation check passed (attempt {attempt})")
            return ros
        print(f"actuation check FAILED (attempt {attempt}) - reconnecting fresh")
        ros.disconnect()  # never terminate(): the Twisted reactor can't restart
        time.sleep(2.0)
    return None


def main(argv=None):
    args = parse_args(argv)
    profile = config.PROFILES[args.profile]
    detector = YuNetDetector(args.model, score_threshold=config.TRACKER["score_threshold"])

    with VideoSource(profile["video_url"]) as src:
        ros = connect_checked(profile["rosbridge_url"], src)
        if ros is None:
            print("no working rosbridge session - restart the robot stack:\n"
                  "  ssh jetson@<robot> sudo systemctl restart rosbridge-dashboard.service")
            return 1
        try:
            cal = Calibrator(ros, detector, src, args.delta)
            cal.send(config.GIMBAL_PAN.servo_id, config.GIMBAL_PAN.home_deg)
            time.sleep(0.2)
            cal.send(config.GIMBAL_TILT.servo_id, config.GIMBAL_TILT.home_deg)
            time.sleep(1.5)
            if not cal.find_face():
                print("NO FACE found in tilt scan - stand in front of the robot and rerun")
                return 1
            results = {}
            for name, axis, idx in (("PAN", config.GIMBAL_PAN, 0),
                                    ("TILT", config.GIMBAL_TILT, 1)):
                cal.rough_center()
                results[name] = cal.probe(f"{name} +{args.delta}", axis.servo_id, idx)
        finally:
            ros.close()

    print()
    for name, (lat, gain) in results.items():
        lat_s = f"{lat:.2f}s" if lat else "n/a"
        gain_s = f"{gain:.1f}" if gain else "n/a"
        print(f"SUMMARY {name}: latency={lat_s} gain={gain_s} deg/err-unit")
    print("apply: ai/config.py kp_deg = gain x 0.85; settle_updates >= latency x rate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
