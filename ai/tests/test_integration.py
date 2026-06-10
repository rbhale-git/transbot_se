"""End-to-end pipeline test: video file -> VideoSource -> YuNet -> tracker.

Builds a short clip with a real photographic face pasted off-center on a
1280x720 canvas and checks the tracker walks the gimbal toward it with the
robot's verified direction signs. This is the offline stand-in for the
recorded-video testing the stage-1 plan requires before live runs.
"""

from pathlib import Path

import cv2
import numpy as np
import pytest

from ai.common.video import VideoSource
from ai.face_tracking.detector import YuNetDetector, select_primary
from ai.face_tracking.tracker import AxisConfig, GimbalTracker

MODEL = Path(__file__).resolve().parents[1] / "models" / "face_detection_yunet_2023mar.onnx"
FACE_IMG = Path(__file__).resolve().parent / "data" / "face.jpg"

pytestmark = pytest.mark.skipif(
    not (MODEL.exists() and FACE_IMG.exists()),
    reason="model or sample face image missing",
)


@pytest.fixture
def face_right_clip(tmp_path):
    """15 frames, 1280x720, face pasted right of and below center."""
    face = cv2.resize(cv2.imread(str(FACE_IMG)), (256, 256))
    canvas = np.full((720, 1280, 3), 120, dtype=np.uint8)
    canvas[400:656, 900:1156] = face  # center ~(1028, 528)
    path = str(tmp_path / "face_right.avi")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 15.0, (1280, 720))
    assert writer.isOpened()
    for _ in range(15):
        writer.write(canvas)
    writer.release()
    return path


def test_tracker_walks_gimbal_toward_offset_face(face_right_clip):
    detector = YuNetDetector(str(MODEL))
    tracker = GimbalTracker(
        pan=AxisConfig(servo_id=1, min_deg=0, max_deg=180, home_deg=90, sign=-1),
        tilt=AxisConfig(servo_id=2, min_deg=0, max_deg=180, home_deg=22, sign=-1),
        kp_deg=8.0, deadband=0.06, max_step_deg=4.0,
    )
    commands = []
    with VideoSource(face_right_clip) as src:
        while True:
            frame = src.read(timeout_s=2.0)
            if frame is None:
                break
            face = select_primary(detector.detect(frame))
            assert face is not None, "face must be detected in every frame"
            h, w = frame.shape[:2]
            commands.extend(tracker.update(face.center, (w, h)))

    pan_cmds = [a for sid, a in commands if sid == 1]
    tilt_cmds = [a for sid, a in commands if sid == 2]
    # Face is right of center (positive error) -> pan steps down from 90...
    assert pan_cmds and pan_cmds[0] < 90
    assert pan_cmds == sorted(pan_cmds, reverse=True)
    # ...and below center -> tilt steps down from 22 toward 0 (clamped).
    assert tilt_cmds and tilt_cmds[0] < 22
    assert all(a >= 0 for a in tilt_cmds)
