"""Tests for ai.face_tracking.detector — face selection logic and the
YuNet wrapper's contract (model loading is exercised; real-face accuracy is
validated on recorded/live video per the stage-1 plan).
"""

from pathlib import Path

import numpy as np
import pytest

from ai.face_tracking.detector import Detection, YuNetDetector, select_primary

MODEL = Path(__file__).resolve().parents[1] / "models" / "face_detection_yunet_2023mar.onnx"
FACE_IMG = Path(__file__).resolve().parent / "data" / "face.jpg"


def det(x, y, w, h, score=0.9):
    return Detection(x=x, y=y, w=w, h=h, score=score)


class TestSelectPrimary:
    def test_empty_list_returns_none(self):
        assert select_primary([]) is None

    def test_single_detection_is_primary(self):
        d = det(10, 10, 50, 60)
        assert select_primary([d]) is d

    def test_largest_area_wins(self):
        small = det(0, 0, 20, 20)
        big = det(100, 100, 80, 90)
        assert select_primary([small, big, det(5, 5, 30, 30)]) is big


class TestDetectionCenter:
    def test_center_is_bbox_midpoint(self):
        d = det(100, 40, 50, 60)
        assert d.center == (125.0, 70.0)


@pytest.mark.skipif(not MODEL.exists(), reason="YuNet model not downloaded")
class TestYuNetDetector:
    def test_no_faces_on_blank_frame(self):
        detector = YuNetDetector(str(MODEL))
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        assert detector.detect(frame) == []

    def test_handles_changing_frame_sizes(self):
        detector = YuNetDetector(str(MODEL))
        for shape in [(720, 1280, 3), (480, 640, 3)]:
            frame = np.zeros(shape, dtype=np.uint8)
            assert detector.detect(frame) == []

    @pytest.mark.skipif(not FACE_IMG.exists(), reason="sample face image missing")
    def test_detects_real_face_near_known_location(self):
        import cv2

        detector = YuNetDetector(str(MODEL))
        dets = detector.detect(cv2.imread(str(FACE_IMG)))
        assert len(dets) == 1
        cx, cy = dets[0].center
        # Known face center in OpenCV's lena.jpg, generous tolerance.
        assert abs(cx - 281) < 40 and abs(cy - 286) < 40
        assert dets[0].score > 0.8
