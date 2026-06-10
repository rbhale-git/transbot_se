"""YuNet face detector wrapper.

YuNet (cv2.FaceDetectorYN) was chosen in the stage-1 plan: it ships with
OpenCV, runs fast on CPU, and needs only a ~230 KB ONNX model. The wrapper
normalizes output to a list of Detection bboxes; select_primary() picks the
face to track (largest, i.e. closest).
"""

from dataclasses import dataclass

import cv2


@dataclass
class Detection:
    x: float
    y: float
    w: float
    h: float
    score: float

    @property
    def area(self):
        return self.w * self.h

    @property
    def center(self):
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)


def select_primary(detections):
    """The face to track: largest bbox (closest to the camera), or None."""
    if not detections:
        return None
    return max(detections, key=lambda d: d.area)


class YuNetDetector:
    def __init__(self, model_path, score_threshold=0.7):
        self._net = cv2.FaceDetectorYN.create(
            model_path, "", (320, 320), score_threshold=score_threshold
        )
        self._input_size = None

    def detect(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        if self._input_size != (w, h):
            self._net.setInputSize((w, h))
            self._input_size = (w, h)
        _, faces = self._net.detect(frame_bgr)
        if faces is None:
            return []
        return [
            Detection(x=float(f[0]), y=float(f[1]), w=float(f[2]), h=float(f[3]),
                      score=float(f[14]))
            for f in faces
        ]
