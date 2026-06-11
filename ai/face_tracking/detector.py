"""YuNet face detector wrapper.

YuNet (cv2.FaceDetectorYN) was chosen in the stage-1 plan: it ships with
OpenCV, runs fast on CPU, and needs only a ~230 KB ONNX model. The wrapper
normalizes output to a list of Detection bboxes; select_primary() picks the
face to track (largest, i.e. closest).
"""

import cv2

from ai.common.detection import Detection, select_largest

# Backwards-compatible alias: stage-1 code and tests use select_primary.
select_primary = select_largest


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
