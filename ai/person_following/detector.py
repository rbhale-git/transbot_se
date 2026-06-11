"""YOLO11n object detector via OpenCV DNN.

Chosen in the stage-2 design: nano-size (~10 MB ONNX), 80 COCO classes (so
the follow target is a config choice — person, dog, ...), runs on the laptop
CPU well above the 10 Hz control rate, and the same model serves stages 3/4
(find-and-approach for pick and place). Runtime is cv2.dnn only — same stack
as YuNet, no new dependencies; `ultralytics` was used once, offline, to
export the ONNX.

Decode notes (YOLOv8/11 ONNX, 640x640 input): output is (1, 84, 8400) —
4 box coords (cx,cy,w,h in input pixels) + 80 class scores per anchor, no
objectness term. Frames are letterboxed top-left (pad right/bottom with the
conventional gray 114) so mapping back to frame coords is a single divide.
"""

import cv2
import numpy as np

from ai.common.detection import Detection

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


class YoloDetector:
    def __init__(self, model_path, score_threshold=0.5, nms_threshold=0.45,
                 input_size=640):
        self._net = cv2.dnn.readNetFromONNX(str(model_path))
        self._score = score_threshold
        self._nms = nms_threshold
        self._size = input_size

    def detect(self, frame_bgr):
        """All detections above threshold, in frame coordinates."""
        h, w = frame_bgr.shape[:2]
        scale = min(self._size / w, self._size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        canvas = np.full((self._size, self._size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(frame_bgr, (nw, nh))
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0,
                                     (self._size, self._size),
                                     swapRB=True, crop=False)
        self._net.setInput(blob)
        out = self._net.forward()[0].T          # (8400, 84)
        class_ids = np.argmax(out[:, 4:], axis=1)
        scores = out[np.arange(len(out)), 4 + class_ids]
        keep = scores >= self._score
        out, class_ids, scores = out[keep], class_ids[keep], scores[keep]
        boxes = [[float(cx - bw / 2), float(cy - bh / 2), float(bw), float(bh)]
                 for cx, cy, bw, bh in out[:, :4]]
        idxs = cv2.dnn.NMSBoxes(boxes, [float(s) for s in scores],
                                self._score, self._nms)
        dets = []
        for i in np.asarray(idxs).flatten():
            x, y, bw, bh = boxes[i]
            dets.append(Detection(
                x=x / scale, y=y / scale, w=bw / scale, h=bh / scale,
                score=float(scores[i]), label=COCO_CLASSES[class_ids[i]]))
        return dets
