"""Shared detection bbox type for every AI behavior.

Originally lived in ai/face_tracking/detector.py; promoted here when stage 2
added a second detector (YOLO). `label` is the class name for multi-class
detectors; single-class detectors (YuNet faces) leave it empty.
"""

from dataclasses import dataclass


@dataclass
class Detection:
    x: float
    y: float
    w: float
    h: float
    score: float
    label: str = ""

    @property
    def area(self):
        return self.w * self.h

    @property
    def center(self):
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)

    def point(self, fx, fy):
        """Point at (fx, fy) fractions of the bbox from its top-left.

        A person's bbox center sits mid-frame even when the box clips at the
        frame edges, so center-tracking never tilts; aiming at fy~0.2 keeps
        the face/chest framed instead.
        """
        return (self.x + fx * self.w, self.y + fy * self.h)


def select_largest(detections):
    """The detection to act on: largest bbox (closest), or None."""
    if not detections:
        return None
    return max(detections, key=lambda d: d.area)
