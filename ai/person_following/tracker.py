"""Lock-at-arm target tracker — stop on loss, no re-identification.

Design decision (spec): appearance re-ID silently follows the wrong person
when it mismatches — the worst failure shape for a follower. This tracker
only associates frame-to-frame by box overlap (IoU); any loss longer than a
short grace makes the robot STOP, and re-acquisition is simply "largest
detection of the target class" (the operator accepts that semantics).

States: SEARCHING (never had a target) / FOLLOWING / LOST. SEARCHING and
LOST behave identically (lock largest when something appears); they are
distinct only so the dashboard/preview can tell "never saw anyone" from
"had someone and lost them".
"""

SEARCHING = "SEARCHING"
FOLLOWING = "FOLLOWING"
LOST = "LOST"


def iou(a, b):
    """Intersection-over-union of two Detections."""
    x1, y1 = max(a.x, b.x), max(a.y, b.y)
    x2 = min(a.x + a.w, b.x + b.w)
    y2 = min(a.y + a.h, b.y + b.h)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = a.area + b.area - inter
    return inter / union if union > 0 else 0.0


class TargetTracker:
    def __init__(self, min_iou=0.2, lost_grace_s=0.5):
        self._min_iou = min_iou
        self._grace = lost_grace_s
        self.state = SEARCHING
        self._box = None
        self._last_seen = None

    def update(self, detections, now):
        """Feed this control step's detections (already class-filtered).

        Returns the target Detection to steer toward, or None (=> the
        controller must command zero motion).
        """
        if self.state == FOLLOWING:
            best, best_iou = None, 0.0
            for d in detections:
                v = iou(d, self._box)
                if v > best_iou:
                    best, best_iou = d, v
            if best is not None and best_iou >= self._min_iou:
                self._box = best
                self._last_seen = now
                return best
            if now - self._last_seen > self._grace:
                self.state = LOST
                self._box = None
            return None

        # SEARCHING or LOST: lock the largest target-class detection.
        if detections:
            self._box = max(detections, key=lambda d: d.area)
            self._last_seen = now
            self.state = FOLLOWING
            return self._box
        return None
