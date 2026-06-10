"""Latest-frame video reader.

A background thread drains cv2.VideoCapture continuously so read() always
returns the freshest frame — critical against the robot's live MJPEG stream,
where letting frames queue adds seconds of control latency. Works the same
on a recorded file or local webcam index, which is how behaviors are tested
before live runs.
"""

import threading
import time

import cv2
import numpy as np


def frames_differ(a, b, threshold=15.0):
    """True if two frames show a genuinely different view.

    Compares mean absolute difference on downscaled grayscale. Calibrated
    live on the robot: static-scene MJPEG noise scores 4-8, real gimbal
    motion scores ~40. Used to verify a gimbal command actually moved the
    camera — the only end-to-end actuation feedback this robot offers.
    """
    small_a = cv2.cvtColor(cv2.resize(a, (160, 90)), cv2.COLOR_BGR2GRAY)
    small_b = cv2.cvtColor(cv2.resize(b, (160, 90)), cv2.COLOR_BGR2GRAY)
    return bool(np.mean(cv2.absdiff(small_a, small_b)) > threshold)


class VideoSource:
    """Open with a URL, file path, or integer webcam index.

    `pace_s` throttles the reader thread between frames — set it when the
    source is a recorded file, which OpenCV otherwise plays at CPU speed.
    Live sources (stream/webcam) pace themselves; leave it None.
    """

    def __init__(self, source, pace_s=None):
        self._cap = cv2.VideoCapture(source)
        self._pace_s = pace_s
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open video source: {source!r}")
        self._lock = threading.Condition()
        self._frame = None
        self._seq = 0
        self._alive = True
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self):
        while self._alive:
            ok, frame = self._cap.read()
            with self._lock:
                if not ok:
                    self._alive = False
                else:
                    self._frame = frame
                    self._seq += 1
                self._lock.notify_all()
            if ok and self._pace_s:
                time.sleep(self._pace_s)
        self._cap.release()

    @property
    def alive(self):
        """False once the stream has ended or the source was closed."""
        return self._alive

    def read(self, timeout_s=1.0):
        """Block until a frame newer than the last returned one arrives.

        Returns None on timeout or end of stream.
        """
        with self._lock:
            seen = getattr(self, "_returned_seq", 0)
            if self._seq == seen and self._alive:
                self._lock.wait(timeout=timeout_s)
            if self._seq == seen:
                return None
            self._returned_seq = self._seq
            return self._frame

    def close(self):
        self._alive = False
        self._thread.join(timeout=2.0)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
