"""Tests for ai.common.video — the latest-frame camera reader.

Uses a real generated video file so the cv2.VideoCapture path is exercised
end to end, exactly as it will be against the robot's MJPEG stream or a
recorded clip.
"""

import time

import cv2
import numpy as np
import pytest

from ai.common.video import VideoSource, frames_differ


@pytest.fixture
def tiny_video(tmp_path):
    """A 10-frame 64x48 video where frame N is filled with gray level N*20."""
    path = str(tmp_path / "tiny.avi")
    writer = cv2.VideoWriter(
        path, cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (64, 48)
    )
    assert writer.isOpened()
    for n in range(10):
        writer.write(np.full((48, 64, 3), n * 20, dtype=np.uint8))
    writer.release()
    return path


class TestVideoSource:
    def test_reads_frames_from_file(self, tiny_video):
        with VideoSource(tiny_video) as src:
            frame = src.read(timeout_s=2.0)
        assert frame is not None
        assert frame.shape == (48, 64, 3)

    def test_returns_none_after_stream_ends(self, tiny_video):
        with VideoSource(tiny_video) as src:
            deadline = time.monotonic() + 5.0
            while src.alive and time.monotonic() < deadline:
                src.read(timeout_s=0.2)
            assert not src.alive
            assert src.read(timeout_s=0.2) is None

    def test_open_failure_raises(self):
        with pytest.raises(RuntimeError):
            VideoSource("Z:/does/not/exist.avi")

    def test_paced_file_still_delivers_all_remaining_frames(self, tiny_video):
        # With pacing slower than the reader, no frame is skipped — needed so
        # recorded clips play back at real-time speed instead of CPU speed.
        with VideoSource(tiny_video, pace_s=0.03) as src:
            levels = []
            while True:
                frame = src.read(timeout_s=1.0)
                if frame is None:
                    break
                levels.append(int(frame[0, 0, 0]))
        # MJPG is lossy; allow a little tolerance on the gray levels.
        assert len(levels) == 10
        assert all(abs(got - n * 20) <= 3 for n, got in enumerate(levels))


class TestFramesDiffer:
    """Actuation self-test primitive: did the camera view actually change
    after a gimbal command? Thresholds chosen from live measurements —
    static-scene MJPEG noise scored 4-8, real gimbal motion scored ~40."""

    def test_identical_frames_do_not_differ(self):
        f = np.random.default_rng(1).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
        assert frames_differ(f, f.copy()) is False

    def test_noise_level_changes_do_not_differ(self):
        rng = np.random.default_rng(2)
        a = rng.integers(100, 140, (720, 1280, 3), dtype=np.uint8)
        b = a.astype(np.int16) + rng.integers(-8, 8, a.shape, dtype=np.int16)
        assert frames_differ(a, b.clip(0, 255).astype(np.uint8)) is False

    def test_shifted_view_differs(self):
        rng = np.random.default_rng(3)
        a = rng.integers(0, 255, (720, 1280, 3), dtype=np.uint8)
        b = np.roll(a, 200, axis=1)  # camera panned: content shifted
        assert frames_differ(a, b) is True
