"""Detector throughput check — the design gate is >= 10 fps on the laptop.

Run: python -m ai.person_following.benchmark
"""

import time
from pathlib import Path

import cv2

from ai.person_following.detector import YoloDetector

MODEL = Path(__file__).resolve().parents[1] / "models" / "yolo11n.onnx"
IMAGE = Path(__file__).resolve().parents[1] / "tests" / "data" / "people.jpg"
RUNS = 30


def main():
    det = YoloDetector(str(MODEL))
    frame = cv2.imread(str(IMAGE))
    det.detect(frame)  # warm-up (first inference pays one-time init)
    t0 = time.perf_counter()
    for _ in range(RUNS):
        det.detect(frame)
    dt = (time.perf_counter() - t0) / RUNS
    print(f"{dt * 1000:.1f} ms/frame  ->  {1 / dt:.1f} fps")
    print("PASS (>= 10 fps)" if 1 / dt >= 10 else "FAIL: below the 10 fps design gate")


if __name__ == "__main__":
    main()
