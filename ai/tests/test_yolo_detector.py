"""YoloDetector tests against real images (model + stills are in the repo)."""

from pathlib import Path

import cv2
import pytest

from ai.person_following.detector import YoloDetector, COCO_CLASSES

MODEL = Path(__file__).resolve().parents[1] / "models" / "yolo11n.onnx"
DATA = Path(__file__).parent / "data"


@pytest.fixture(scope="module")
def detector():
    return YoloDetector(str(MODEL))


def test_coco_has_the_classes_we_follow():
    assert "person" in COCO_CLASSES
    assert "dog" in COCO_CLASSES
    assert len(COCO_CLASSES) == 80


def test_detects_people(detector):
    img = cv2.imread(str(DATA / "people.jpg"))
    dets = [d for d in detector.detect(img) if d.label == "person"]
    assert len(dets) >= 2          # scene contains 4; require 2 to be robust
    h, w = img.shape[:2]
    for d in dets:
        assert d.score >= 0.5
        assert -5 <= d.x <= w and -5 <= d.y <= h   # boxes land on the image
        assert 0 < d.w <= w and 0 < d.h <= h


def test_detects_dog(detector):
    img = cv2.imread(str(DATA / "dog.jpg"))
    labels = [d.label for d in detector.detect(img)]
    assert "dog" in labels


def test_no_person_in_dog_image(detector):
    img = cv2.imread(str(DATA / "dog.jpg"))
    assert [d for d in detector.detect(img) if d.label == "person"] == []
