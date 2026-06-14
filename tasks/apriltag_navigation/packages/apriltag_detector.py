from typing import List, NamedTuple, Tuple

import cv2
import numpy as np

_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
_PARAMS = cv2.aruco.DetectorParameters()
_DETECTOR = cv2.aruco.ArucoDetector(_DICT, _PARAMS)


class TagDetection(NamedTuple):
    tag_id: int
    area: float
    center: Tuple[float, float]
    corners: np.ndarray


def detect_tags(bgr_image: np.ndarray) -> List[TagDetection]:
    """Detect Duckietown AprilTags (tag36h11) in a BGR frame."""
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _DETECTOR.detectMarkers(gray)

    if ids is None:
        return []

    detections = []
    for tag_corners, tag_id in zip(corners, ids.flatten()):
        pts = tag_corners.reshape(4, 2)
        area = float(cv2.contourArea(pts))
        center = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
        detections.append(TagDetection(int(tag_id), area, center, pts))

    return detections
