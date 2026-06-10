import cv2
import numpy as np

from servers.visual_lane_servoing.visualization import create_lane_visualization

_STATE_COLORS = {
    'DRIVE':     (0, 200, 0),
    'STOPPED':   (0, 0, 255),
    'YIELDING':  (0, 255, 255),
    'DUCK_WAIT': (0, 165, 255),
    'TURNING':   (255, 0, 255),
}


def _draw_tags(bgr: np.ndarray, detected_signs: list) -> np.ndarray:
    font = cv2.FONT_HERSHEY_SIMPLEX
    for sign in detected_signs:
        pts   = sign['corners'].astype(np.int32)
        color = (0, 255, 0) if sign['sign_type'] else (150, 150, 150)
        cv2.polylines(bgr, [pts], True, color, 2)

        label = f"{sign['sign_type'] or 'unknown'} id={sign['tag_id']} a={int(sign['area'])}"
        x, y = pts[0]
        cv2.putText(bgr, label, (int(x), max(15, int(y) - 8)), font, 0.45, color, 1, cv2.LINE_AA)
    return bgr


def _state_strip(width: int, debug_info: dict) -> np.ndarray:
    h = 90
    canvas = np.zeros((h, width, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    state     = debug_info.get('state', 'DRIVE')
    remaining = debug_info.get('state_remaining', 0.0)
    color     = _STATE_COLORS.get(state, (200, 200, 200))
    label     = state if remaining <= 0 else f"{state} ({remaining:.1f}s)"
    cv2.putText(canvas, f"STATE: {label}", (10, 22), font, 0.6, color, 2)

    for i, line in enumerate(debug_info.get('event_log', [])[:3]):
        cv2.putText(canvas, line, (10, 44 + i * 16), font, 0.4, (180, 180, 180), 1)

    return canvas


def create_traffic_nav_visualization(
    bgr: np.ndarray,
    debug_info: dict,
    pwm_left: float,
    pwm_right: float,
) -> np.ndarray:
    annotated = _draw_tags(bgr.copy(), debug_info.get('detected_signs', []))
    grid  = create_lane_visualization(annotated, debug_info, pwm_left, pwm_right)
    strip = _state_strip(grid.shape[1], debug_info)
    return np.vstack([grid, strip])
