import os
import random
import time
from collections import deque
from typing import Tuple

import cv2
import numpy as np
import yaml

from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
from tasks.apriltag_navigation.packages import apriltag_detector, sign_rules

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'config', 'apriltag_config.yaml'
))

# Signs that indicate which turn(s) are possible at an upcoming intersection.
_TURN_SIGN_TYPES = ('one_way_left', 'one_way_right', 'no_entry')


class TrafficNavigationAgent:
    """Lane following (via LaneServoingAgent) + AprilTag traffic-sign reactions."""

    def __init__(self, config_path: str = None):
        path = config_path or _CONFIG_FILE
        try:
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            cfg = {}

        self.stop_duration_s               = cfg.get('stop_duration_s', 4.0)
        self.stop_trigger_area             = cfg.get('stop_trigger_area', 3500)
        self.yield_slowdown_factor         = cfg.get('yield_slowdown_factor', 0.5)
        self.yield_duration_s              = cfg.get('yield_duration_s', 1.5)
        self.yield_trigger_area            = cfg.get('yield_trigger_area', 2500)
        self.duck_crossing_slowdown_factor = cfg.get('duck_crossing_slowdown_factor', 0.4)
        self.duck_crossing_trigger_area    = cfg.get('duck_crossing_trigger_area', 2000)
        self.duck_stop_area                = cfg.get('duck_stop_area', 4000)
        self.sign_cooldown_s               = cfg.get('sign_cooldown_s', 6.0)
        self.turn_trigger_area             = cfg.get('turn_trigger_area', 3000)
        self.turn_duration_s               = cfg.get('turn_duration_s', 1.0)
        self.turn_speed                    = cfg.get('turn_speed', 0.2)
        self.turn_bias                     = cfg.get('turn_bias', 0.08)

        self.lane_agent = LaneServoingAgent()

        self.state          = 'DRIVE'  # DRIVE | STOPPED | YIELDING | DUCK_WAIT | TURNING
        self._state_until   = 0.0
        self._turn_direction = None  # 'left' | 'right', set while TURNING
        self._cooldowns     = {}  # tag_id -> timestamp until which re-triggering is suppressed
        self._visible_tags  = set()  # tag ids seen in the previous frame
        self.event_log      = deque(maxlen=20)
        self._duck_agent    = None  # lazily created ObjectDetectionAgent

        self.last_debug_info = {}

    @property
    def frame_count(self) -> int:
        return self.lane_agent.frame_count

    def _log(self, message: str) -> None:
        line = f"[{time.strftime('%H:%M:%S')}] {message}"
        self.event_log.appendleft(line)
        print(f"[AprilTagNav] {line}")

    def _get_duck_agent(self):
        if self._duck_agent is None:
            from tasks.object_detection.packages.agent import ObjectDetectionAgent
            self._duck_agent = ObjectDetectionAgent()
        return self._duck_agent

    def _duck_ahead(self, image_rgb: np.ndarray) -> bool:
        agent = self._get_duck_agent()
        if not agent.model_loaded:
            return False

        detections = agent.detect(image_rgb)
        if not detections:
            return False

        for bbox, _score, cls_id in detections:
            if cls_id != 0:  # duckie
                continue
            x1, y1, x2, y2 = bbox
            area = (x2 - x1) * (y2 - y1)
            if area >= self.duck_stop_area:
                return True

        return False

    def reset(self) -> None:
        self.lane_agent._prev_error = 0.0
        self.state           = 'DRIVE'
        self._state_until    = 0.0
        self._turn_direction = None
        self._cooldowns.clear()
        self._visible_tags.clear()

    def compute_commands(self, image: np.ndarray) -> Tuple[float, float]:
        left, right = self.lane_agent.compute_commands(image)

        bgr  = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        tags = apriltag_detector.detect_tags(bgr)
        # Closest (largest-area) tag first, so when two signs are visible at
        # once (e.g. one_way_left and one_way_right placed close together)
        # the nearer sign wins instead of whichever cv2.aruco happened to
        # return first.
        tags = sorted(tags, key=lambda t: t.area, reverse=True)
        now  = time.time()

        detected_signs = []
        current_tag_ids = set()
        for tag in tags:
            sign_type = sign_rules.classify_tag(tag.tag_id)
            detected_signs.append({
                'tag_id': tag.tag_id,
                'sign_type': sign_type,
                'area': tag.area,
                'center': tag.center,
                'corners': tag.corners,
            })

            current_tag_ids.add(tag.tag_id)
            if tag.tag_id not in self._visible_tags:
                label = sign_type.replace('_', ' ').upper() if sign_type else 'unrecognized'
                self._log(f"Tag id={tag.tag_id} ({label}) detected, area={tag.area:.0f}")

            if sign_type is None:
                continue

            on_cooldown = self._cooldowns.get(tag.tag_id, 0.0) > now

            if on_cooldown or self.state != 'DRIVE':
                continue

            if sign_type == 'stop' and tag.area >= self.stop_trigger_area:
                self.state        = 'STOPPED'
                self._state_until = now + self.stop_duration_s
                self._cooldowns[tag.tag_id] = now + self.sign_cooldown_s
                self._log(f"STOP sign (id={tag.tag_id}) -> stopping for {self.stop_duration_s:.0f}s")

            elif sign_type == 'yield' and tag.area >= self.yield_trigger_area:
                self.state        = 'YIELDING'
                self._state_until = now + self.yield_duration_s
                self._cooldowns[tag.tag_id] = now + self.sign_cooldown_s
                self._log(f"YIELD sign (id={tag.tag_id}) -> slowing down")

            elif sign_type in ('pedestrian', 'duck_crossing') and tag.area >= self.duck_crossing_trigger_area:
                self.state = 'DUCK_WAIT'
                self._cooldowns[tag.tag_id] = now + self.sign_cooldown_s
                self._log(f"{sign_type.replace('_', ' ').upper()} sign (id={tag.tag_id}) -> watching for duckies")

            elif sign_type in _TURN_SIGN_TYPES and tag.area >= self.turn_trigger_area:
                # Mandatory directions are signed explicitly via one_way_left /
                # one_way_right. If only one of those is present (alone, or
                # together with no_entry blocking straight), that's the only
                # way to go -> take it directly. Otherwise (e.g. a bare
                # no_entry, or both one_way signs together) more than one way
                # is open -> pick randomly among them.
                mandatory = set()
                if any(d['sign_type'] == 'one_way_left' for d in detected_signs):
                    mandatory.add('left')
                if any(d['sign_type'] == 'one_way_right' for d in detected_signs):
                    mandatory.add('right')

                available = mandatory if mandatory else {'left', 'right'}
                direction = next(iter(available)) if len(available) == 1 else random.choice(sorted(available))

                self.state           = 'TURNING'
                self._state_until    = now + self.turn_duration_s
                self._turn_direction = direction
                for d in detected_signs:
                    if d['sign_type'] in _TURN_SIGN_TYPES:
                        self._cooldowns[d['tag_id']] = now + self.sign_cooldown_s

                if len(available) == 1:
                    self._log(f"{sign_type.replace('_', ' ').upper()} sign (id={tag.tag_id}) -> only one way possible, turning {direction}")
                else:
                    self._log(f"{sign_type.replace('_', ' ').upper()} sign (id={tag.tag_id}) -> {len(available)} ways possible, randomly turning {direction}")

        state_remaining = 0.0

        if self.state == 'STOPPED':
            state_remaining = max(0.0, self._state_until - now)
            if now >= self._state_until:
                self.state = 'DRIVE'
            else:
                left, right = 0.0, 0.0

        elif self.state == 'YIELDING':
            state_remaining = max(0.0, self._state_until - now)
            if now >= self._state_until:
                self.state = 'DRIVE'
            else:
                left  *= self.yield_slowdown_factor
                right *= self.yield_slowdown_factor

        elif self.state == 'TURNING':
            state_remaining = max(0.0, self._state_until - now)
            if now >= self._state_until:
                self.state           = 'DRIVE'
                self._turn_direction = None
            else:
                if self._turn_direction == 'left':
                    left, right = self.turn_speed - self.turn_bias, self.turn_speed + self.turn_bias
                else:
                    left, right = self.turn_speed + self.turn_bias, self.turn_speed - self.turn_bias
                left  = float(np.clip(left, 0.0, 1.0))
                right = float(np.clip(right, 0.0, 1.0))

        elif self.state == 'DUCK_WAIT':
            if self._duck_ahead(image):
                left, right = 0.0, 0.0
            else:
                left  *= self.duck_crossing_slowdown_factor
                right *= self.duck_crossing_slowdown_factor

                still_close = any(
                    d['sign_type'] in ('pedestrian', 'duck_crossing')
                    and d['area'] >= self.duck_crossing_trigger_area
                    for d in detected_signs
                )
                if not still_close:
                    self.state = 'DRIVE'

        self._visible_tags = current_tag_ids

        self.last_debug_info = dict(self.lane_agent.last_debug_info)
        self.last_debug_info.update({
            'detected_signs':  detected_signs,
            'state':           self.state,
            'state_remaining': state_remaining,
            'event_log':       list(self.event_log),
        })

        return left, right
