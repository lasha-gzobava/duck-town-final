import os
from typing import Dict, Optional

import yaml

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'config', 'apriltag_config.yaml'
))


def _load_config() -> dict:
    try:
        with open(_CONFIG_FILE) as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


_CFG = _load_config()
_SIGN_TAGS: Dict[str, list] = _CFG.get('sign_tags', {})

# tag_id -> sign type, e.g. {0: 'stop', 1: 'yield', ...}
TAG_TO_SIGN: Dict[int, str] = {
    tag_id: sign_type
    for sign_type, tag_ids in _SIGN_TAGS.items()
    for tag_id in tag_ids
}


def classify_tag(tag_id: int) -> Optional[str]:
    """Return the sign type for a detected AprilTag id, or None if unknown."""
    return TAG_TO_SIGN.get(tag_id)
