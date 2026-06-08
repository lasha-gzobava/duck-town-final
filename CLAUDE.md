# Duckiebot Navigation Platform — Codebase Reference

## Project Overview

Hybrid robotics education platform for the **Duckiebot DB21J**. Supports both real robot deployment and a **Godot 4.6** simulation. Students work through progressive tasks (Braitenberg → lane servoing → modcon → object detection → navigation) using Python/Flask servers that expose a browser-based dashboard with live camera feed and real-time parameter tuning.

**Stack:** Python 3 + Flask · OpenCV · ONNX Runtime · Godot 4.6 (GDScript) · Jupyter notebooks · PyYAML · PySerial

---

## Repository Structure

```
.
├── launch.py                  # Main entry point — sim or robot deploy
├── requirements.txt           # Python dependencies
├── config/                    # Per-task YAML configs (gains, HSV bounds)
├── tasks/                     # Student-facing packages + Jupyter notebooks
├── servers/                   # Flask web servers, one per task
├── duckiebot/                 # Hardware drivers (camera, wheels, encoder, LEDs, HAT)
├── GodotSimulation/           # Godot 4.6 project (scenes, scripts, models)
├── launcher/                  # Port discovery + config loading utilities
└── docs/                      # MAP_MAKER.md — guide for building Godot road maps
```

### `tasks/`
Each sub-directory is a student task with `packages/` (Python logic) and `notebooks/` (Jupyter theory):
- `braitenberg/` — reactive vehicle using HSV color blobs
- `introduction/` — basic robot control intro
- `modcon/` — model-based control / wheel calibration (PID on encoder feedback)
- `object_detection/` — ONNX inference + YOLO-style detection
- `visual_lane_servoing/` — **primary focus, see deep-dive below**
- `project/` — full navigation task (Dijkstra pathfinding over road map)
- `CollisionStuff/` — collision detection helpers

### `servers/`
One Flask server per task with `virtual_server.py` (simulation) and `real_server.py` (robot):
- `templates/` — base Flask class + Jinja HTML/JS templates per task
- `common.py` — `make_frame_generator`, `shutdown_cleanup`, `suppress_http_logs`

### `config/`
| File | Purpose |
|------|---------|
| `lane_servoing_config.yaml` | PD gains, speed, curve params |
| `lane_servoing_hsv_config.yaml` | HSV color bounds for yellow/white lines |
| `braitenberg_config.yaml` | Braitenberg HSV + speed |
| `modcon_config.yaml` | PID gains for encoder control |
| `object_detection_config.yaml` | Detection confidence thresholds |
| `project_config.yaml` | Navigation task settings |

### `duckiebot/`
Hardware drivers — each has a real variant and a `godot_*` variant for simulation:
- `camera_driver/` — `GodotCameraDriver` (TCP socket from Godot), `RealCameraDriver`
- `wheel_driver/` — `GodotWheelsDriver`, `WheelsPWMDriver`, `WheelPWMConfiguration`
- `encoder_driver/`, `led_driver/`, `button_driver/`, `hat_driver/`

### `GodotSimulation/ducky-bot/`
- `project.godot` — Godot project root
- `scenes/` — road maps, duckiebot body, intersection tiles
- `scripts/` — GDScript (26 files): `MapLoader.gd`, `MapData.gd`, camera/wheel socket bridges, duck AI
- `models/` — textures for road surfaces (straight, curve)
- `addons/` — simplegrasstextured add-on

---

## Launch Commands

```bash
# Simulation
python launch.py --sim --task visual_lane_servoing

# Robot deployment
python launch.py --run --bot <hostname> --task visual_lane_servoing

# Other tasks
python launch.py --sim --task braitenberg
python launch.py --sim --task modcon
python launch.py --sim --task object_detection
python launch.py --sim --task project
```

The launcher auto-downloads Godot if missing, starts the Godot simulation, then starts the Flask server. Web UI served at `http://localhost:5000` (or next available port).

---

## Visual Lane Servoing — Deep Dive

### File Map

```
tasks/visual_lane_servoing/
├── packages/
│   ├── agent.py                    # LaneServoingAgent — main control class
│   ├── visual_servoing_activity.py # Lane detection (student-editable)
│   ├── cuvrve_behavior.py          # Curve detection
│   └── __init__.py
└── notebooks/05-Visual-Servoing/
    └── visual_servoing_activity.ipynb

servers/visual_lane_servoing/
├── virtual_server.py               # Flask app for simulation
└── visualization.py                # 4-panel debug display

servers/templates/
└── lane_servoing.py                # HTML/JS UI template

config/
├── lane_servoing_config.yaml       # Control parameters
└── lane_servoing_hsv_config.yaml   # HSV color bounds

GodotSimulation/ducky-bot/scenes/maps/
└── lane_follower.tscn              # Simulation track
```

---

### Control Loop Data Flow

```
Godot Engine
  └── camera socket (TCP :5001) → GodotCameraDriver.get_image() → RGB frame
        └── virtual_server.visualize(frame)
              └── agent.compute_commands(frame)
                    ├── cv2.cvtColor(RGB → BGR)
                    ├── visual_servoing_activity.detect_lane_markings(BGR)
                    │     ├── BGR → HSV color threshold → mask_yellow, mask_white
                    │     ├── Sobel edge magnitude > 50
                    │     └── horizon mask (top 40% ignored), white excludes left 25%
                    ├── detect_lines_in_slices(mask_y, mask_w, h)
                    │     └── 3 horizontal slices from ROI_START=0.47 → yellow_xs, white_xs
                    ├── detect_curve(yellow_xs, white_xs, threshold=350)
                    │     └── x_far - x_near shift → is_curve, direction
                    ├── _calculate_error(yellow_xs, white_xs, ...)
                    │     └── normalized lateral error ∈ [-1, 1]
                    ├── exponential filter: 0.7*prev + 0.3*raw
                    ├── _calculate_steering(error)  ← PD controller
                    ├── _motor_commands(steering, recovery, is_curve, both_visible)
                    └── _smooth(left, right, both_visible)  ← deque buffer 1 or 2
              └── wheels.set_wheels_speed(left_pwm, right_pwm) → Godot wheel socket (:5002)
              └── create_lane_visualization() → MJPEG stream → browser
```

---

### `agent.py` — `LaneServoingAgent`

**Key constants:**
- `_LINE_OFFSET = 160` — initial assumed half-lane width in pixels
- `_ROI_START = 0.47` — vertical start of detection region (47% from top)
- `_NUM_SLICES = 3` — horizontal slices for line position sampling
- `_SLICE_TOL = 5` — ±5 px strip height per slice

**Config parameters** (from `lane_servoing_config.yaml`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `p_gain` | 0.1 | Proportional gain on lateral error |
| `d_gain` | 0.6 | Derivative gain on steering |
| `max_steer` | 0.4 | Maximum steering command |
| `base_speed` | 0.33 | Base forward PWM |
| `curve_speed` | 0.15 | Speed during curve (slower) |
| `curve_threshold` | 350 | Pixel x-shift to declare a curve |
| `steering_threshold` | 0.2 | Minimum steering to apply curve boost |
| `curve_boost` | 1.3 | Speed multiplier on outer wheel in curve |
| `detection_threshold` | 100 | Minimum total lane pixels before recovery |

**Error calculation** (`_calculate_error`):
- Both lanes visible: `error = image_center - midpoint(yellow_mean, white_mean)` — also updates `_lane_half_width` with exponential filter
- Left only: `error = image_center - (yellow_mean + _lane_half_width)`
- Right only: `error = image_center - (white_mean - _lane_half_width)`
- Neither: hold previous error

**Steering** (`_calculate_steering`): PD only — `steering = p_gain * error + d_gain * (error - prev_error)`

**Motor commands** (`_motor_commands`):
- `speed = curve_speed if is_curve else base_speed`
- If only one lane visible: `speed *= 0.8`
- `left = speed - steering`, `right = speed + steering`
- On sharp curve: outer wheel gets 5× boost (right turn) or `curve_boost` × (left turn)

**Smoothing** (`_smooth`): rolling average, buffer size 2 if both lanes visible, 1 otherwise.

---

### `visual_servoing_activity.py` — Lane Detection

**Student-editable.** Called once per frame by the agent.

```python
def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # image is BGR
    # Returns: mask_yellow (float 0/1), mask_white (float 0/1)
```

Pipeline:
1. `BGR → HSV` for color thresholding
2. `BGR → Gray → GaussianBlur(sigma=2) → Sobel → magnitude > 50` for edges
3. Horizon mask: ignore top 40% (sky)
4. Yellow mask: `HSV_in_range AND edge AND horizon`
5. White mask: `HSV_in_range AND edge AND horizon AND x > w//4`  (excludes far-left strip)

**Default HSV bounds** (from `lane_servoing_hsv_config.yaml`):
| Color | H | S | V |
|-------|---|---|---|
| Yellow lower | 15 | 100 | 100 |
| Yellow upper | 35 | 255 | 255 |
| White lower | 0 | 0 | 180 |
| White upper | 179 | 80 | 255 |

`set_hsv_bounds(yellow_lower, yellow_upper, white_lower, white_upper)` — called by web UI to update in-memory globals. `get_hsv_bounds()` returns current values as a dict for the `/get_hsv` endpoint.

---

### `cuvrve_behavior.py` — Curve Detection

```python
def detect_curve(yellow_xs, white_xs, curve_threshold=350) -> Tuple[bool, int]:
```

- Prefers yellow line; falls back to white if yellow has < 2 points
- `shift = x_far - x_near` (last vs first slice x-position)
- `abs(shift) > curve_threshold` → curve detected
- Returns `(True, +1)` for right turn, `(True, -1)` for left turn

---

### `virtual_server.py` — Flask Server

**Init sequence:**
1. `GodotWheelsDriver(host, wheel_port=5002)` — TCP socket to Godot
2. `GodotCameraDriver(host, frame_port=5001)` — TCP socket, blocks until Godot connects
3. `LaneServoingAgent()` — loads config from YAML

**Routes:**
| Route | Method | Purpose |
|-------|--------|---------|
| `/` | GET | Render web UI |
| `/video` | GET | MJPEG camera stream |
| `/start` | POST | Enable motor output |
| `/stop` | POST | Zero wheels + disable |
| `/reset` | POST | Reset sim position, resume base speed |
| `/running` | GET | `{"running": bool}` |
| `/status` | GET | Frame count + current config |
| `/update_config` | POST | Update `p_gain`, `d_gain`, `base_speed`; persist to YAML |
| `/update_hsv` | POST | Update HSV bounds in-memory + persist to YAML |
| `/get_hsv` | GET | Return current HSV bounds |

---

### `visualization.py` — Debug Display

4-panel composite frame returned on every `/video` frame:

| Panel | Content |
|-------|---------|
| Top-left | Live camera with slice overlay (yellow dots = detected line positions) |
| Top-right | Combined lane mask (HOT colormap) |
| Bottom-left | White mask (BONE colormap) |
| Bottom-right | Yellow mask (green channel highlighted) |

Bottom info strip: lateral error bar (green <0.1, cyan <0.3, red ≥0.3), left/right PWM bars, "LANE OK"/"NO LANE" status, frame counter.

---

### Web UI (`servers/templates/lane_servoing.py`)

Interactive tuning dashboard:
- **HSV Calibration**: 12 sliders (yellow H/S/V min/max, white H/S/V min/max) — posts to `/update_hsv` on change
- **Drive Control**: Start / Stop buttons, polls `/running` to update indicator
- **Control Parameters**: Lateral Gain (`k_d` → `p_gain`), Heading Gain (`k_phi` → `d_gain`), Base Speed — apply button posts to `/update_config`
- **Reset Position**: posts to `/reset`

---

## Navigation Project Task

**Files:**
- `tasks/project/packages/road_map.py` — graph representation of the road map; reads nodes/edges from YAML or Godot map data
- `tasks/project/packages/optimal_path.py` — Dijkstra's algorithm over the road graph
- `servers/project/virtual_server.py` — adds manual/autonomous mode toggle on top of lane servoing

**How it works:**
1. Map is loaded from the Godot simulation (tile positions extracted to nodes + edges)
2. User selects start and destination nodes via the web UI
3. Dijkstra finds optimal path
4. Navigation mode follows the path tile-by-tile, using lane servoing for straight/curve driving and switching direction at intersections

---

## Configuration Tuning Guide

### HSV Bounds
Use the web UI sliders while watching the debug visualization panels. Yellow lines should appear bright in the yellow mask panel; white lines in the white mask panel. Adjust in this order:
1. Set V (brightness) range first to isolate lane brightness from background
2. Narrow H range to target the specific hue
3. Adjust S to exclude dull/washed-out areas

### PD Gains
- `p_gain` (Lateral Gain): Higher → more aggressive correction, risk of oscillation
- `d_gain` (Heading Gain): Higher → faster damping of oscillations, too high → jitter
- Start low, increase `p_gain` until robot tracks well, then increase `d_gain` to smooth oscillation
- `base_speed`: Lower speeds make tuning easier; increase after gains are stable

---

## Key Cross-Task Dependencies

- `servers/object_detection/virtual_server.py` imports `LaneServoingAgent` — object detection task reuses lane following as a baseline behavior
- `servers/common.py` — shared by all servers: `make_frame_generator`, `suppress_http_logs`, `shutdown_cleanup`
- `launcher/ports.py` — `find_available_port()` used by all servers to avoid port conflicts
- `launcher/config.py` — maps task names to Godot scene paths and server module paths
