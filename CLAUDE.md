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
- `apriltag_navigation/` — lane servoing + AprilTag traffic-sign reactions, see deep-dive below
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
| `apriltag_config.yaml` | Tag ID → sign type map, stop/yield/duck-crossing thresholds |

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
python launch.py --sim --task apriltag_navigation
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

## AprilTag Traffic-Sign Navigation

Wraps `LaneServoingAgent` (unchanged, via composition) and adds Duckietown
AprilTag (tag36h11) traffic-sign detection on top — same pattern as
`tasks/object_detection`'s `ObjectDetectionAgent`.

### File Map

```
tasks/apriltag_navigation/
└── packages/
    ├── agent.py              # TrafficNavigationAgent — wraps LaneServoingAgent
    ├── apriltag_detector.py  # cv2.aruco DICT_APRILTAG_36h11 wrapper -> TagDetection list
    └── sign_rules.py         # tag id -> sign type (student-editable)

servers/apriltag_navigation/
├── virtual_server.py         # Flask app for simulation
└── visualization.py          # Tag overlay + lane 4-panel + state strip

servers/templates/
└── apriltag_navigation.py    # HTML/JS UI (Sign Detection card + lane servoing controls)

config/
└── apriltag_config.yaml      # Tag id -> sign type, trigger areas, durations

GodotSimulation/ducky-bot/scenes/
├── maps/apriltag_navigation_fork.tscn # tile-grid road map + 6 sign instances (the scene launch.py loads)
└── objects/obj_apriltag_sign.tscn     # generic post+panel+texture sign (sign.gd)
```

### Sign → Tag Mapping (`config/apriltag_config.yaml`)

| Tag ID | Sign type | Behavior |
|--------|-----------|----------|
| 0 | `stop` | Stop for `stop_duration_s` (default 4s), then resume |
| 1 | `yield` | Slow to `yield_slowdown_factor` for `yield_duration_s` |
| 2 | `no_entry` | Straight ahead is blocked — turn left or right (picked at random) |
| 3 | `one_way_left` | Mandatory turn left |
| 4 | `one_way_right` | Mandatory turn right |
| 5 | `pedestrian` | Slow down; full stop only if a duckie is detected ahead |
| 6 | `duck_crossing` | Same as pedestrian |

Each sign's tag ID maps to a PNG in
`GodotSimulation/ducky-bot/textures/tag36h11/tag36_11_000XX.png`. Traffic
lights are explicitly out of scope — the only stop trigger is the `stop` tag.

### How It Works

`TrafficNavigationAgent.compute_commands(image)`:
1. `left, right = self.lane_agent.compute_commands(image)` — normal lane following, unchanged.
2. `apriltag_detector.detect_tags(bgr)` — `cv2.aruco.ArucoDetector` on `DICT_APRILTAG_36h11`, returns id/area/center/corners per tag, then sorted by `area` descending (closest tag first) so that when two signs are visible in the same frame (e.g. `one_way_left`/`one_way_right` placed close together), the nearer one wins instead of whichever `cv2.aruco` happened to return first.
3. `sign_rules.classify_tag(tag_id)` — maps tag id to sign type via `apriltag_config.yaml`.
4. A small state machine (`DRIVE` / `STOPPED` / `YIELDING` / `DUCK_WAIT` / `TURNING`) reacts once a tag's pixel area crosses its `*_trigger_area` threshold (closer = larger area), with a per-tag `sign_cooldown_s` to avoid re-triggering on the same sign every frame.
5. `DUCK_WAIT` lazily creates an `ObjectDetectionAgent` (from `tasks.object_detection`) to check for a `duckie` bbox ahead before forcing a full stop — degrades gracefully (just slows down) if no `.onnx` model is present.
6. `TURNING` (triggered by `no_entry` / `one_way_left` / `one_way_right` crossing `turn_trigger_area`) overrides the lane-following wheel speeds for `turn_duration_s`: `left/right = turn_speed ∓ turn_bias` (sign depends on `_turn_direction`, `'left'` or `'right'`). `one_way_left`/`one_way_right` set the direction directly; `no_entry` picks `random.choice(('left', 'right'))` since straight is the blocked option.
7. Debug info (`detected_signs`, `state`, `state_remaining`, `event_log`) is merged into `last_debug_info` for `/status` and the visualization overlay.

### Sign Placement Caveat

`apriltag_navigation_fork.tscn` is a **tile-grid** map (`Tiles/Tile_C_R` →
world `X = 0.6*C + 0.3`, `Z = 0.6*R + 0.3`), so signs are placed at real
road-edge coordinates. It has three junctions: **J1** T-junction `(0.9, 4.5)`
(arms N/S/E, straight-W blocked), **J2** 4-way `(2.1, 4.5)`, and **J3**
T-junction `(2.7, 2.1)` (arms W/E/S, straight-N blocked). The DuckieBot spawns
at `(1.25, 4.6)` heading **−X (west)** toward J1.

**Routing avoids the sharp geometry.** The top-right corner (the
`C8_3 ╗ → C8_4 ╝ → C7_4 ╔` hairpin) and the center spur (`C4_5 ╝ → C3_5 ╔`)
are tight back-to-back S-curves the lane-follower cannot hold at `base_speed`
— the robot gets stuck there. So the signs route it on the **left rectangle
only** (J1 → down C1 → bottom road → up C3 → J2 → R7 back to J1), which uses
only gentle 90° `tile_curve` corners and never visits J3 or the hairpin. The
loop is counter-clockwise, so every forced turn is a **left** turn; J1 is
always entered from the east and J2 always from the south, making the route
deterministic and self-sustaining.

The 5 `Signs/Sign_*` instances (each on the approach's shoulder or in the
junction's blocked arm, facing oncoming traffic):

| Sign | Tag | Pos `(X,Z)` | Role |
|------|-----|-------------|------|
| `Sign_NoEntry_J1`    | 2 | `0.6, 4.65` | J1 blocked-W arm; `no_entry` + `one_way_left` |
| `Sign_OneWayLeft_J1` | 3 | `0.6, 4.35` | together = deterministic left (S) down the C1 leg |
| `Sign_Yield_C1`      | 1 | `0.6, 5.7`  | mid-road slowdown on the long south straight |
| `Sign_Stop_J2`       | 0 | `2.4, 4.8`  | stop at the 4-way's S-approach "red line" |
| `Sign_OneWayLeft_J2` | 3 | `2.4, 4.5`  | forces left (W) onto R7 back toward J1 |

This exercises `no_entry`, `one_way_left`, `stop`, and `yield` each lap. To
also test `one_way_right`, swap a `one_way_left` tag (id 3 → 4) — but note the
loop's geometry only supports left turns, so a right turn aims the robot at the
sharp side of the map.

Signs spanning an **X-facing** plane (robot travels along X) use the 90°-about-Y
basis `(-4.371139e-08, 0, 1, 0, 1, 0, -1, 0, -4.371139e-08)`; **Z-facing** ones
(robot travels along Z) use the identity basis. Because the panels are
two-sided, the basis only sets which plane the tag spans, not which lone
direction it is seen from. These are **starting coordinates** — run
`python launch.py --sim --task apriltag_navigation` and use the live tag-area
readout in the "Sign Detection" UI card to recalibrate the `*_trigger_area`
values in `config/apriltag_config.yaml` (these were lowered to 1000–1500 to
match the small sim tags, whose pixel area peaks in the low thousands at
~0.3m). If a forced turn goes the wrong way, swap the
`one_way_left`/`one_way_right` tag on that sign (left/right is relative to the
robot's heading).

**Signs are two-sided.** `obj_apriltag_sign.tscn` has two tag PlaneMeshes:
`texture` (normal toward local `+Z`) and `texture_back`, a 180°-about-**Y**
copy of it (`Transform3D(-1, 0, 0, 0, -4.371139e-08, -1, 0, -1,
-4.371139e-08, 0, 0.13, -0.0006)`) whose normal points toward local `-Z`. A
180°-Y rotation is a pure rotation (no mirror), so the back face stays a
valid AprilTag rather than a mirrored (undetectable) one — do NOT make the
back by flipping the front about X/Z, which mirrors the pattern. `sign.gd`'s
`_apply()` loops over both meshes so the same `sign_texture` lands on each
face. Net effect: the same tag is detectable from either approach direction,
so a sign no longer has to be pre-aimed at one specific oncoming side.

Each sign instance's own transform still rotates the whole post. A 180°-Y
instance rotation — `Transform3D(-1, 0, 8.742278e-08, 0, 1, 0, -8.742278e-08,
0, -1, ...)` — puts the (now two-sided) panel's faces along world `±Z`;
identity puts them along world `∓Z`; the along-X transforms
(`±4.371139e-08` form) put them along world `±X`. Because both faces carry
the tag, the instance rotation now only controls the panel's *plane*
(which axis it spans), not which lone direction it can be seen from.

Tags 0-6's PNGs (`GodotSimulation/ducky-bot/textures/tag36h11/tag36_11_0000{0..6}.png`)
must be imported with `compress/mode=0` (Lossless) — VRAM-compressed (S3TC)
import of these tiny 10x10 textures destroys the tag's bit pattern and makes
it undetectable by `cv2.aruco`. Tags 2 and 3 were originally imported as
VRAM-compressed and have been fixed; if new tag textures are added, verify
their `.import` file uses `compress/mode=0`.

The sign post (`obj_apriltag_sign.tscn`'s `post` `BoxMesh`) is sized to stay
below the panel's bottom edge (`size.y = 0.085` vs. the panel's bottom at
`y = 0.0895`). A taller post overlaps the tag panel in Y and — since the post
sits closer to the camera in Z than the tag texture — visually occludes part
of the AprilTag, breaking `cv2.aruco`'s quad detection.

### Turn-Choice Sign Caveat

`no_entry` / `one_way_left` / `one_way_right` drive a `TURNING` state that
overrides the lane-following wheel speeds with a fixed `turn_speed ±
turn_bias` differential for `turn_duration_s` (`config/apriltag_config.yaml`).

The wheel differential maps to angular velocity in
`GodotSimulation/ducky-bot/scripts/Moveee.gd` as
`omega = (v_right - v_left) / baseline` where `v_* = wheel_cmd * max_speed`
(`max_speed=1.0`, `baseline=0.10`). So `omega = (2 * turn_bias) * 10`. The
defaults `turn_bias=0.08`, `turn_duration_s=1.0` give `omega ~= 1.6 rad/s`,
i.e. roughly a 90-degree turn. Pushing `turn_bias` much higher (e.g. the
original 0.25, which clips one wheel to 0) drives `omega` up to several
rad/s and over a 1-2s duration the robot spins multiple full rotations
instead of turning — if retuning, keep `turn_bias` small and adjust
`turn_duration_s` to hit the desired turn angle (`angle = omega *
turn_duration_s`).

This is a **starting behavior only** — `lane_follower.tscn` is a single loop
with no branching road geometry, so a "turn" currently rotates the robot
~90° in place (while still moving forward at `turn_speed`) and then hands
control back to `LaneServoingAgent`, which re-centers it on whatever lane
markings are now in front of it. To make the turn actually lead somewhere,
add a branching intersection to the map (`docs/MAP_MAKER.md`), place the
corresponding sign(s) before it, and retune `turn_trigger_area` /
`turn_duration_s` / `turn_speed` / `turn_bias` against the live tag-area
readout, the same way `*_trigger_area` values are calibrated for the other
signs.

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
- `tasks/apriltag_navigation/packages/agent.py` wraps `LaneServoingAgent` and lazily uses `ObjectDetectionAgent` for duck-crossing stops
- `servers/common.py` — shared by all servers: `make_frame_generator`, `suppress_http_logs`, `shutdown_cleanup`
- `launcher/ports.py` — `find_available_port()` used by all servers to avoid port conflicts
- `launcher/config.py` — maps task names to Godot scene paths and server module paths
