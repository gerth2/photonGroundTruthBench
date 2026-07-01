# AGENTS.md

FRC 2027 RobotPy project — camera ground-truth calibration bench.

## Project identity

- **Year:** 2027 (NOT 2026 or prior. 2027 has major breaking changes, read docs with caution.). Use `robotpy` metapackage version `2027.*` and `photonlibpy` pinned to `2027.*`.
- **RobotPy commands:** `uv run robotpy sync` (install deps), `uv run robotpy deploy` (to SystemCore), `uv run robotpy --help`.
- **Purpose:** Drive a physical test bench that positions a camera at known poses relative to Apriltag/calibration targets, then compares PhotonVision pose estimates against assumed-correct ground-truth sensor readings.

## Structure

```
robot.py                  — OpModeRobot entry point + @teleop/@autonomous/@utility decorators
modes/                    — OpMode subclasses (one file per mode)
hardware/                 — Subsystem hardware abstractions (lightweight, no WPILib framework)
core/                     — Base classes (Subsystem ABC)
config/bench_config.py    — Physical constants (sensor ranges, motor IDs, tag field layout)
utilities/                — Pure-math domain: measurement models, comparison math, IMU filter
tests/                    — pytest (robotpy-sim or sim-only)
```

- `hardware/` owns hardware IO (reads sensors, writes PWM); each class extends `core.Subsystem` for a consistent `periodic()` contract.
- `utilities/` owns domain logic (pure math, no hardware calls, no wpilib imports).
- Add new hardware in `hardware/`, new opmodes in `modes/`, new math in `utilities/`.
- Top-level flow is OpMode-based via `OpModeRobot` — no command scheduler.

## OpMode framework

`robot.py` exports three decorators that register an `OpMode` subclass with the robot:

```python
from robot import teleop, autonomous, utility
from wpilib import PeriodicOpMode

@teleop("Manual Control", group="Bench")
class MyTeleop(PeriodicOpMode):
    def __init__(self, robot: Robot): ...
    def start(self): ...
    def periodic(self): ...
    def end(self): ...
```

| Decorator       | DS Mode         | Typical use                |
|-----------------|-----------------|----------------------------|
| `@teleop`       | TELEOPERATED    | Manual joystick control    |
| `@autonomous`   | AUTONOMOUS      | Full benchmark run         |
| `@utility`      | UTILITY         | Calibration sweeps, tests  |

The `Robot` instance is passed to each OpMode constructor (via `addOpMode`'s factory). Hardware subsystems live on the Robot and are called from `robotPeriodic()`.

## Conventions

- **Modular constants** — split by concern in `config/` (don't dump everything in one file).
- **OpMode-based** — one class per mode, state machine in `periodic()`, lifecycle via `start()` / `end()`.
- **Robot** owns all hardware instantiation; opmodes receive it in their constructor and access subsystems via typed attributes (`robot.sensors`, `robot.positioner`, etc.).
- **Don't import wpilib in utilities/domain logic** — keep it pure Python for testability.

## Commenting

Every `.py` file and every function (public **and** private) must carry a
docstring.  The purpose is to make the code readable without having to read
its body — a compact summary that a teammate can scan in the call-stack
peek of their editor.

### Docstring rules

1. **Every module** — one paragraph: what the module owns / is responsible
   for.  No list of classes it contains (importers can see that).
2. **Every class** — one paragraph: what the class encapsulates, what its
   lifecycle looks like, any invariants the caller must uphold.
3. **Every function / method** (including `__init__`, private helpers, and
   `@property` bodies) — one or two sentences covering:
   - **What** it does (not *how* — that's the body).
   - **Parameters** (if not obvious from the type annotation + name).
   - **Return value** (if any).
   - **Side effects** (mutates state, writes a file, starts a timer, …).

For trivial one-liners (`def x(self) -> int: return self._x`) a single
phrase is fine: `"""Return the cached x value."""`.

### Forbidden

- Negative descriptions ("does not block", "no goniometric fallback").
- WPILib or project-wide conventions ("state machine in periodic()").
- Section-marker comments (`# ── Config ──`).
- Agent workflow instructions or TODOs about future features.
- "Optional" disclaimers for parameters that always have a value.
- Implementation details that belong in the function body.

### Enforcement

A CI-compatible check lives at `scripts/check_docstrings.py`.  It parses
every `.py` file with the `ast` module and reports any module, class, or
function that lacks a docstring.  The project-level lint suite runs it
after `ruff`:

```
uv run ruff check . && uv run python scripts/check_docstrings.py
```

## Key commands

| Action | Command |
|---|---|---|
| Create venv + install deps | `uv venv && uv pip install --prerelease=allow robotpy==2027.0.0a6.post1 photonlibpy==2027.0.0a2 pytest ruff mypy` |
| Activate venv (per shell) | `source .venv/bin/activate` |
| Deploy to SystemCore | `uv run robotpy deploy` |
| Run sim (desktop) | `uv run robotpy sim` |
| Run tests | `uv run python -m pytest tests/` |
| Format | `uv run ruff format .` |
| Lint | `uv run ruff check . && uv run python scripts/check_docstrings.py` |
| Typecheck | `uv run python -m mypy . --strict` |
| Retrieve test results (AUTONOMOUS) | `scp lvuser@systemcore-6708.local:/home/lvuser/test_results/static_pose_results.csv .` |
| Retrieve calibration data (UTILITY) | `scp lvuser@systemcore-6708.local:/home/lvuser/calibration_data/servo_calib_*.csv .` |

Run `lint -> typecheck -> test` before committing. All three are expected to pass.  The full lint command is `uv run ruff check . && uv run python scripts/check_docstrings.py`.

## First-time setup

```bash
uv venv
uv pip install --prerelease=allow \
  robotpy==2027.0.0a6.post1 \
  photonlibpy==2027.0.0a2 \
  pytest ruff mypy
# Activate for the current shell session:
source .venv/bin/activate
```

## Development workflow

1. **Code review the delta** — every change must include a review of what was added/changed.
2. **New unit test** — every change must include a new unit test (or modification of an existing test) that covers the new behavior.
3. **All tests pass** — after each change, the full test suite must pass before moving on.
4. **Iterate until green** — if tests fail, fix the code (not the tests). Only ask the user about deleting a test if it genuinely appears obsolete.
5. **Audit docs** — after each change, review `README.md` and `AGENTS.md`, and update them to reflect the current state of the project. Do not include development history ("first we tried X, then Y") in any file — git history provides that. Plainly state how the code works *now*.
6. **Commit per change** — after each change passes all checks, commit and push. Summarise *why* the change is important in the commit message, not just what changed.

## Dependencies (pyproject.toml)

- `robotpy>=2027` (metapackage, includes OpModeRobot)
- `photonlibpy>=2027` (PhotonVision)
- For offline/sim: `robotpy-sim` component

## Hardware — servo positioner

- Three MG90S servos (roll, pitch, yaw) on SystemCore PWM channels 0-2.
- Servo output via `wpilib.Servo.set()` with range 0-1. Config provides center/min/max in **-1..1 space** (see `PositionerConfig` in `config/bench_config.py`). The `CameraPositioner` hardware class maps commanded radians to this range.
- **Two command modes**, both always smooth — no instantaneous snaps:
  - **RPY (radians)** — `set_goal_rad()` → trapezoid profile + optional PI loop. Uses the inverse calibration map for feedforward.
  - **N11 (raw servo space)** — `set_target_n11()` → linear slew from current position. The positioner interpolates internally; callers just set the target and optionally check `n11_slew_finished()`.
- `set_raw_n11()` exists as a low-level bypass (immediate direct PWM, cancels any active profile or slew). Used for initial homing.
- `get_current_n11()` returns the last commanded n11 for state initialisation.
- **No blocking** loops anywhere — all state machines run in `periodic()` (20 ms cycles). The positioner's own internal state machine (profile or slew) advances each cycle.
- **Soft limits**: every N11 value is clamped to `[min, max]` per axis at every API entry point and at PWM write. The full `[-1, 1]` N11 range represents the servo's full physical travel; soft limits define the mechanism-safe window.
- **RPY → N11**: the only conversion path is `CalibrationMap.inverse()`. No goniometric fallback.

## Hardware — IMU ground truth

- **MPU6050** accelerometer + gyroscope (I2C, address `0x68`). Driver at `hardware/mpu6050.py`.
- **Zeroing** (first phase of any OpMode): state machine in `GroundTruthSensors.periodic()` accumulates N gyro samples, averages to estimate bias, stores as offset in the filter. No `time.sleep()` — runs at loop rate.
- **Mahony complementary filter** (`utilities/imu_filter.py`) fuses gyro integration with accelerometer gravity-vector correction into a quaternion. Output is a `Rotation3d` via `get_rotation()`.
  - Gyro dominates short timescale (high-rate integration).
  - Accelerometer corrects roll/pitch drift (low-pass tilt from gravity).
  - Yaw drifts freely (no magnetometer). Acceptable for short bench runs.
- `GroundTruthSensors.periodic()` reads sensor each cycle and pushes through the filter.

## Servo calibration pipeline

1. **On SystemCore** — `CalibrateServosMode` opmode homes the positioner to center, zeros the IMU at that repeatable mechanical zero, then commands random servo triplets one per settling window via `positioner.set_target_n11()` (linear slew), records IMU rotation via `average_rotations()` (1 s sample window), writes CSV to `/home/lvuser/calibration_data/`. All state in `periodic()` — no blocking.
2. **Off-bench** — `scripts/download_calibration.py` SSH/SCPs CSV from SystemCore, fits degree-2 polynomial (forward: servo→angle, **inverse**: angle→servo), plots residual distributions, and writes `config/servo_calibration_map.py` on user approval.
3. **At runtime** — `CameraPositioner.set_goal_rad()` uses `CalibrationMap.inverse()` for feedforward, then PI in `periodic()` converges to the commanded angle.

Key files:
- `modes/calibrate_servos.py` — state machine (HOME → ZEROING → MOVING → SLEWING → SETTLING → SAMPLING → RECORD → DONE)
- `modes/home_servos.py` — linear n11 slew back to centre
- `scripts/download_calibration.py` — standalone tool, depends on `numpy` + `matplotlib`
- `config/servo_calibration_map.py` — auto-generated CalibrationMap class
- `config/bench_config.py` — `PIDConfig` (gains per axis), `CalibrationConfig` (sweep params, storage path)

## Measurement standard

The bench evaluates PhotonVision's per-tag camera-pose accuracy by comparing
each detected tag's implied camera pose against a ground-truth camera pose.

### Ground-truth (expected) camera pose

```
expected_pose = Pose3d(
    translation = camera_pose_in_bench.translation()   # from CAD
    rotation    = Rotation3d(roll_imu, pitch_imu, yaw_imu)  # from Mahony filter
)
```

The IMU provides the camera's orientation; the CAD model provides the camera's
translation relative to the bench origin.

### PhotonVision camera pose (per tag)

For each visible bench tag (IDs 6 = left, 7 = right), PhotonVision reports a
``cameraToTarget`` transform.  The camera pose implied by that detection is:

```
pv_camera_pose = tag_pose * inverse(cameraToTarget)
```

where ``tag_pose`` is the known bench-frame pose of that tag from CAD.

### Error computed per tag detection

For every tag-based camera-pose sample in a measurement window:

```
error = Transform3d(expected_pose, pv_camera_pose)
```

produces a 6-DOF residual (dx, dy, dz, droll, dpitch, dyaw).  The per-pose
CSV row reports the RMS of each axis across all tag detections in the window.

### Tag vocabulary

Only tags 6 (left) and 7 (right) are bench tags.  All other tag IDs are
ignored.  The mapping is hardcoded in ``CADConstants``:

- ``left_tag_id = 6``, ``right_tag_id = 7``
- ``left_tag_pose`` / ``right_tag_pose`` — bench-frame Pose3d from CAD

## Hardware — coordinate system & CAD constants

Bench coordinate system defined in `config/bench_config.py`:
- **Origin**: halfway between targets and camera.
- **+X**: from origin toward calibration targets.
- **+Y**: right when facing targets.
- **+Z**: up.

Update `CADConstants` class when CAD changes:
- `camera_pose_in_bench` — `Pose3d` camera translation in bench frame (rotation from IMU).
- `camera_to_imu` — `Transform3d` from camera reference to IMU chip.
- `camera_focal_point_offset` — `Translation3d` from camera ref to lens focal point.
- `left_tag_pose`, `right_tag_pose`, `charuco_board_pose` — fixed `Pose3d` in bench frame.
