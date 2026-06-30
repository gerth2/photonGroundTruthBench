# AGENTS.md

FRC 2027 RobotPy project — camera ground-truth calibration bench.

## Project identity

- **Year:** 2027 (NOT 2026). Use `robotpy` metapackage version `2027.*` and `photonlibpy` pinned to `2027.*`.
- **RobotPy commands:** `robotpy sync` (install deps), `robotpy deploy` (to roboRIO), `robotpy --help`.
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

## Key commands

| Action | Command |
|---|---|
| Sync deps | `robotpy sync` |
| Deploy to roboRIO | `robotpy deploy` |
| Run sim (desktop) | `robotpy sim` |
| Run tests | `python -m pytest tests/` |
| Format | `ruff format .` |
| Lint | `ruff check .` |
| Typecheck | `python -m mypy . --strict` |

Run `lint -> typecheck -> test` before committing. All three are expected to pass.

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

- Three MG90S servos (roll, pitch, yaw) on roboRIO PWM channels 0-2.
- Servo output via `wpilib.Servo.set()` with range 0-1. Config provides center/min/max in **-1..1 space** (see `PositionerConfig` in `config/bench_config.py`). The `CameraPositioner` hardware class maps commanded radians to this range.
- **Open-loop only** — 3D-printed parts have slop; do not trust commanded position as ground truth.
- Closed-loop correction uses a **PI controller per axis** in `CameraPositioner.periodic()`. The inverse calibration map (from `CalibrationMap`) provides feedforward; the PI controller corrects the residual. I term typically dominates.
- **No blocking** loops anywhere — all state machines run in `periodic()` (20 ms cycles).

## Hardware — IMU ground truth

- **MPU6050** accelerometer + gyroscope (I2C, address `0x68`). Driver at `hardware/mpu6050.py`.
- **Zeroing** (first phase of any OpMode): state machine in `GroundTruthSensors.periodic()` accumulates N gyro samples, averages to estimate bias, stores as offset in the filter. No `time.sleep()` — runs at loop rate.
- **Mahony complementary filter** (`utilities/imu_filter.py`) fuses gyro integration with accelerometer gravity-vector correction into a quaternion. Output is a `Rotation3d` via `get_rotation()`.
  - Gyro dominates short timescale (high-rate integration).
  - Accelerometer corrects roll/pitch drift (low-pass tilt from gravity).
  - Yaw drifts freely (no magnetometer). Acceptable for short bench runs.
- `GroundTruthSensors.periodic()` reads sensor each cycle and pushes through the filter.

## Servo calibration pipeline

1. **On roboRIO** — `CalibrateServosMode` opmode zeros the IMU, then commands random servo triplets one per settling window, records IMU rotation via `average_rotations()` (200 ms sample window), writes CSV to `/home/lvuser/calibration_data/`. All state in `periodic()` — no blocking.
2. **Off-bench** — `scripts/download_calibration.py` SSH/SCPs CSV from roboRIO, fits degree-2 polynomial (forward: servo→angle, **inverse**: angle→servo), plots residual distributions, and writes `config/servo_calibration_map.py` on user approval.
3. **At runtime** — `CameraPositioner.set_goal_rad()` uses `CalibrationMap.inverse()` for feedforward, then PI in `periodic()` converges to the commanded angle.

Key files:
- `modes/calibrate_servos.py` — state machine (ZEROING → MOVING → SETTLING → SAMPLING → RECORD)
- `scripts/download_calibration.py` — standalone tool, depends on `numpy` + `matplotlib`
- `config/servo_calibration_map.py` — auto-generated CalibrationMap class
- `config/bench_config.py` — `PIDConfig` (gains per axis), `CalibrationConfig` (sweep params, storage path)

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
- `apriltag6_pose`, `apriltag7_pose`, `charuco_board_pose` — fixed `Pose3d` in bench frame.
