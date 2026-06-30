# Photon Ground Truth Bench

FRC 2027 RobotPy project — a **camera ground-truth calibration bench** for comparing PhotonVision pose estimates against physically measured camera poses.

## CAD model

Bench coordinate system defined in `config/bench_config.py:CADConstants`:
- **Origin**: halfway between targets and camera.
- **+X**: from origin toward calibration targets.
- **+Y**: right when facing targets.
- **+Z**: up.

Fixed poses from CAD (update when the mechanical model changes):
- `camera_pose_in_bench` — camera translation (rotation comes from IMU).
- `camera_to_imu` — Transform3d from camera reference to IMU chip.
- `camera_focal_point_offset` — Translation3d from camera ref to lens.
- `apriltag6_pose`, `apriltag7_pose`, `charuco_board_pose` — target poses in bench frame.

## Hardware — IO port map

| Port | Peripheral | Wired as |
|---|---|---|
| **PWM 0** | MG90S servo — pitch | roboRIO PWM header |
| **PWM 1** | MG90S servo — yaw | roboRIO PWM header |
| **PWM 2** | MG90S servo — roll | roboRIO PWM header |
| **I²C port 1** (0x68) | MPU6050 accel/gyro | MXP I²C bus |
| **USB** (via switch) | Camera | PhotonVision / NetworkTables |

Servo pulse range: 1000–2000 µs via `wpilib.PWM.setPulseTime()`.

## What it does

1. Commands the bench to a target camera pose via three servos.
2. Reads ground-truth orientation from the onboard MPU6050 (Mahony complementary filter).
3. Captures a PhotonVision pose estimate at the same moment.
4. Records both for offline comparison.

## Modes (Driver- Station selectable)

The robot uses the **OpModeRobot** framework — modes are registered with `@teleop`, `@autonomous`, and `@utility` decorators and appear as selectable options in the Driver Station:

| DS Slot | Mode | Description |
|---|---|---|
| **UTILITY** | Calibrate Servos | Random-position sweep → IMU recording → CSV for offline calibration-map fitting |
| **UTILITY** | Zero IMU | Re-run gyro-bias estimation in isolation |
| **AUTONOMOUS** | Static Pose Test | Per-pose: trapezoid profile → IMU stability check → 1 s sampling window. Records IMU mean/std, PhotonVision RMS translation/rotation error per axis, expected tag IDs. |
| **AUTONOMOUS** | Dynamic Sweep Test | Sinusoidal sweeps at increasing velocity; track error vs angular rate |
| **TELEOPERATED** | Manual Operation | Joystick-driven servo positioner |

## Running tests

All test modes are deployed as AUTONOMOUS opmodes. Select via Driver Station:

1. Deploy to roboRIO: `robotpy deploy`
2. In Driver Station, switch to **AUTONOMOUS** mode.
3. Select the desired test from the autonomous routine dropdown.
4. Enable the robot. The test runs immediately.

**To stop early**, disable the robot. `end()` flushes any accumulated results to CSV.

### Static Pose Test — phase machine

| Phase | What happens |
|---|---|
| **ZEROING** | IMU gyro bias estimation (~100 cycles). |
| **MOVING** | Commands the positioner to the next pose via `set_goal_rad()`. |
| **PROFILE_WAIT** | Waits for the trapezoid profile output to equal the goal (no timeout). |
| **STABILIZING** | Buffers 1 s of IMU Euler angles. Once peak-to-peak range < 0.5° per axis for a full 1 s, proceeds. Timeout after 5 s (logs a warning, proceeds anyway). |
| **SAMPLING** | 1 s window (50 cycles): collects IMU Euler + PhotonVision `estimatedPose` every cycle. |
| **RECORD** | Computes IMU mean/std, per-sample GT⁻¹ × PV transform, RMS per axis. Advances to the next pose. |
| **DONE** | Flushes all results to CSV. |

**NT telemetry** (live during run):

| Key | Type | Meaning |
|---|---|---|
| `static_pose/running` | bool | 1 while test is executing |
| `static_pose/pose_index` | number | Current pose (0-based) |
| `static_pose/total_poses` | number | Total poses in the sweep |
| `static_pose/completed` | bool | 1 after all poses finish |
| `static_pose/csv_path` | string | Path of the output CSV |
| `static_pose/warning` | string | Non-empty if stability timeout occurred |

### Dynamic Sweep Test

*(Phase machine and NT keys — TBD — follow the same pattern.)*

## Results

Each test writes one CSV to the roboRIO. Retrieve via SCP after the run:

```bash
# Static Pose Test results
scp admin@roborio-XXXX-frc.local:/home/lvuser/calibration_data/static_pose_results.csv .

# Calibration sweep data (used by scripts/download_calibration.py)
scp admin@roborio-XXXX-frc.local:/home/lvuser/calibration_data/servo_calibration_*.csv .
```

### Static Pose Test CSV columns

| Column | Description |
|---|---|
| `pose_idx` | 0-based pose index |
| `expected_tags` | AprilTag IDs expected in view (e.g. `6,7`) |
| `cmd_roll/pitch/yaw` | Commanded camera orientation (rad) |
| `imu_count` | IMU samples in the 1 s window (always 50) |
| `pv_count` | PhotonVision frames with a valid pose estimate |
| `imu_mean_roll/pitch/yaw` | Mean IMU Euler angles over the window (rad) |
| `imu_std_roll/pitch/yaw` | Standard deviation of IMU Euler angles (rad) |
| `rms_dx/dy/dz` | RMS translation error (m) — GT⁻¹ × PV, in camera frame |
| `rms_droll/dpitch/dyaw` | RMS rotation error (rad) |

Rows with `pv_count = 0` and zeros in the RMS columns indicate no PV data was available at that pose.

## Project layout

| Path | Role |
|---|---|
| `robot.py` | `OpModeRobot` entry point + `@teleop`/`@autonomous`/`@utility` decorators |
| `modes/` | OpMode subclasses (one file per mode) |
| `hardware/` | Hardware abstractions (PWM, I²C, PhotonVision) |
| `core/` | Base classes (lightweight `Subsystem` ABC) |
| `config/` | Physical constants, calibration map |
| `utilities/` | Pure-math domain (no wpilib imports) |
| `scripts/` | Laptop-only offline tools (calibration fitting) |
| `tests/` | pytest unit tests |

## Getting started

```bash
robotpy sync

# Simulation
robotpy sim

# Tests
python -m pytest tests/
```

## Conventions

- **`utilities/` never imports `wpilib`** — keeps domain logic pure and testable.
- **`config/bench_config.py`** is the single source of truth for physical layout. Key classes: `CADConstants` (fixed tag/camera poses), `PositionerConfig` (servo ranges), `ValidationConfig` (pose lists, expected tags).
- **Hardware** classes extend `core.Subsystem` and provide a `periodic()` method called from `robotPeriodic()`.
- **OpModes** receive the `Robot` instance in their constructor and access hardware via typed attributes (`robot.sensors`, `robot.positioner`, `robot.vision`).
- State machines live in `OpMode.periodic()` — no blocking loops.
- NetworkTables keys are documented per-file in `hardware/` source.
- Run `ruff check . && python -m mypy . --strict && python -m pytest tests/` before committing.

## Dependencies

- `robotpy>=2027` (metapackage — includes `OpModeRobot`)
- `photonlibpy==2027.0.0a2`
- `robotpy-sim` (desktop simulation)

See `pyproject.toml` for exact version pins.
