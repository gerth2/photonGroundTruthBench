# Photon Ground Truth Bench

FRC 2027 RobotPy project — a **camera ground-truth calibration bench** for comparing PhotonVision pose estimates against physically measured camera poses.

1. Commands the bench to a target camera pose via three servos.
2. Reads ground-truth orientation from the onboard MPU6050 (Mahony complementary filter).
3. Captures a PhotonVision pose estimate at the same moment.
4. Records both for offline comparison.

## Getting started

```bash
uv venv
uv pip install --prerelease=allow \
  robotpy==2027.0.0a6.post1 \
  photonlibpy==2027.0.0a2 \
  matplotlib numpy pytest ruff mypy
source .venv/bin/activate

# Deploy to SystemCore
uv run robotpy deploy

# Desktop simulation
uv run robotpy sim

# Offline tests
uv run python -m pytest tests/
```

## Hardware IO port map

| Port | Peripheral | Wired as |
|---|---|---|
| **PWM 0** | MG90S servo — pitch | SystemCore PWM header |
| **PWM 1** | MG90S servo — yaw | SystemCore PWM header |
| **PWM 2** | MG90S servo — roll | SystemCore PWM header |
| **I²C port 1** (0x68) | MPU6050 accel/gyro | SystemCore I²C bus |
| **USB** (via switch) | Camera | PhotonVision / NetworkTables |

Servo pulse range: 1000–2000 µs (linear N11 mapping).

## N11 coordinate frame

All servos use a normalised **N11** space (`-1` to `1`):

| N11  | PWM pulse |
|------|-----------|
| `-1` | 1000 µs   |
| ` 0` | 1500 µs   |
| ` 1` | 2000 µs   |

The `[-1, 1]` range represents the servo's full physical travel. Per-axis N11 **soft limits** (`PositionerConfig` in `config/bench_config.py`) define the mechanism-safe operating window and are enforced at every command entry point.

RPY (radians) → N11 conversion uses `CalibrationMap.inverse()`, a degree-2 polynomial fit from the calibration pipeline.

## Modes (Driver-Station selectable)

| DS Slot | Mode | Description |
|---|---|---|
| **UTILITY** | Calibrate Servos | Random-position sweep → linear slew → IMU recording → CSV for offline map fitting |
| **UTILITY** | Zero IMU | Re-run gyro-bias estimation in isolation |
| **AUTONOMOUS** | Static Pose Test | Per-pose: profile to target → IMU stability check → 1 s sampling window. Records IMU, PV error, expected tag IDs |
| **AUTONOMOUS** | Dynamic Sweep Test | Sinusoidal sweeps at increasing velocity |
| **TELEOPERATED** | Manual Operation | Joystick-driven servo positioner |
| **TELEOPERATED** | Home Servos | Linear slew all axes back to centre |

### Static Pose Test phases

| Phase | Description |
|---|---|
| **ZEROING** | IMU gyro bias estimation (~100 cycles) |
| **MOVING** | `set_goal_rad()` to the next pose |
| **PROFILE_WAIT** | Wait for trapezoid profile to reach goal |
| **STABILIZING** | 1 s IMU buffer; proceed when peak-to-peak < 0.5° or 5 s timeout |
| **SAMPLING** | 1 s window: collect IMU + PhotonVision every cycle |
| **RECORD** | Compute RMS per axis, advance to next pose |
| **DONE** | Flush CSV, return to first pose |

### Calibrate Servos phases

| Phase | Description |
|---|---|
| **HOME** | Slew to centre (0,0,0) N11, settle 3 s |
| **ZEROING** | Gyro bias estimation at repeatable mechanical zero |
| **MOVING** | Select next random servo triplet |
| **SLEWING** | Linear n11 ramp over 1 s |
| **SETTLING** | Wait 3 s for mechanical damping |
| **SAMPLING** | Accumulate 50 IMU readings (1 s) |
| **RECORD** | Average buffer, write to CSV |
| **DONE** | Flush CSV to disk |

## Retrieving results

**AUTONOMOUS test results:**
```bash
scp lvuser@systemcore-6708.local:/home/lvuser/test_results/static_pose_results.csv .
```

**Calibration data (UTILITY):**
```bash
scp lvuser@systemcore-6708.local:/home/lvuser/calibration_data/servo_calib_*.csv .
```

## CSV column reference

### Static Pose Test

| Column | Description |
|---|---|
| `pose_idx` | 0-based pose index |
| `expected_tags` | AprilTag IDs expected in view |
| `cmd_roll (rad)` / `cmd_pitch (rad)` / `cmd_yaw (rad)` | Commanded camera orientation |
| `imu_count` / `pv_count` | Samples in the 1 s window |
| `imu_mean_roll (rad)` / … | Mean IMU Euler angles |
| `imu_std_roll (rad)` / … | Standard deviation of IMU Euler angles |
| `rms_dx (m)` / `rms_dy (m)` / `rms_dz (m)` | RMS translation error (GT⁻¹ × PV, camera frame) |
| `rms_droll (rad)` / `rms_dpitch (rad)` / `rms_dyaw (rad)` | RMS rotation error |

Rows with `pv_count = 0` indicate no PhotonVision data at that pose.

## Project layout

| Path | Role |
|---|---|
| `robot.py` | `OpModeRobot` entry point + mode decorators |
| `modes/` | OpMode subclasses (one file per mode) |
| `hardware/` | Hardware abstractions (PWM, I²C, PhotonVision) |
| `core/` | Base `Subsystem` ABC |
| `config/` | Physical constants, calibration map |
| `utilities/` | Pure-math domain (no wpilib imports) |
| `scripts/` | Laptop-only offline tools |
| `tests/` | pytest unit tests |

## Dependencies

- `robotpy>=2027` (metapackage — includes `OpModeRobot`)
- `photonlibpy==2027.0.0a2`
- `robotpy-sim` (desktop simulation)
- `matplotlib`, `numpy` (offline calibration fitting)

See `pyproject.toml` for exact version pins.
