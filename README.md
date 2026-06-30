# Photon Ground Truth Bench

FRC 2027 RobotPy project — a **camera ground-truth calibration bench** for comparing PhotonVision pose estimates against physically measured camera poses.

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
| **AUTONOMOUS** | Static Pose Test | Sequence of pre-planned poses; measure IMU vs PV error per pose |
| **AUTONOMOUS** | Dynamic Sweep Test | Sinusoidal sweeps at increasing velocity; track error vs angular rate |
| **TELEOPERATED** | Manual Operation | Joystick-driven servo positioner |

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
- **`config/bench_config.py`** is the single source of truth for physical layout.
- **Hardware** classes extend `core.Subsystem` and provide a `periodic()` method called from `robotPeriodic()`.
- **OpModes** receive the `Robot` instance in their constructor and access hardware via typed attributes (`robot.sensors`, `robot.positioner`, `robot.vision`).
- State machines live in `OpMode.periodic()` — no blocking loops.
- Run `ruff check . && python -m mypy . --strict && python -m pytest tests/` before committing.

## Dependencies

- `robotpy>=2027` (metapackage — includes `OpModeRobot`)
- `photonlibpy==2027.0.0a2`
- `robotpy-sim` (desktop simulation)

See `pyproject.toml` for exact version pins.
