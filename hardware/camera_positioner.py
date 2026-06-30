"""Subsystem for the 3-axis servo positioner (roll, pitch, yaw).

Pulse range 1000-2000 us (standard for MG90S servos).

Closed-loop correction (PI) runs in periodic() when the IMU has been
zeroed and a goal has been commanded via set_goal_rad().  Each axis
has a 1-D trapezoidal motion profile (TrapezoidProfile) that smooths
step inputs — the profile output, not the raw goal, is used as the
feedforward setpoint in the PI loop.
"""

import math

import wpilib
from wpimath import TrapezoidProfile

from config.servo_calibration_map import CalibrationMap
from core.subsystem import Subsystem
from hardware.ground_truth_sensors import GroundTruthSensors


class CameraPositioner(Subsystem):
    def __init__(
        self,
        pitch_channel: int,
        yaw_channel: int,
        roll_channel: int,
        pitch_center: float,
        pitch_min: float,
        pitch_max: float,
        pitch_range_deg: tuple[float, float],
        yaw_center: float,
        yaw_min: float,
        yaw_max: float,
        yaw_range_deg: tuple[float, float],
        roll_center: float,
        roll_min: float,
        roll_max: float,
        roll_range_deg: tuple[float, float],
        sensors: GroundTruthSensors,
        calibration_map: type[CalibrationMap] | None = None,
        pitch_kp: float = 0.0,
        pitch_ki: float = 0.0,
        yaw_kp: float = 0.0,
        yaw_ki: float = 0.0,
        roll_kp: float = 0.0,
        roll_ki: float = 0.0,
        integral_limit: float = 0.0,
        position_tolerance_rad: float = 0.05,
        pitch_max_velocity: float = 0.5,
        pitch_max_acceleration: float = 1.0,
        yaw_max_velocity: float = 0.5,
        yaw_max_acceleration: float = 1.0,
        roll_max_velocity: float = 0.5,
        roll_max_acceleration: float = 1.0,
    ) -> None:
        super().__init__()

        self._pitch_servo = wpilib.PWM(pitch_channel)
        self._yaw_servo = wpilib.PWM(yaw_channel)
        self._roll_servo = wpilib.PWM(roll_channel)

        self._sensors = sensors
        self._cal = calibration_map

        self._pitch_kp = pitch_kp
        self._pitch_ki = pitch_ki
        self._yaw_kp = yaw_kp
        self._yaw_ki = yaw_ki
        self._roll_kp = roll_kp
        self._roll_ki = roll_ki
        self._integral_limit = integral_limit
        self._position_tolerance_rad = position_tolerance_rad

        self._pitch_center = pitch_center
        self._pitch_min = pitch_min
        self._pitch_max = pitch_max
        self._pitch_range = pitch_range_deg

        self._yaw_center = yaw_center
        self._yaw_min = yaw_min
        self._yaw_max = yaw_max
        self._yaw_range = yaw_range_deg

        self._roll_center = roll_center
        self._roll_min = roll_min
        self._roll_max = roll_max
        self._roll_range = roll_range_deg

        self._p_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(pitch_max_velocity, pitch_max_acceleration)
        )
        self._y_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(yaw_max_velocity, yaw_max_acceleration)
        )
        self._r_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(roll_max_velocity, roll_max_acceleration)
        )

        self._ff_pitch_n11: float | None = None
        self._ff_yaw_n11: float | None = None
        self._ff_roll_n11: float | None = None

        self._desired_pitch: float | None = None
        self._desired_yaw: float | None = None
        self._desired_roll: float | None = None

        self._profile_t = 0.0

        self._profiled_pitch: float = 0.0
        self._profiled_yaw: float = 0.0
        self._profiled_roll: float = 0.0

        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0
        self._feedback_enabled = True

    @staticmethod
    def _angle_to_n11(
        angle_deg: float,
        center: float,
        lo: float,
        hi: float,
        range_deg: tuple[float, float],
    ) -> float:
        if angle_deg >= 0.0:
            frac = angle_deg / range_deg[1] if range_deg[1] != 0.0 else 0.0
            return center + frac * (hi - center)
        else:
            frac = angle_deg / range_deg[0] if range_deg[0] != 0.0 else 0.0
            return center + frac * (center - lo)

    def set_goal_rad(self, pitch_rad: float, yaw_rad: float, roll_rad: float) -> None:
        self._desired_pitch = pitch_rad
        self._desired_yaw = yaw_rad
        self._desired_roll = roll_rad

        self._profile_t = 0.0
        self._profiled_pitch = pitch_rad
        self._profiled_yaw = yaw_rad
        self._profiled_roll = roll_rad

        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0

        if self._cal is not None and hasattr(self._cal, "INVERSE_COEFFS_SERVO_R"):
            sr, sp, sy = self._cal.inverse(pitch_rad, yaw_rad, roll_rad)
            self._ff_pitch_n11 = float(sp)
            self._ff_yaw_n11 = float(sy)
            self._ff_roll_n11 = float(sr)
        else:
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)
            roll_deg = math.degrees(roll_rad)

            self._ff_pitch_n11 = CameraPositioner._angle_to_n11(
                pitch_deg,
                self._pitch_center,
                self._pitch_min,
                self._pitch_max,
                self._pitch_range,
            )
            self._ff_yaw_n11 = CameraPositioner._angle_to_n11(
                yaw_deg,
                self._yaw_center,
                self._yaw_min,
                self._yaw_max,
                self._yaw_range,
            )
            self._ff_roll_n11 = CameraPositioner._angle_to_n11(
                roll_deg,
                self._roll_center,
                self._roll_min,
                self._roll_max,
                self._roll_range,
            )

    def set_raw_n11(self, pitch: float, yaw: float, roll: float) -> None:
        self._ff_pitch_n11 = float(pitch)
        self._ff_yaw_n11 = float(yaw)
        self._ff_roll_n11 = float(roll)
        self._desired_pitch = None
        self._desired_yaw = None
        self._desired_roll = None
        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0

    def at_goal(self) -> bool:
        if (
            self._ff_pitch_n11 is None
            or self._ff_yaw_n11 is None
            or self._ff_roll_n11 is None
        ):
            return False

        if (
            self._feedback_enabled
            and self._sensors.is_zeroed()
            and self._desired_pitch is not None
            and self._desired_yaw is not None
            and self._desired_roll is not None
        ):
            ap, ay, ar = self._sensors.get_euler_angles()
            return (
                abs(self._desired_pitch - ap) < self._position_tolerance_rad
                and abs(self._desired_yaw - ay) < self._position_tolerance_rad
                and abs(self._desired_roll - ar) < self._position_tolerance_rad
            )

        return True

    def enable_feedback(self) -> None:
        self._feedback_enabled = True

    def disable_feedback(self) -> None:
        self._feedback_enabled = False
        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0

    @staticmethod
    def _n11_to_pulse(v: float) -> int:
        clamped = max(-1.0, min(1.0, v))
        zero_to_one = (clamped + 1.0) / 2.0
        return int(1000 + zero_to_one * 1000)

    def periodic(self) -> None:
        pitch_cmd = self._ff_pitch_n11
        yaw_cmd = self._ff_yaw_n11
        roll_cmd = self._ff_roll_n11

        err_p = 0.0
        err_y = 0.0
        err_r = 0.0

        if pitch_cmd is None or yaw_cmd is None or roll_cmd is None:
            self._publish_telemetry(pitch_cmd, yaw_cmd, roll_cmd, err_p, err_y, err_r)
            return

        if (
            self._desired_pitch is not None
            and self._desired_yaw is not None
            and self._desired_roll is not None
        ):
            dt = 0.02
            self._profile_t += dt

            pitch_goal = TrapezoidProfile.State(self._desired_pitch, 0.0)
            yaw_goal = TrapezoidProfile.State(self._desired_yaw, 0.0)
            roll_goal = TrapezoidProfile.State(self._desired_roll, 0.0)

            pitch_current = TrapezoidProfile.State(self._profiled_pitch, 0.0)
            yaw_current = TrapezoidProfile.State(self._profiled_yaw, 0.0)
            roll_current = TrapezoidProfile.State(self._profiled_roll, 0.0)

            s_p = self._p_profile.calculate(self._profile_t, pitch_current, pitch_goal)
            s_y = self._y_profile.calculate(self._profile_t, yaw_current, yaw_goal)
            s_r = self._r_profile.calculate(self._profile_t, roll_current, roll_goal)

            self._profiled_pitch = s_p.position
            self._profiled_yaw = s_y.position
            self._profiled_roll = s_r.position

        if (
            self._feedback_enabled
            and self._sensors is not None
            and self._sensors.is_zeroed()
            and self._desired_pitch is not None
            and self._desired_yaw is not None
            and self._desired_roll is not None
        ):
            ap, ay, ar = self._sensors.get_euler_angles()

            err_p = self._profiled_pitch - ap
            err_y = self._profiled_yaw - ay
            err_r = self._profiled_roll - ar

            self._integral_pitch += err_p
            self._integral_yaw += err_y
            self._integral_roll += err_r

            il = self._integral_limit
            if il > 0.0:
                self._integral_pitch = max(-il, min(il, self._integral_pitch))
                self._integral_yaw = max(-il, min(il, self._integral_yaw))
                self._integral_roll = max(-il, min(il, self._integral_roll))

            pitch_cmd += self._pitch_kp * err_p + self._pitch_ki * self._integral_pitch
            yaw_cmd += self._yaw_kp * err_y + self._yaw_ki * self._integral_yaw
            roll_cmd += self._roll_kp * err_r + self._roll_ki * self._integral_roll

        self._pitch_servo.setPulseTime(CameraPositioner._n11_to_pulse(pitch_cmd))
        self._yaw_servo.setPulseTime(CameraPositioner._n11_to_pulse(yaw_cmd))
        self._roll_servo.setPulseTime(CameraPositioner._n11_to_pulse(roll_cmd))

        self._publish_telemetry(pitch_cmd, yaw_cmd, roll_cmd, err_p, err_y, err_r)

    def _publish_telemetry(
        self,
        pitch_n11: float | None,
        yaw_n11: float | None,
        roll_n11: float | None,
        err_p: float,
        err_y: float,
        err_r: float,
    ) -> None:
        sd = wpilib.SmartDashboard

        sd.putNumber(
            "positioner/ff_n11_pitch", pitch_n11 if pitch_n11 is not None else 0.0
        )
        sd.putNumber("positioner/ff_n11_yaw", yaw_n11 if yaw_n11 is not None else 0.0)
        sd.putNumber(
            "positioner/ff_n11_roll", roll_n11 if roll_n11 is not None else 0.0
        )

        dp = self._desired_pitch if self._desired_pitch is not None else 0.0
        dy = self._desired_yaw if self._desired_yaw is not None else 0.0
        dr = self._desired_roll if self._desired_roll is not None else 0.0
        sd.putNumberArray("positioner/goal_rpy", [dr, dp, dy])
        sd.putNumberArray(
            "positioner/profiled_rpy",
            [self._profiled_roll, self._profiled_pitch, self._profiled_yaw],
        )

        sd.putNumberArray("positioner/error_rpy", [err_r, err_p, err_y])
