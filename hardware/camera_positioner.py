"""Three-axis servo positioner that orients the camera in pitch, yaw, and roll.

Supports two smooth command modes (trapezoidal profile with PI feedback, and
linear N11 slew) and a raw bypass for homing.  All state machines advance in
``periodic()`` — no blocking calls.
"""

import ntcore
import wpilib
from wpilib import Timer
from wpimath import Pose3d, Rotation3d, Translation3d, TrapezoidProfile

from config.servo_calibration_map import CalibrationMap
from core.subsystem import Subsystem
from hardware.ground_truth_sensors import GroundTruthSensors


class CameraPositioner(Subsystem):
    """Drives three MG90S servos to position the camera at commanded angles.

    Lifecycle: construct with config values, then call ``set_goal_rad()``,
    ``set_target_n11()``, or ``set_raw_n11()`` to command motion.  The
    ``periodic()`` method advances the active state machine (profile or slew)
    and writes PWM pulses each cycle.  Callers check ``at_goal()`` or
    ``n11_slew_finished()`` to detect completion.
    """

    def __init__(
        self,
        pitch_channel: int,
        yaw_channel: int,
        roll_channel: int,
        pitch_center: float,
        pitch_min: float,
        pitch_max: float,
        yaw_center: float,
        yaw_min: float,
        yaw_max: float,
        roll_center: float,
        roll_min: float,
        roll_max: float,
        sensors: GroundTruthSensors,
        calibration_map: type[CalibrationMap] = CalibrationMap,
        camera_translation: Translation3d = Translation3d(),
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
        """Initialise PWM outputs, calibration map reference, and internal state.

        Args:
            pitch_channel: PWM channel for the pitch servo.
            yaw_channel: PWM channel for the yaw servo.
            roll_channel: PWM channel for the roll servo.
            pitch_center: N11 centre value for pitch.
            pitch_min: N11 soft lower limit for pitch.
            pitch_max: N11 soft upper limit for pitch.
            yaw_center: N11 centre value for yaw.
            yaw_min: N11 soft lower limit for yaw.
            yaw_max: N11 soft upper limit for yaw.
            roll_center: N11 centre value for roll.
            roll_min: N11 soft lower limit for roll.
            roll_max: N11 soft upper limit for roll.
            sensors: Ground-truth sensor instance used for feedback.
            calibration_map: CalibrationMap class for RPY-to-N11 conversion.
            camera_translation: Bench-frame camera translation (from CAD).
            pitch_kp: Proportional gain for pitch feedback.
            pitch_ki: Integral gain for pitch feedback.
            yaw_kp: Proportional gain for yaw feedback.
            yaw_ki: Integral gain for yaw feedback.
            roll_kp: Proportional gain for roll feedback.
            roll_ki: Integral gain for roll feedback.
            integral_limit: Maximum integral accumulator value per axis.
            position_tolerance_rad: Tolerance for ``at_goal()`` convergence check.
            pitch_max_velocity: Max trapezoidal profile velocity for pitch (rad/s).
            pitch_max_acceleration: Max trapezoidal profile acceleration for pitch (rad/s²).
            yaw_max_velocity: Max trapezoidal profile velocity for yaw (rad/s).
            yaw_max_acceleration: Max trapezoidal profile acceleration for yaw (rad/s²).
            roll_max_velocity: Max trapezoidal profile velocity for roll (rad/s).
            roll_max_acceleration: Max trapezoidal profile acceleration for roll (rad/s²).
        """
        super().__init__()

        self._pitch_servo = wpilib.PWM(pitch_channel)
        self._yaw_servo = wpilib.PWM(yaw_channel)
        self._roll_servo = wpilib.PWM(roll_channel)

        self._sensors = sensors
        self._cal = calibration_map

        self._last_timestamp = Timer.getTimestamp()

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
        self._yaw_center = yaw_center
        self._yaw_min = yaw_min
        self._yaw_max = yaw_max
        self._roll_center = roll_center
        self._roll_min = roll_min
        self._roll_max = roll_max

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

        self._profiled_pitch: float = 0.0
        self._profiled_yaw: float = 0.0
        self._profiled_roll: float = 0.0
        self._profiled_pitch_vel: float = 0.0
        self._profiled_yaw_vel: float = 0.0
        self._profiled_roll_vel: float = 0.0

        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0
        self._feedback_enabled = True

        self._n11_slewing: bool = False
        self._n11_slew_start: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._n11_slew_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._n11_slew_start_time: float = 0.0
        self._n11_slew_duration: float = 1.0

        self._camera_translation = camera_translation

        inst = ntcore.NetworkTableInstance.getDefault()
        pt = inst.getTable("positioner")
        self._pub_goal_pose = pt.getStructTopic("goal_pose", Pose3d).publish()
        self._pub_profiled_pose = pt.getStructTopic("profiled_pose", Pose3d).publish()

    @staticmethod
    def _clamp_n11(v: float, lo: float, hi: float) -> float:
        """Clamp a value to the N11 soft limit range [lo, hi]."""
        return max(lo, min(hi, v))

    def set_goal_rad(self, pitch_rad: float, yaw_rad: float, roll_rad: float) -> None:
        """Command a target orientation via trapezoidal profile with PI feedback.

        Uses the inverse calibration map to compute an N11 feedforward value,
        then converges via feedback in subsequent ``periodic()`` cycles.

        Args:
            pitch_rad: Target pitch in radians.
            yaw_rad: Target yaw in radians.
            roll_rad: Target roll in radians.
        """
        self._desired_pitch = pitch_rad
        self._desired_yaw = yaw_rad
        self._desired_roll = roll_rad

        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0
        self._last_timestamp = Timer.getTimestamp()
        self._n11_slewing = False

        sr, sp, sy = self._cal.inverse(pitch_rad, yaw_rad, roll_rad)
        self._ff_pitch_n11 = CameraPositioner._clamp_n11(
            float(sp), self._pitch_min, self._pitch_max
        )
        self._ff_yaw_n11 = CameraPositioner._clamp_n11(
            float(sy), self._yaw_min, self._yaw_max
        )
        self._ff_roll_n11 = CameraPositioner._clamp_n11(
            float(sr), self._roll_min, self._roll_max
        )

    def set_raw_n11(self, pitch: float, yaw: float, roll: float) -> None:
        """Immediately apply raw N11 values to the servos (bypasses profile and slew).

        Any active profile or slew is cancelled.

        Args:
            pitch: Raw N11 value for pitch.
            yaw: Raw N11 value for yaw.
            roll: Raw N11 value for roll.
        """
        self._ff_pitch_n11 = CameraPositioner._clamp_n11(
            float(pitch), self._pitch_min, self._pitch_max
        )
        self._ff_yaw_n11 = CameraPositioner._clamp_n11(
            float(yaw), self._yaw_min, self._yaw_max
        )
        self._ff_roll_n11 = CameraPositioner._clamp_n11(
            float(roll), self._roll_min, self._roll_max
        )
        self._desired_pitch = None
        self._desired_yaw = None
        self._desired_roll = None
        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0
        self._n11_slewing = False

    def set_target_n11(
        self, pitch: float, yaw: float, roll: float, duration_s: float = 1.0
    ) -> None:
        """Command a target in N11 space via a linear slew from the current position.

        Any active profile or feedback goal is cancelled.

        Args:
            pitch: Target N11 value for pitch.
            yaw: Target N11 value for yaw.
            roll: Target N11 value for roll.
            duration_s: Duration of the linear slew in seconds.
        """
        self._desired_pitch = None
        self._desired_yaw = None
        self._desired_roll = None
        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0

        cp, cy, cr = self.get_current_n11()
        self._n11_slew_start = (cp, cy, cr)
        self._n11_slew_target = (
            CameraPositioner._clamp_n11(float(pitch), self._pitch_min, self._pitch_max),
            CameraPositioner._clamp_n11(float(yaw), self._yaw_min, self._yaw_max),
            CameraPositioner._clamp_n11(float(roll), self._roll_min, self._roll_max),
        )
        self._n11_slew_start_time = Timer.getTimestamp()
        self._n11_slew_duration = max(duration_s, 0.001)
        self._n11_slewing = True

    def n11_slew_finished(self) -> bool:
        """Return True if the active N11 slew has completed."""
        return not self._n11_slewing

    def profile_finished(self) -> bool:
        """Return True if the trapezoidal profile has reached its target."""
        if (
            self._desired_pitch is None
            or self._desired_yaw is None
            or self._desired_roll is None
        ):
            return False
        return (
            abs(self._profiled_pitch - self._desired_pitch) < 1e-6
            and abs(self._profiled_yaw - self._desired_yaw) < 1e-6
            and abs(self._profiled_roll - self._desired_roll) < 1e-6
        )

    def at_goal(self) -> bool:
        """Return True if the servos have converged to the commanded orientation.

        When feedback is enabled and the IMU is zeroed, compares IMU readings
        against the desired angles within ``position_tolerance_rad``.  Otherwise
        returns True immediately after the profile/slew completes.
        """
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
        """Re-enable PI feedback correction in ``periodic()``."""
        self._feedback_enabled = True

    def get_current_n11(self) -> tuple[float, float, float]:
        """Return the last-commanded N11 values (pitch, yaw, roll)."""
        return (
            self._ff_pitch_n11 if self._ff_pitch_n11 is not None else 0.0,
            self._ff_yaw_n11 if self._ff_yaw_n11 is not None else 0.0,
            self._ff_roll_n11 if self._ff_roll_n11 is not None else 0.0,
        )

    def disable_feedback(self) -> None:
        """Disable PI feedback and reset the integral accumulators."""
        self._feedback_enabled = False
        self._integral_pitch = 0.0
        self._integral_yaw = 0.0
        self._integral_roll = 0.0

    @staticmethod
    def _n11_to_pulse(v: float) -> int:
        """Convert an N11 value (-1..1) to a PWM pulse width in microseconds."""
        clamped = max(-1.0, min(1.0, v))
        zero_to_one = (clamped + 1.0) / 2.0
        return int(1000 + zero_to_one * 1000)

    def periodic(self) -> None:
        """Advance the active state machine and write PWM outputs.

        Handles three mutually exclusive modes:
        1. N11 linear slew (``set_target_n11``).
        2. Trapezoidal profile with optional PI feedback (``set_goal_rad``).
        3. Raw N11 hold (``set_raw_n11``).
        """
        pitch_cmd = self._ff_pitch_n11
        yaw_cmd = self._ff_yaw_n11
        roll_cmd = self._ff_roll_n11

        err_p = 0.0
        err_y = 0.0
        err_r = 0.0

        if pitch_cmd is None or yaw_cmd is None or roll_cmd is None:
            self._publish_telemetry(pitch_cmd, yaw_cmd, roll_cmd, err_p, err_y, err_r)
            return

        if self._n11_slewing:
            now = Timer.getTimestamp()
            elapsed = now - self._n11_slew_start_time
            frac = min(elapsed / self._n11_slew_duration, 1.0)

            p0, y0, r0 = self._n11_slew_start
            p1, y1, r1 = self._n11_slew_target
            self._ff_pitch_n11 = p0 + frac * (p1 - p0)
            self._ff_yaw_n11 = y0 + frac * (y1 - y0)
            self._ff_roll_n11 = r0 + frac * (r1 - r0)
            pitch_cmd = self._ff_pitch_n11
            yaw_cmd = self._ff_yaw_n11
            roll_cmd = self._ff_roll_n11

            if frac >= 1.0:
                self._n11_slewing = False

        elif (
            self._desired_pitch is not None
            and self._desired_yaw is not None
            and self._desired_roll is not None
        ):
            now = Timer.getTimestamp()
            dt = now - self._last_timestamp
            self._last_timestamp = now

            pitch_goal = TrapezoidProfile.State(self._desired_pitch, 0.0)
            yaw_goal = TrapezoidProfile.State(self._desired_yaw, 0.0)
            roll_goal = TrapezoidProfile.State(self._desired_roll, 0.0)

            pitch_current = TrapezoidProfile.State(
                self._profiled_pitch, self._profiled_pitch_vel
            )
            yaw_current = TrapezoidProfile.State(
                self._profiled_yaw, self._profiled_yaw_vel
            )
            roll_current = TrapezoidProfile.State(
                self._profiled_roll, self._profiled_roll_vel
            )

            s_p = self._p_profile.calculate(dt, pitch_current, pitch_goal)
            s_y = self._y_profile.calculate(dt, yaw_current, yaw_goal)
            s_r = self._r_profile.calculate(dt, roll_current, roll_goal)

            self._profiled_pitch = s_p.position
            self._profiled_pitch_vel = s_p.velocity
            self._profiled_yaw = s_y.position
            self._profiled_yaw_vel = s_y.velocity
            self._profiled_roll = s_r.position
            self._profiled_roll_vel = s_r.velocity

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

        pitch_cmd = CameraPositioner._clamp_n11(
            pitch_cmd, self._pitch_min, self._pitch_max
        )
        yaw_cmd = CameraPositioner._clamp_n11(
            yaw_cmd, self._yaw_min, self._yaw_max
        )
        roll_cmd = CameraPositioner._clamp_n11(
            roll_cmd, self._roll_min, self._roll_max
        )

        self._pitch_servo.setPulseTime(CameraPositioner._n11_to_pulse(pitch_cmd))
        self._yaw_servo.setPulseTime(CameraPositioner._n11_to_pulse(yaw_cmd))
        self._roll_servo.setPulseTime(CameraPositioner._n11_to_pulse(roll_cmd))

        self._publish_telemetry(pitch_cmd, yaw_cmd, roll_cmd, err_p, err_y, err_r)

        dp = self._desired_pitch
        dy = self._desired_yaw
        dr = self._desired_roll
        if dp is not None and dy is not None and dr is not None:
            goal_pose = Pose3d(
                self._camera_translation, Rotation3d(dr, dp, dy)
            )
            self._pub_goal_pose.set(goal_pose)

        profiled_pose = Pose3d(
            self._camera_translation,
            Rotation3d(self._profiled_roll, self._profiled_pitch, self._profiled_yaw),
        )
        self._pub_profiled_pose.set(profiled_pose)

    def _publish_telemetry(
        self,
        pitch_n11: float | None,
        yaw_n11: float | None,
        roll_n11: float | None,
        err_p: float,
        err_y: float,
        err_r: float,
    ) -> None:
        """Write positioner state to SmartDashboard for debugging."""
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
