"""Degree-2 polynomial maps for converting between servo N11 commands and joint angles.

Generated offline by ``scripts/download_calibration.py`` after a calibration
sweep.  The forward map (servo -> angle) and inverse map (angle -> servo) are
both degree-2 polynomials in three variables.
"""


def _predict(features: list[float], coeffs: list[float]) -> float:
    """Evaluate a degree-2 polynomial given feature vector and coefficients."""
    return sum(c * f for c, f in zip(coeffs, features))


def _make_features(r: float, p: float, y: float) -> list[float]:
    """Build the degree-2 feature vector [1, r, p, y, r², p², y², rp, ry, py]."""
    return [1.0, r, p, y, r * r, p * p, y * y, r * p, r * y, p * y]


class CalibrationMap:
    """Bidirectional degree-2 polynomial maps between servo N11 and radian angles.

    Class attributes hold the fitted polynomial coefficients (forward and
    inverse for each axis) along with summary statistics from the fit.
    Callers use the ``forward()`` and ``inverse()`` classmethods to convert.
    """

    FORWARD_COEFFS_ROLL: list[float] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    FORWARD_COEFFS_PITCH: list[float] = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    FORWARD_COEFFS_YAW: list[float] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    INVERSE_COEFFS_SERVO_R: list[float] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    INVERSE_COEFFS_SERVO_P: list[float] = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    INVERSE_COEFFS_SERVO_Y: list[float] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    AVG_GAIN_RAD_PER_SERVO: float = 1.0

    N_POINTS: int = 0
    RMS_ERROR_ROLL: float = 0.0
    RMS_ERROR_PITCH: float = 0.0
    RMS_ERROR_YAW: float = 0.0

    @classmethod
    def forward(
        cls, servo_r: float, servo_p: float, servo_y: float
    ) -> tuple[float, float, float]:
        """Map servo N11 commands to estimated joint angles in radians."""
        feats = _make_features(servo_r, servo_p, servo_y)
        return (
            _predict(feats, cls.FORWARD_COEFFS_ROLL),
            _predict(feats, cls.FORWARD_COEFFS_PITCH),
            _predict(feats, cls.FORWARD_COEFFS_YAW),
        )

    @classmethod
    def inverse(
        cls, roll_rad: float, pitch_rad: float, yaw_rad: float
    ) -> tuple[float, float, float]:
        """Map desired joint angles in radians to required servo N11 commands."""
        feats = _make_features(roll_rad, pitch_rad, yaw_rad)
        return (
            _predict(feats, cls.INVERSE_COEFFS_SERVO_R),
            _predict(feats, cls.INVERSE_COEFFS_SERVO_P),
            _predict(feats, cls.INVERSE_COEFFS_SERVO_Y),
        )
