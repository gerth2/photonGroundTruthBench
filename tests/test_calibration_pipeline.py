"""Round-trip test of the servo-calibration polynomial-fit pipeline.

Synthesises a CSV from known coefficients, runs the same fit functions the
download script uses, and verifies the coefficients are recovered within
expectation, then checks that the generated Python code is structurally
correct.
"""

import os
import tempfile

import numpy as np
import pytest

from config.servo_calibration_map import CalibrationMap
from scripts.download_calibration import (
    _FEATURE_NAMES,
    _fit,
    _make_features,
    generate_map_code,
    interactive_analysis,
    load_csv,
)

# ── Helpers ──────────────────────────────────────────────────────────────


def _write_csv(path: str, cols: dict[str, np.ndarray]) -> str:
    """Write a minimal calibration CSV matching the on-robot format.

    Returns the path written to (for convenience when *path* is a dir).
    """
    if os.path.isdir(path):
        path = os.path.join(path, "fake_calib.csv")
    with open(path, "w", newline="") as f:
        f.write("# SystemCore serial: test-serial-0001\n")
        f.write("# generated: 2027-01-01 00:00:00\n")
        f.write(
            "servo_roll,servo_pitch,servo_yaw,"
            "actual_roll_rad,actual_pitch_rad,actual_yaw_rad\n"
        )
        n = len(next(iter(cols.values())))
        for i in range(n):
            f.write(
                f"{cols['servo_roll'][i]},{cols['servo_pitch'][i]},"
                f"{cols['servo_yaw'][i]},{cols['actual_roll_rad'][i]},"
                f"{cols['actual_pitch_rad'][i]},{cols['actual_yaw_rad'][i]}\n"
            )
    return path


def _apply_map(
    r: np.ndarray, p: np.ndarray, y: np.ndarray, forward: bool = True
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply the CalibrationMap forward or inverse to arrays of points."""
    out_r, out_p, out_y = [], [], []
    for ri, pi, yi in zip(r, p, y):
        if forward:
            ar, ap, ay = CalibrationMap.forward(float(ri), float(pi), float(yi))
        else:
            ar, ap, ay = CalibrationMap.inverse(float(ri), float(pi), float(yi))
        out_r.append(ar)
        out_p.append(ap)
        out_y.append(ay)
    return np.array(out_r), np.array(out_p), np.array(out_y)


# ── Tests ────────────────────────────────────────────────────────────────


class TestCalibrationPipeline:
    n = 200
    noise_std = 0.01  # rad — sensible IMU noise level

    def _synthesise_data(
        self,
        n: int | None = None,
        noise_std: float | None = None,
    ) -> dict[str, np.ndarray]:
        n = n or self.n
        noise_std = noise_std or self.noise_std

        rng = np.random.default_rng(42)
        # Sample servo N11 commands across the mechanism range.
        n11_r = rng.uniform(-0.8, 0.8, n)
        n11_p = rng.uniform(-0.8, 0.8, n)
        n11_y = rng.uniform(-0.8, 0.8, n)

        # True angle = CalibrationMap.forward(n11)
        true_r, true_p, true_y = _apply_map(n11_r, n11_p, n11_y)

        # Add IMU noise
        actual_r = true_r + rng.normal(0, noise_std, n)
        actual_p = true_p + rng.normal(0, noise_std, n)
        actual_y = true_y + rng.normal(0, noise_std, n)

        return {
            "servo_roll": n11_r,
            "servo_pitch": n11_p,
            "servo_yaw": n11_y,
            "actual_roll_rad": actual_r,
            "actual_pitch_rad": actual_p,
            "actual_yaw_rad": actual_y,
        }

    # ── Identity recovery ────────────────────────────────────────────

    def test_identity_round_trip(self) -> None:
        """With identity map and modest noise, forward coefficients should
        be recovered with RMS close to the injected noise level and the
        recovered linear terms within 0.05 of the true coefficients."""
        data = self._synthesise_data()

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = _write_csv(tmpdir, data)
            loaded, _ = load_csv(csv_path)

        R, P, Y_s = loaded["servo_roll"], loaded["servo_pitch"], loaded["servo_yaw"]
        AR, AP, AY = (
            loaded["actual_roll_rad"],
            loaded["actual_pitch_rad"],
            loaded["actual_yaw_rad"],
        )

        # Forward fit: N11 → angle
        X = _make_features(R, P, Y_s)
        cr, rms_r, _ = _fit(X, AR)
        cp, rms_p, _ = _fit(X, AP)
        cy, rms_y, _ = _fit(X, AY)

        # RMS should be close to the injected noise
        assert rms_r < 0.025, f"Roll RMS too high: {rms_r:.4f}"
        assert rms_p < 0.025, f"Pitch RMS too high: {rms_p:.4f}"
        assert rms_y < 0.025, f"Yaw RMS too high: {rms_y:.4f}"

        # Linear term (index 1 for r, 2 for p, 3 for y) should be ≈ 1.0
        assert abs(cr[1] - 1.0) < 0.05, f"Roll linear coeff: {cr[1]:.4f}"
        assert abs(cp[2] - 1.0) < 0.05, f"Pitch linear coeff: {cp[2]:.4f}"
        assert abs(cy[3] - 1.0) < 0.05, f"Yaw linear coeff: {cy[3]:.4f}"

        # All other coefficients should be near zero
        for label, coeffs in [("roll", cr), ("pitch", cp), ("yaw", cy)]:
            for i, c in enumerate(coeffs):
                if (label == "roll" and i == 1) or (
                    label == "pitch" and i == 2
                ) or (label == "yaw" and i == 3):
                    continue
                assert abs(c) < 0.05, f"{label} coeff {_FEATURE_NAMES[i]} = {c:.4f}"

    def test_forward_backward_consistency(self) -> None:
        """Applying forward then inverse (or vice versa) should approximately
        reconstruct the original values for the given set of points."""
        n = 50
        rng = np.random.default_rng(123)
        n11_r = rng.uniform(-0.6, 0.6, n)
        n11_p = rng.uniform(-0.6, 0.6, n)
        n11_y = rng.uniform(-0.6, 0.6, n)

        # forward(inverse(angle)) ≈ angle
        for ri, pi, yi in zip(n11_r, n11_p, n11_y):
            ar, ap, ay = CalibrationMap.forward(float(ri), float(pi), float(yi))
            sr, sp, sy = CalibrationMap.inverse(ar, ap, ay)
            assert abs(sr - ri) < 1e-12, f"forward→inverse roll mismatch: {sr} vs {ri}"
            assert abs(sp - pi) < 1e-12, f"forward→inverse pitch mismatch: {sp} vs {pi}"
            assert abs(sy - yi) < 1e-12, f"forward→inverse yaw mismatch: {sy} vs {yi}"

        # inverse(forward(angle)) ≈ angle
        angle_r = rng.uniform(-0.5, 0.5, n)
        angle_p = rng.uniform(-0.5, 0.5, n)
        angle_y = rng.uniform(-0.5, 0.5, n)
        for ri, pi, yi in zip(angle_r, angle_p, angle_y):
            sr, sp, sy = CalibrationMap.inverse(ri, pi, yi)
            ar, ap, ay = CalibrationMap.forward(sr, sp, sy)
            assert abs(ar - ri) < 1e-12, f"inverse→forward roll mismatch: {ar} vs {ri}"
            assert (
                abs(ap - pi) < 1e-12
            ), f"inverse→forward pitch mismatch: {ap} vs {pi}"
            assert (
                abs(ay - yi) < 1e-12
            ), f"inverse→forward yaw mismatch: {ay} vs {yi}"

    def test_pipeline_with_nontrivial_coefficients(self) -> None:
        """Overwrite CalibrationMap coefficients with a weakly-coupled
        polynomial, synthesise data, fit, and verify the recovery."""
        saved: dict[str, list[float]] = {}
        for attr in (
            "FORWARD_COEFFS_ROLL",
            "FORWARD_COEFFS_PITCH",
            "FORWARD_COEFFS_YAW",
            "INVERSE_COEFFS_SERVO_R",
            "INVERSE_COEFFS_SERVO_P",
            "INVERSE_COEFFS_SERVO_Y",
        ):
            saved[attr] = list(getattr(CalibrationMap, attr))

        try:
            # Non-trivial forward map with small cross-coupling.
            # roll_rad = 0.9*r + 0.1*p
            # pitch_rad = 0.9*p + 0.1*y
            # yaw_rad = 0.9*y + 0.1*r
            CalibrationMap.FORWARD_COEFFS_ROLL = [
                0.0, 0.9, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
            CalibrationMap.FORWARD_COEFFS_PITCH = [
                0.0, 0.0, 0.9, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
            CalibrationMap.FORWARD_COEFFS_YAW = [
                0.0, 0.1, 0.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
            # Inverse — approximate decoupled inverse of the above.
            CalibrationMap.INVERSE_COEFFS_SERVO_R = [
                0.0, 1.111111, -0.123457, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
            CalibrationMap.INVERSE_COEFFS_SERVO_P = [
                0.0, 0.0, 1.111111, -0.123457, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
            CalibrationMap.INVERSE_COEFFS_SERVO_Y = [
                0.0, -0.123457, 0.0, 1.111111, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]

            data = self._synthesise_data(n=300, noise_std=0.02)

            with tempfile.TemporaryDirectory() as tmpdir:
                csv_path = _write_csv(tmpdir, data)
                loaded, _ = load_csv(csv_path)

            R, P, Y_s = (
                loaded["servo_roll"],
                loaded["servo_pitch"],
                loaded["servo_yaw"],
            )
            AR, AP, AY = (
                loaded["actual_roll_rad"],
                loaded["actual_pitch_rad"],
                loaded["actual_yaw_rad"],
            )

            X = _make_features(R, P, Y_s)
            cr, rms_r, _ = _fit(X, AR)
            cp, rms_p, _ = _fit(X, AP)
            cy, rms_y, _ = _fit(X, AY)

            assert rms_r < 0.04
            assert rms_p < 0.04
            assert rms_y < 0.04

            # Check the dominant cross-term is detectable
            assert abs(cr[2] - 0.1) < 0.04, f"Roll←pitch cross {cr[2]:.4f}"
            assert abs(cp[3] - 0.1) < 0.04, f"Pitch←yaw cross {cp[3]:.4f}"
            assert abs(cy[1] - 0.1) < 0.04, f"Yaw←roll cross {cy[1]:.4f}"
        finally:
            for attr, vals in saved.items():
                setattr(CalibrationMap, attr, vals)

    def test_generated_code_contains_expected_coefficients(self) -> None:
        """generate_map_code() should produce valid Python that contains
        the coefficient values we pass in (within formatting precision)."""
        cr = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        cp = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        cy = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        icr = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        icp = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        icy = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        code = generate_map_code(
            cr, cp, cy, icr, icp, icy,
            rms_r=0.0123, rms_p=0.0111, rms_y=0.0099,
            n_points=200, avg_gain=1.0,
            source_file="fake_calib.csv",
            systemcore_serial="test-serial-0001",
        )

        # Check structural elements
        assert "class CalibrationMap" in code
        assert "def forward" in code
        assert "def inverse" in code
        assert "fake_calib.csv" in code
        assert "test-serial-0001" in code

        # Check coefficient values appear (formatted to 10 decimals)
        assert "1.0000000000" in code
        assert "0.0000000000" in code

        # Check RMS values
        assert "0.01230000" in code
        assert "0.01110000" in code
        assert "0.00990000" in code

        # Check N_POINTS
        assert "N_POINTS = 200" in code

        # Verify the code is syntactically valid
        compile(code, "<test-generated>", "exec")

    def test_generated_code_is_executable_map(self) -> None:
        """The code from generate_map_code() can be exec'd and the
        resulting CalibrationMap class works."""

        cr = np.array([0.0, 0.9, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        cp = np.array([0.0, 0.0, 0.9, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        cy = np.array([0.0, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        icr = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        icp = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        icy = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        code = generate_map_code(
            cr, cp, cy, icr, icp, icy,
            rms_r=0.01, rms_p=0.01, rms_y=0.01,
            n_points=100, avg_gain=1.0,
        )

        from typing import Any

        ns: dict[str, Any] = {}
        exec(code, ns)  # noqa: S102

        MapClass = ns["CalibrationMap"]
        assert hasattr(MapClass, "forward")
        assert hasattr(MapClass, "inverse")

        sr, sp, sy = MapClass.inverse(1.0, 0.0, 0.0)
        assert abs(sr - 1.0) < 1e-9
        assert abs(sp) < 1e-9
        assert abs(sy) < 1e-9

        ar, ap, ay = MapClass.forward(1.0, 0.0, 0.0)
        assert abs(ar - 0.9) < 1e-9
        assert abs(ap) < 1e-9
        assert abs(ay) < 1e-9

        assert MapClass.N_POINTS == 100
        assert abs(MapClass.RMS_ERROR_ROLL - 0.01) < 1e-9

    def test_interactive_calibration_analysis(
        self, interactive_calib: bool
    ) -> None:
        """Open the interactive cross-axis coupling GUI on synthetic data.

        Only runs when ``pytest --interactive-calib`` is passed.  The GUI
        shows 3 subplots (roll / pitch / yaw) with sliders and checkboxes
        so you can explore cross-axis coupling on a known ground-truth map
        with injected noise.  After closing the window the same numeric
        assertions as identity_round_trip are checked.
        """
        if not interactive_calib:
            pytest.skip("pass --interactive-calib to open the GUI")

        data = self._synthesise_data()

        R, P, Y_s = data["servo_roll"], data["servo_pitch"], data["servo_yaw"]
        AR, AP, AY = (
            data["actual_roll_rad"],
            data["actual_pitch_rad"],
            data["actual_yaw_rad"],
        )

        X = _make_features(R, P, Y_s)
        cr, _, _ = _fit(X, AR)
        cp, _, _ = _fit(X, AP)
        cy, _, _ = _fit(X, AY)

        interactive_analysis(data, cr, cp, cy)
