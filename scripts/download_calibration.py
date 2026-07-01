#!/usr/bin/env python3
"""
Download a servo-calibration CSV from the SystemCore, fit a degree-2
polynomial map (forward and inverse), show an interactive cross-axis
coupling GUI, and write the result to config/servo_calibration_map.py.

Usage:
    python scripts/download_calibration.py [systemcore-host]
"""

import argparse
import csv
import os
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

import numpy as np

# ── GUI backend selection ──────────────────────────────────────────────

_HAS_GUI = False
try:
    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, CheckButtons

    _HAS_GUI = True
except Exception:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

print(f"matplotlib backend: {matplotlib.get_backend()}")

# ── Polynomial-fit helpers ─────────────────────────────────────────────

_FEATURE_NAMES = ["1", "r", "p", "y", "r²", "p²", "y²", "r·p", "r·y", "p·y"]


def _make_features(R: np.ndarray, P: np.ndarray, Y: np.ndarray) -> np.ndarray:
    """Build the degree-2 design matrix with cross terms from servo or angle arrays.

    Columns: 1, r, p, y, r², p², y², r·p, r·y, p·y.
    """
    ones = np.ones_like(R)
    return np.column_stack([ones, R, P, Y, R**2, P**2, Y**2, R * P, R * Y, P * Y])


def _fit(X: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
    """Fit a linear model via least squares.

    Returns coefficients, RMS error, and predictions.
    """
    coeffs, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coeffs
    resid = y - pred
    rms = float(np.sqrt(np.mean(resid**2)))
    return coeffs, rms, pred


# ── Remote IO ──────────────────────────────────────────────────────────


def list_remote_files(host: str, remote_dir: str) -> list[str]:
    """List files in a remote directory over SSH.

    Returns full remote paths.  Exits the process on SSH failure.
    """
    result = subprocess.run(
        ["ssh", f"lvuser@{host}", "ls", "-1", remote_dir],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0:
        print(f"Error listing remote files: {result.stderr.strip()}")
        sys.exit(1)
    lines = [line.strip() for line in result.stdout.strip().split("\n") if line.strip()]
    return [os.path.join(remote_dir, line) for line in lines]


def download_file(host: str, remote_path: str, local_path: str) -> None:
    """Download a file from the remote host via SCP."""
    subprocess.run(
        ["scp", f"lvuser@{host}:{remote_path}", local_path],
        check=True,
        timeout=30,
    )


def load_csv(path: str) -> tuple[dict[str, np.ndarray], dict[str, str]]:
    """Return (column_data, metadata) where metadata is parsed from # comments."""
    meta: dict[str, str] = {}
    data_rows: list[str] = []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                if ":" in line:
                    key, _, val = line.lstrip("#").strip().partition(":")
                    meta[key.strip()] = val.strip()
            else:
                data_rows.append(line)
    reader = csv.DictReader(data_rows)
    rows = list(reader)
    data = {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}
    return data, meta


# ── Interactive cross-axis coupling GUI ────────────────────────────────


def interactive_analysis(
    data: dict[str, np.ndarray],
    coeffs_r: np.ndarray,
    coeffs_p: np.ndarray,
    coeffs_y: np.ndarray,
) -> None:
    """Open an interactive GUI with sliders to explore cross-axis coupling.

    For each axis (roll / pitch / yaw):
      - X axis: commanded N11 value
      - Y axis: actual IMU reading (rad)
      - Sliders for the other two axes filter the data in N11 space

    Checkboxes toggle raw data, filtered data points, and error bars.
    The best-fit polynomial line is always visible.
    """
    if not _HAS_GUI:
        print("No interactive GUI backend available; skipping analysis.")
        return

    R, P, Y_s = data["servo_roll"], data["servo_pitch"], data["servo_yaw"]
    AR, AP, AY = (
        data["actual_roll_rad"],
        data["actual_pitch_rad"],
        data["actual_yaw_rad"],
    )

    # Default slider positions (median of each axis)
    default_r = float(np.median(R))
    default_p = float(np.median(P))
    default_y = float(np.median(Y_s))

    fig, axes = plt.subplots(3, 1, figsize=(10, 11))
    fig.subplots_adjust(left=0.12, bottom=0.26, top=0.96)

    # Per-axis config: (name, X_data, Y_data, other1_data, other1_name,
    #                  other2_data, other2_name, coeffs, color)
    axis_cfg = [
        ("Roll", R, AR, P, "pitch", Y_s, "yaw", coeffs_r, "#1f77b4"),
        ("Pitch", P, AP, R, "roll", Y_s, "yaw", coeffs_p, "#ff7f0e"),
        ("Yaw", Y_s, AY, R, "roll", P, "pitch", coeffs_y, "#2ca02c"),
    ]

    store: dict[str, dict] = {}

    for ax, (name, x_raw, y_raw, o1, o1n, o2, o2n, coeffs, color) in zip(
        axes, axis_cfg
    ):
        # Background: all data points
        (raw_scat,) = ax.plot(x_raw, y_raw, ".", color="gray", alpha=0.15, ms=3)

        # Filtered subset
        (filt_scat,) = ax.plot([], [], ".", color=color, alpha=0.85, ms=5, zorder=4)

        # Error bars on binned filtered data
        (eb_line,) = ax.plot([], [], "k.", ms=1, zorder=6)
        eb_container = ax.errorbar([], [], yerr=[], fmt="none", ecolor="k", capsize=3)

        # Best-fit polynomial line (always shown)
        (fit_line,) = ax.plot([], [], "-", color=color, lw=2, zorder=5)

        ax.set_xlabel(f"Commanded {name} N11")
        ax.set_ylabel(f"Actual {name} (rad)")
        ax.set_title(name)
        ax.set_xlim(-1.05, 1.05)
        ax.grid(True, alpha=0.25)

        store[name.lower()] = {
            "x_raw": x_raw,
            "y_raw": y_raw,
            "o1": o1,
            "o1n": o1n,
            "o2": o2,
            "o2n": o2n,
            "coeffs": coeffs,
            "raw_scat": raw_scat,
            "filt_scat": filt_scat,
            "eb_line": eb_line,
            "eb_container": eb_container,
            "fit_line": fit_line,
            "visible_raw": True,
            "visible_filt": True,
            "visible_eb": False,
        }

    # ── Sliders ───────────────────────────────────────────────────
    slider_ax_h = 0.025
    slider_left = 0.25
    slider_width = 0.60

    ax_r = fig.add_axes([slider_left, 0.18, slider_width, slider_ax_h])
    s_r = Slider(ax_r, "Roll N11", -1.0, 1.0, valinit=default_r, valstep=0.01)

    ax_p = fig.add_axes([slider_left, 0.14, slider_width, slider_ax_h])
    s_p = Slider(ax_p, "Pitch N11", -1.0, 1.0, valinit=default_p, valstep=0.01)

    ax_y = fig.add_axes([slider_left, 0.10, slider_width, slider_ax_h])
    s_y = Slider(ax_y, "Yaw N11", -1.0, 1.0, valinit=default_y, valstep=0.01)

    # ── Checkboxes ────────────────────────────────────────────────
    ax_cb = fig.add_axes([0.02, 0.10, 0.18, 0.12])
    cb = CheckButtons(
        ax_cb,
        ["Raw data", "Filtered points", "Error bars"],
        [True, True, False],
    )

    # ── Update callback ───────────────────────────────────────────
    _N_BINS = 20

    def _update(val: object = None) -> None:
        """Recompute filtered data and replot on slider or checkbox change."""
        tol = 0.3
        r_val = s_r.val
        p_val = s_p.val
        y_val = s_y.val

        for name, ax in zip(("roll", "pitch", "yaw"), axes):
            s = store[name]
            xr, yr = s["x_raw"], s["y_raw"]
            o1, o2 = s["o1"], s["o2"]

            # Build filter mask based on which sliders correspond to
            # the two "other" axes for this plot.
            if name == "roll":
                mask = (np.abs(o1 - p_val) < tol) & (np.abs(o2 - y_val) < tol)
            elif name == "pitch":
                mask = (np.abs(o1 - r_val) < tol) & (np.abs(o2 - y_val) < tol)
            else:  # yaw
                mask = (np.abs(o1 - r_val) < tol) & (np.abs(o2 - p_val) < tol)

            x_filt = xr[mask]
            y_filt = yr[mask]

            # Update filtered scatter
            s["filt_scat"].set_data(x_filt, y_filt)

            # Best-fit polynomial through filtered data
            if len(x_filt) > 3:
                xs = np.linspace(max(x_filt.min(), -1.0), min(x_filt.max(), 1.0), 200)
                feat = _make_features(
                    np.full_like(xs, r_val if name != "roll" else xs),
                    np.full_like(xs, p_val if name != "pitch" else xs),
                    np.full_like(xs, y_val if name != "yaw" else xs),
                )
                y_pred = feat @ s["coeffs"]
                s["fit_line"].set_data(xs, y_pred)
            else:
                s["fit_line"].set_data([], [])

            # Error bars via binning
            if len(x_filt) > _N_BINS:
                bins = np.linspace(x_filt.min(), x_filt.max(), _N_BINS + 1)
                bin_centers = (bins[:-1] + bins[1:]) / 2.0
                bin_means = np.zeros(_N_BINS)
                bin_stds = np.zeros(_N_BINS)
                for bi in range(_N_BINS):
                    bin_mask = (x_filt >= bins[bi]) & (x_filt < bins[bi + 1])
                    if bin_mask.sum() > 1:
                        bin_means[bi] = np.mean(y_filt[bin_mask])
                        bin_stds[bi] = np.std(y_filt[bin_mask], ddof=1)
                    else:
                        bin_means[bi] = np.nan
                        bin_stds[bi] = np.nan
                valid = ~np.isnan(bin_means)
                s["eb_container"].remove()
                s["eb_container"] = ax.errorbar(
                    bin_centers[valid],
                    bin_means[valid],
                    yerr=bin_stds[valid],
                    fmt="none",
                    ecolor="k",
                    capsize=3,
                    zorder=6,
                )
            else:
                s["eb_container"].remove()
                s["eb_container"] = ax.errorbar(
                    [], [], yerr=[], fmt="none", ecolor="k", capsize=3
                )

            # Apply visibility toggles
            s["raw_scat"].set_visible(s["visible_raw"])
            s["filt_scat"].set_visible(s["visible_filt"])
            for child in s["eb_container"].get_children():
                child.set_visible(s["visible_eb"])

        fig.canvas.draw_idle()

    def _checkbox_toggle(label: str) -> None:
        """Toggle visibility of raw data, filtered points, or error bars."""
        for name in ("roll", "pitch", "yaw"):
            s = store[name]
            if label == "Raw data":
                s["visible_raw"] = not s["visible_raw"]
            elif label == "Filtered points":
                s["visible_filt"] = not s["visible_filt"]
            elif label == "Error bars":
                s["visible_eb"] = not s["visible_eb"]
        _update()

    cb.on_clicked(_checkbox_toggle)

    for slider in (s_r, s_p, s_y):
        slider.on_changed(_update)

    # Initial render
    _update()

    plt.show()


# ── Static plot for file output ────────────────────────────────────────


def plot_results(
    data: dict[str, np.ndarray],
    pred_r: np.ndarray,
    pred_p: np.ndarray,
    pred_y: np.ndarray,
    rms_r: float,
    rms_p: float,
    rms_y: float,
    out_path: str,
) -> None:
    """Generate a scatter-versus-prediction and residual-histogram figure, saved to PNG.

    Produces a 3x2 figure (one row per axis) showing predicted vs actual
    scatter with a unity line and a histogram of residuals.  Saves to
    *out_path* at 150 DPI.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Servo Calibration — Polynomial Fit (degree 2, cross terms)")

    labels = [
        ("Roll", "actual_roll_rad", pred_r, rms_r),
        ("Pitch", "actual_pitch_rad", pred_p, rms_p),
        ("Yaw", "actual_yaw_rad", pred_y, rms_y),
    ]

    for i, (name, actual_key, pred, rms) in enumerate(labels):
        actual = data[actual_key]
        resid = actual - pred

        ax_scatter = axes[i, 0]
        ax_scatter.scatter(actual, pred, s=8, alpha=0.6)
        lim = [min(actual.min(), pred.min()), max(actual.max(), pred.max())]
        ax_scatter.plot(lim, lim, "r--", lw=1)
        ax_scatter.set_xlabel(f"Actual {name} (rad)")
        ax_scatter.set_ylabel(f"Predicted {name} (rad)")
        ax_scatter.set_title(f"{name} — RMS {rms:.4f} rad")
        ax_scatter.grid(True, alpha=0.3)

        ax_hist = axes[i, 1]
        ax_hist.hist(resid, bins=30, alpha=0.7)
        ax_hist.set_xlabel(f"{name} Residual (rad)")
        ax_hist.set_ylabel("Count")
        ax_hist.set_title(f"{name} Residual Distribution")
        ax_hist.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved to {out_path}")
    plt.close(fig)


# ── Config map code generation ─────────────────────────────────────────


def generate_map_code(
    coeffs_r: np.ndarray,
    coeffs_p: np.ndarray,
    coeffs_y: np.ndarray,
    inv_coeffs_r: np.ndarray,
    inv_coeffs_p: np.ndarray,
    inv_coeffs_y: np.ndarray,
    rms_r: float,
    rms_p: float,
    rms_y: float,
    n_points: int,
    avg_gain: float,
    source_file: str = "<unknown>",
    systemcore_serial: str = "<unknown>",
) -> str:
    """Generate Python source code for a ``CalibrationMap`` class.

    Returns the complete class source as a string, ready to write to
    ``config/servo_calibration_map.py``.
    """
    now = __import__("datetime").datetime.now().strftime("%Y-%m-%d %H:%M")
    host = __import__("socket").gethostname()

    def fmt_list(arr: np.ndarray) -> str:
        """Format a numpy array as a Python list literal string."""
        vals = ", ".join(f"{v:.10f}" for v in arr)
        return f"[{vals}]"

    return textwrap.dedent(f'''\
    """
    Generated by scripts/download_calibration.py — do not edit manually.

    Last updated:  {now}
    Source CSV:    {source_file}
    SystemCore serial:{systemcore_serial}
    Generated on:  {host}
    """


    def _predict(features: list[float], coeffs: list[float]) -> float:
        return sum(c * f for c, f in zip(coeffs, features))


    def _make_features(
        r: float, p: float, y: float
    ) -> list[float]:
        return [1.0, r, p, y, r * r, p * p, y * y, r * p, r * y, p * y]


    class CalibrationMap:
        FORWARD_COEFFS_ROLL = {fmt_list(coeffs_r)}
        FORWARD_COEFFS_PITCH = {fmt_list(coeffs_p)}
        FORWARD_COEFFS_YAW = {fmt_list(coeffs_y)}

        INVERSE_COEFFS_SERVO_R = {fmt_list(inv_coeffs_r)}
        INVERSE_COEFFS_SERVO_P = {fmt_list(inv_coeffs_p)}
        INVERSE_COEFFS_SERVO_Y = {fmt_list(inv_coeffs_y)}

        AVG_GAIN_RAD_PER_SERVO = {avg_gain:.6f}

        N_POINTS = {n_points}
        RMS_ERROR_ROLL = {rms_r:.8f}
        RMS_ERROR_PITCH = {rms_p:.8f}
        RMS_ERROR_YAW = {rms_y:.8f}

        @classmethod
        def forward(
            cls, servo_r: float, servo_p: float, servo_y: float
        ) -> tuple[float, float, float]:
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
            feats = _make_features(roll_rad, pitch_rad, yaw_rad)
            return (
                _predict(feats, cls.INVERSE_COEFFS_SERVO_R),
                _predict(feats, cls.INVERSE_COEFFS_SERVO_P),
                _predict(feats, cls.INVERSE_COEFFS_SERVO_Y),
            )
    ''')


# ── Main ───────────────────────────────────────────────────────────────


def main() -> None:
    """CLI entry point: parse args, download CSV, fit polynomial, display plots, write config."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "host",
        nargs="?",
        default="systemcore-6708.local",
        help="SystemCore hostname or IP",
    )
    parser.add_argument(
        "--remote-dir",
        default="/home/lvuser/calibration_data",
        help="remote directory with calibration CSV files",
    )
    args = parser.parse_args()

    print(f"Listing CSV files on {args.host}:{args.remote_dir} ...")
    remote_files = list_remote_files(args.host, args.remote_dir)
    csv_files = [fn for fn in remote_files if fn.endswith(".csv")]
    if not csv_files:
        print("No CSV files found.")
        sys.exit(1)

    print("Available calibration files:")
    for i, fn in enumerate(csv_files):
        print(f"  [{i}] {fn}")
    choice = int(input("Select file #: "))
    if choice < 0 or choice >= len(csv_files):
        print("Invalid selection.")
        sys.exit(1)
    remote_path = csv_files[choice]

    with tempfile.TemporaryDirectory() as tmpdir:
        local_path = os.path.join(tmpdir, "calib.csv")
        print(f"Downloading {remote_path} ...")
        download_file(args.host, remote_path, local_path)

        data, meta = load_csv(local_path)
    n_points = len(next(iter(data.values())))
    print(f"Loaded {n_points} data points.")

    R, P, Y_s = data["servo_roll"], data["servo_pitch"], data["servo_yaw"]
    AR, AP, AY = (
        data["actual_roll_rad"],
        data["actual_pitch_rad"],
        data["actual_yaw_rad"],
    )

    X_servo = _make_features(R, P, Y_s)

    coeffs_r, rms_r, pred_r = _fit(X_servo, AR)
    coeffs_p, rms_p, pred_p = _fit(X_servo, AP)
    coeffs_y, rms_y, pred_y = _fit(X_servo, AY)

    X_angle = _make_features(AR, AP, AY)
    inv_cr, inv_rms_r, _ = _fit(X_angle, R)
    inv_cp, inv_rms_p, _ = _fit(X_angle, P)
    inv_cy, inv_rms_y, _ = _fit(X_angle, Y_s)

    avg_gain = float(np.mean(np.abs(AR - AP + AY) / (np.abs(R + P + Y_s) + 1e-8)))

    print("\n=== Forward map (servo → angle) ===")
    print(f"  Roll RMS:  {rms_r:.6f} rad")
    print(f"  Pitch RMS: {rms_p:.6f} rad")
    print(f"  Yaw RMS:   {rms_y:.6f} rad")
    print("\n=== Inverse map (angle → servo) ===")
    print(f"  Roll RMS:  {inv_rms_r:.6f} servo units")
    print(f"  Pitch RMS: {inv_rms_p:.6f} servo units")
    print(f"  Yaw RMS:   {inv_rms_y:.6f} servo units")

    # Interactive cross-axis coupling analysis
    interactive_analysis(data, coeffs_r, coeffs_p, coeffs_y)

    # Static plot for file output
    plot_path = os.path.join(os.path.dirname(__file__) or ".", "calibration_fit.png")
    plot_results(data, pred_r, pred_p, pred_y, rms_r, rms_p, rms_y, plot_path)

    print(f"\nAverage gain: {avg_gain:.4f} rad/servo-unit")

    approved = input("\nWrite config/servo_calibration_map.py? [y/N] ")
    if approved.strip().lower() != "y":
        print("Aborted.")
        sys.exit(0)

    repo_root = Path(__file__).resolve().parent.parent
    out_path = repo_root / "config" / "servo_calibration_map.py"
    source_name = os.path.basename(remote_path)
    systemcore_serial = meta.get("SystemCore serial", "<unknown>")
    code = generate_map_code(
        coeffs_r,
        coeffs_p,
        coeffs_y,
        inv_cr,
        inv_cp,
        inv_cy,
        rms_r,
        rms_p,
        rms_y,
        n_points,
        avg_gain,
        source_file=source_name,
        systemcore_serial=systemcore_serial,
    )
    with open(out_path, "w") as f:
        f.write(code)
    print(f"Written to {out_path}")


if __name__ == "__main__":
    main()
