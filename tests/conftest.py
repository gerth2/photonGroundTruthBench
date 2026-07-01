"""Shared fixtures and CLI options for the test suite."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def pytest_addoption(parser: pytest.Parser) -> None:
    """Register the ``--interactive-calib`` CLI flag for the cross-axis coupling GUI."""
    parser.addoption(
        "--interactive-calib",
        action="store_true",
        default=False,
        help="Open the interactive cross-axis coupling GUI on synthetic calibration data",
    )


@pytest.fixture
def interactive_calib(request: pytest.FixtureRequest) -> bool:
    """Return True if ``--interactive-calib`` was passed and a GUI backend is available; otherwise skip."""
    if not request.config.getoption("--interactive-calib"):
        return False
    try:
        import matplotlib
        import matplotlib.pyplot as plt

        matplotlib.use("TkAgg")
        fig, ax = plt.subplots()
        plt.close(fig)
    except Exception:
        pytest.skip("no TkAgg backend available for interactive calibration GUI")
    return True
