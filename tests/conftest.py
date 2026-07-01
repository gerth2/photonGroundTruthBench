import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--interactive-calib",
        action="store_true",
        default=False,
        help="Open the interactive cross-axis coupling GUI on synthetic calibration data",
    )


@pytest.fixture
def interactive_calib(request: pytest.FixtureRequest) -> bool:
    if not request.config.getoption("--interactive-calib"):
        return False
    # Verify a usable GUI backend exists before the test runs.
    try:
        import matplotlib
        import matplotlib.pyplot as plt

        matplotlib.use("TkAgg")
        fig, ax = plt.subplots()
        plt.close(fig)
    except Exception:
        pytest.skip("no TkAgg backend available for interactive calibration GUI")
    return True
