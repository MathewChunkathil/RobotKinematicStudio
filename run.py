"""Convenience entry point for N-DOF Arm Simulator (RoboKinematics Studio).

Can be executed via:
    python run.py
or by double-clicking on Windows if Python is registered with .py files.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Add src directory to path if running uninstalled
_ROOT = Path(__file__).resolve().parent
_SRC = _ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from robokinematics.app.main import main

if __name__ == "__main__":
    main()
