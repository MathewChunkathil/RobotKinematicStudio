# N-DOF Arm Simulator (RoboKinematics Studio)

An interactive, high-performance serial robotic manipulator kinematics simulator and engineering design suite built with Python, PyQt6, and OpenGL.

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![GUI: PyQt6 + OpenGL](https://img.shields.io/badge/GUI-PyQt6%20%2B%20OpenGL-green.svg)](https://riverbankcomputing.com/software/pyqt/)

---

## ✨ Key Features

- **60 FPS GPU-Accelerated 3D Viewport**: Smooth rendering using `pyqtgraph.opengl` with closed-cap cylinder links, sphere joints, dynamic coordinate frame triads, and grid reference.
- **Forward Kinematics (FK)**: Real-time slider-driven joint manipulation with interactive 60 FPS animation, hardware-accelerated joint updates, and immediate end-effector pose tracking.
- **Inverse Kinematics (IK) Engine**:
  - Analytical and numerical (Levenberg-Marquardt / Damped Least Squares) IK solvers.
  - Multi-candidate solution generation and ranking.
  - **Ghost Overlays**: Semi-transparent holographic rendering of alternative IK branch solutions in the 3D viewport.
- **Jacobian & Singularity Analysis**:
  - Real-time $6 \times N$ geometric Jacobian calculation.
  - Singular value decomposition (SVD), manipulability index ($\sqrt{\det(J J^T)}$), condition number, and singularity warnings.
- **Trajectory Generation & Playback**:
  - Cartesian linear interpolation and joint-space cubic/quintic polynomial splines.
  - 3D ribbon trajectory path visualization with play, pause, stop, and scrub controls.
- **Workspace Reachability Cloud**: Point cloud generation for reachable workspace estimation.
- **Configurable Robot Architectures**: Pre-configured standard robots (UR5, PUMA 560, SCARA, 2-Link planar) plus a live DH parameter editor.
- **Recent Additions & Roadmap**: See [NEW_FEATURES.md](NEW_FEATURES.md) for dark/light theme details, FK mathematical suite, realistic workcell updates, and future plans.

---

## 🚀 Quick Start

### ⚡ 1-Click Launch (No Terminal Needed!)

- **Windows**: Simply double-click **`run.bat`** (or `run.py`).
  > *On first run, `run.bat` will automatically detect Python, configure `.venv`, install required dependencies, and launch the application seamlessly without keeping a console window open.*
- **Linux / macOS**: Double-click or run:
  ```bash
  ./run.sh
  ```

---

### Manual Setup (Optional)

1. **Clone the repository:**
   ```bash
   git clone https://github.com/MathewChunkathil/ndof-arm-simulator.git
   cd ndof-arm-simulator
   ```

2. **Create and activate a virtual environment:**
   ```bash
   python -m venv .venv
   # Windows PowerShell:
   .venv\Scripts\Activate.ps1
   # macOS/Linux:
   source .venv/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -e ".[dev]"
   ```

4. **Launch the simulator:**
   ```bash
   python -m robokinematics.app.main
   ```

---

## 🧪 Testing

Run the automated test suite (30 unit & numerical IK/Jacobian tests):

```bash
pytest
```

---

## 🛠️ Architecture

```
src/robokinematics/
├── app/             # Application lifecycle, state controller & thread management
├── domain/          # Core models (RobotDefinition, DH parameters, poses, limits)
├── robotics/        # Kinematic math (FK, IK solvers, Jacobian, trajectory, workspace)
├── ui/              # PyQt6 dark-theme GUI (3D viewport, FK sliders, inspectors, HUD)
└── visualization/   # GPU-accelerated OpenGL scene graph (meshes, ghosts, clouds, ribbons)
```

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
