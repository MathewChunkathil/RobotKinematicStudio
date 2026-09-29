#!/usr/bin/env bash
# N-DOF Arm Simulator - Linux / macOS Launcher

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

if [ ! -f ".venv/bin/python" ]; then
    echo "[INFO] First-time setup: creating virtual environment..."
    python3 -m venv .venv
    .venv/bin/pip install --upgrade pip
    .venv/bin/pip install -e .
fi

echo "[INFO] Launching N-DOF Arm Simulator..."
exec .venv/bin/python -m robokinematics.app.main "$@"
