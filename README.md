# RoboKinematics Studio

A configurable serial robotic-manipulator engineering application.

## Core workflow

Configure robot → DH model → FK → place target → IK → validate → analyze → recommend → ghost alternatives → trajectory → playback → calculation inspection.

## Current status

This repository is an **engineering scaffold**. The specifications are complete, but the robotics engine and polished UI are intentionally implemented phase-by-phase rather than generated as one giant code dump.

## Read first

- `00_MASTER_SPEC.md`
- `01_REQUIREMENTS.md`
- `02_ROBOTICS_SPEC.md`
- `03_DATA_MODEL.md`
- `04_ARCHITECTURE.md`
- `05_VISUAL_UX_SPEC.md`
- `06_TECH_STACK.md`
- `07_TEST_PLAN.md`
- `08_DEVELOPMENT_PLAN.md`
- `09_ANTIGRAVITY_INSTRUCTIONS.md`
- `10_ACCEPTANCE_TESTS.md`
- `11_OUT_OF_SCOPE.md`

## Setup

Create a virtual environment and install development dependencies:

```bash
python -m venv .venv
# Windows PowerShell:
.venv\Scripts\Activate.ps1
# macOS/Linux:
# source .venv/bin/activate

pip install -e ".[dev]"
```

Run tests:

```bash
pytest
```

Run lint:

```bash
ruff check .
```

Run the application scaffold:

```bash
robokinematics
```

## Development rule

Do not ask an agent to build the entire project in one pass. Implement one development phase at a time and verify it.
