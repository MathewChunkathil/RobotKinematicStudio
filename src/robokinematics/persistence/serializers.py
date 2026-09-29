"""JSON persistence helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TypeVar

from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


def save_model(model: BaseModel, path: str | Path) -> None:
    Path(path).write_text(
        json.dumps(model.model_dump(mode="json"), indent=2),
        encoding="utf-8",
    )


def load_model(model_type: type[T], path: str | Path) -> T:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return model_type.model_validate(data)
