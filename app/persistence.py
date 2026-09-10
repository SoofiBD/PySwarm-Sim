"""Centralized JSON persistence with file locking."""

import json
import os
from pathlib import Path
from typing import Any

try:
    import fcntl  # Unix only
    _HAS_FCNTL = True
except ImportError:
    _HAS_FCNTL = False


def _load_list(file_path: str | Path) -> list:
    if not os.path.exists(file_path):
        return []
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            if _HAS_FCNTL:
                try:
                    fcntl.flock(f, fcntl.LOCK_SH)
                except Exception:
                    pass
            data = json.load(f)
            return data if isinstance(data, list) else []
    except (json.JSONDecodeError, OSError):
        return []


def _save_list(file_path: str | Path, data: list) -> None:
    Path(file_path).parent.mkdir(parents=True, exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        if _HAS_FCNTL:
            try:
                fcntl.flock(f, fcntl.LOCK_EX)
            except Exception:
                pass
        json.dump(data, f, indent=4, sort_keys=True)


def upsert_by_key(file_path: str | Path, key: str, key_value: Any, record: dict) -> None:
    data = _load_list(file_path)
    for i, item in enumerate(data):
        if isinstance(item, dict) and item.get(key) == key_value:
            data[i] = record
            break
    else:
        data.append(record)
    _save_list(file_path, data)
