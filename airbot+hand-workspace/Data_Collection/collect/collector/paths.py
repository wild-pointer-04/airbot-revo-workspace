"""Resolve Data_Collection roots and put vendor on sys.path."""

from __future__ import annotations

import sys
from pathlib import Path

COLLECT_DIR = Path(__file__).resolve().parents[1]
DATA_COLLECTION_ROOT = COLLECT_DIR.parent
VENDOR_LEROBOT_SRC = DATA_COLLECTION_ROOT / "vendor" / "lerobot-main" / "src"
DEFAULT_DATA_ROOT = DATA_COLLECTION_ROOT / "data"


def ensure_imports() -> None:
    for path in (COLLECT_DIR, VENDOR_LEROBOT_SRC):
        text = str(path)
        if path.exists() and text not in sys.path:
            sys.path.insert(0, text)
