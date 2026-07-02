# formation_logger.py
# ─────────────────────────────────────────────────────────────────────────────
# Week 4 – Formation Logger
# Logs formation state (positions, inter-drone distances, formation accuracy)
# to a JSON-lines file for later analysis and visualization.
# ─────────────────────────────────────────────────────────────────────────────

import json
import logging
import os
from datetime import datetime
from pathlib import Path

# Log file lives alongside the other telemetry logs
LOG_DIR = Path(__file__).resolve().parents[1] / "logs"
FORMATION_LOG_FILE = LOG_DIR / "formation_log.jsonl"


def _ensure_log_dir():
    """Create the log directory if it does not exist."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)


def log_formation_state(
    formation_type: str,
    target_positions: dict,
    actual_positions: dict,
    inter_drone_distances: dict,
    extra: dict = None,
):
    """
    Append a single formation snapshot to the log file.

    Parameters
    ----------
    formation_type : str
        Name of the active formation (e.g. "triangle").
    target_positions : dict
        {drone_id: {"lat": ..., "lon": ..., "alt": ...}} — commanded targets.
    actual_positions : dict
        {drone_id: {"lat": ..., "lon": ..., "alt": ...}} — current positions.
    inter_drone_distances : dict
        {"drone_1<->drone_2": 10.34, ...} — pairwise distances in meters.
    extra : dict, optional
        Any additional metadata (e.g. spacing, heading).
    """
    _ensure_log_dir()

    entry = {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "formation_type": formation_type,
        "target_positions": target_positions,
        "actual_positions": actual_positions,
        "inter_drone_distances": inter_drone_distances,
    }
    if extra:
        entry["extra"] = extra

    try:
        with open(FORMATION_LOG_FILE, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry) + "\n")
        logging.info(
            f"[FormationLogger] Logged formation state → {FORMATION_LOG_FILE}"
        )
    except Exception as e:
        logging.error(f"[FormationLogger] Failed to write log: {e}")


def read_formation_log() -> list:
    """
    Read and return all formation log entries as a list of dicts.
    """
    _ensure_log_dir()
    entries = []
    if FORMATION_LOG_FILE.exists():
        with open(FORMATION_LOG_FILE, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        entries.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
    return entries


def clear_formation_log():
    """Delete the formation log file."""
    _ensure_log_dir()
    if FORMATION_LOG_FILE.exists():
        os.remove(FORMATION_LOG_FILE)
        logging.info("[FormationLogger] Formation log cleared.")
