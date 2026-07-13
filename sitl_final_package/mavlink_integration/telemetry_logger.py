import os
import json
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parents[2] / "logs"

def init_log_dir():
    """Ensure the log directory exists."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)

def _log_file_path(drone_id: str) -> Path:
    return LOG_DIR / f"{drone_id}_telemetry.log"

def append_log(drone_id: str, payload: dict):
    """Append a JSON line to the per‑drone telemetry log."""
    init_log_dir()
    log_path = _log_file_path(drone_id)
    with open(log_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload) + "\n")

def combine_logs() -> str:
    """Combine all per‑drone logs into a single swarm log file.
    Returns the path to the combined log.
    """
    init_log_dir()
    combined_path = LOG_DIR / "swarm_telemetry_combined.log"
    with open(combined_path, "w", encoding="utf-8") as combined:
        for log_file in LOG_DIR.glob("*_telemetry.log"):
            with open(log_file, "r", encoding="utf-8") as lf:
                for line in lf:
                    combined.write(line)
    return str(combined_path)
