#!/bin/bash
# Run the GCS telemetry WebSocket server
# This script works both inside WSL and on native Windows paths.

# Resolve the directory where this script lives and cd there
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

# ----------------------------------------------------------------
# Create a Python virtual environment if it does not already exist
# ----------------------------------------------------------------
if [ ! -d venv ]; then
    echo "[INFO] Creating Python virtual environment…"
    python3 -m venv venv
fi

# Activate the virtual environment
if [ -f venv/Scripts/activate ]; then
    source venv/Scripts/activate
elif [ -f venv/bin/activate ]; then
    source venv/bin/activate
elif [ -f ../venv/Scripts/activate ]; then
    source ../venv/Scripts/activate
elif [ -f ../venv/bin/activate ]; then
    source ../venv/bin/activate
else
    echo "[WARNING] Broken or missing virtual environment. Recreating..."
    rm -rf venv
    python3 -m venv venv
    if [ -f venv/Scripts/activate ]; then
        source venv/Scripts/activate
    else
        source venv/bin/activate
    fi
fi

# ------------------------------------------------------------
# Install required Python packages (quietly, idempotent)
# ------------------------------------------------------------
pip install -r requirements.txt >/dev/null 2>&1

# ------------------------------------------------------------
# Launch the telemetry server on all interfaces, port 5000
# ------------------------------------------------------------
python -u "$(pwd)/mavlink_integration/telemetry_server.py" &

echo "Telemetry server started on http://0.0.0.0:5000"
