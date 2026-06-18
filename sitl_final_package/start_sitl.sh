#!/bin/bash
echo "[INFO] Starting ArduCopter SITL with dual MAVLink outputs..."

#14550 is the default port for QGroundControl
#14551 is the additional port for SITL

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter \
  --location=islamabad \
  --out=127.0.0.1:14550 \
  --out=127.0.0.1:14551
#  --no-mavproxy