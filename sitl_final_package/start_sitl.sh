#!/bin/bash
# Change to the directory of this script
cd "$(dirname "$0")" || exit 1

export PATH="$PATH:/home/immad_baber/.local/bin"
export SITL_RITW_TERMINAL="sh"
HOST_IP=$(ip route | grep default | awk '{print $3}')

NUM_DRONES=${1:-3}

echo "[INFO] Starting $NUM_DRONES ArduCopter SITL instances..."

for ((i=0; i<NUM_DRONES; i++)); do
  INSTANCE=$i
  SYSID=$((i + 1))
  PORT=$((14551 + i))
  
  # Offset each starting coordinate by ~15 meters to prevent collision
  LAT_OFFSET=$(awk "BEGIN {print 33.6844 + ($i * 0.00015)}")
  LON_OFFSET=$(awk "BEGIN {print 73.0479 + ($i * 0.00015)}")
  
  echo "🚀 Launching Drone $SYSID (Instance $INSTANCE) at Lat=$LAT_OFFSET, Lon=$LON_OFFSET, forwarding to port $PORT..."
  
  nohup python3 /home/immad_baber/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter \
    -I $INSTANCE \
    --custom-location=$LAT_OFFSET,$LON_OFFSET,540,0 \
    --sysid=$SYSID \
    --out=127.0.0.1:$PORT \
    --out=$HOST_IP:$PORT \
    --no-rebuild \
    --use-dir=instance_$INSTANCE \
    --mavproxy-args="--daemon" > "/tmp/sitl_instance_$INSTANCE.log" 2>&1 &
done

echo "[INFO] All $NUM_DRONES instances spawned in the background."