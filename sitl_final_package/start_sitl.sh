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

  # --- Clean up instance directory to force a fresh spawn location ---
  # If we don't wipe it, the drone spawns wherever it landed previously.
  if [ -d "instance_$INSTANCE" ]; then
    echo "[INFO] Wiping old instance_$INSTANCE directory..."
    rm -rf "instance_$INSTANCE"
  fi
  mkdir -p "instance_$INSTANCE"

  # Copy eeprom.bin to provide default parameters
  if [ -f "eeprom.bin" ]; then
    cp eeprom.bin "instance_$INSTANCE/eeprom.bin"
    echo "[INFO] Copied eeprom.bin to instance_$INSTANCE/"
  fi

  # Calculate bearing angle to the first waypoint to rotate formation along flight path
  BEARING=$(awk "BEGIN {
    lat1 = 33.6844; lon1 = 73.0479;
    lat2 = 33.665137; lon2 = 73.027023;
    pi = 3.14159265;
    dy = lat2 - lat1;
    dx = (lon2 - lon1) * cos(lat1 * pi / 180.0);
    print atan2(dx, dy);
  }")

  # Calculate body-frame offsets (in meters) relative to the leader based on V-formation
  # Wide lateral spread + shallow depth = clearly visible V-shape
  case $i in
    0) DX_BODY=0; DY_BODY=0 ;;      # Drone 1 (Row 1, Center — LEADER)
    1) DX_BODY=-25; DY_BODY=-10 ;;  # Drone 2 (Row 2, Left Wing)
    2) DX_BODY=25; DY_BODY=-10 ;;   # Drone 3 (Row 2, Right Wing)
    3) DX_BODY=-50; DY_BODY=-20 ;;  # Drone 4 (Row 3, Left Wing)
    4) DX_BODY=50; DY_BODY=-20 ;;   # Drone 5 (Row 3, Right Wing)
    5) DX_BODY=0; DY_BODY=-20 ;;    # Drone 6 (Row 3, Center)
    6) DX_BODY=-75; DY_BODY=-30 ;;  # Drone 7 (Row 4, Outer Left)
    7) DX_BODY=75; DY_BODY=-30 ;;   # Drone 8 (Row 4, Outer Right)
    8) DX_BODY=-25; DY_BODY=-30 ;;  # Drone 9 (Row 4, Inner Left)
    9) DX_BODY=25; DY_BODY=-30 ;;   # Drone 10 (Row 4, Inner Right)
    *) DX_BODY=0; DY_BODY=0 ;;
  esac

  # Rotate body-frame offsets by the bearing angle
  ROTATED=$(awk "BEGIN {
    dx_body = $DX_BODY;
    dy_body = $DY_BODY;
    bearing = $BEARING;
    dx = dx_body * cos(bearing) + dy_body * sin(bearing);
    dy = -dx_body * sin(bearing) + dy_body * cos(bearing);
    print dx \" \" dy;
  }")
  DX=$(echo $ROTATED | awk '{print $1}')
  DY=$(echo $ROTATED | awk '{print $2}')

  # Offset starting coordinate based on rotated formation
  LAT_OFFSET=$(awk "BEGIN {print 33.6844 + ($DY * 0.00000898)}")
  LON_OFFSET=$(awk "BEGIN {print 73.0479 + ($DX * 0.0000108)}")

  echo "🚀 Launching Drone $SYSID (Instance $INSTANCE) at Lat=$LAT_OFFSET, Lon=$LON_OFFSET → port $PORT"

  nohup python3 /home/immad_baber/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter \
    -I $INSTANCE \
    --custom-location=$LAT_OFFSET,$LON_OFFSET,540,0 \
    --sysid=$SYSID \
    --out=127.0.0.1:$PORT \
    --out=$HOST_IP:$PORT \
    --no-rebuild \
    --use-dir=instance_$INSTANCE \
    --mavproxy-args="--daemon" > "/tmp/sitl_instance_$INSTANCE.log" 2>&1 &

  # Stagger: wait 5 seconds between each launch so internal TCP ports don't race
  if [ $i -lt $((NUM_DRONES - 1)) ]; then
    echo "[INFO] Waiting 5s before launching next instance..."
    sleep 5
  fi
done

echo "[INFO] All $NUM_DRONES instances spawned. Drones need ~3-4 min to reach GPS lock."
echo "[INFO] Monitor with: tail -f /tmp/sitl_instance_0.log"