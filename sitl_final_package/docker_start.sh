#!/bin/bash
# docker_start.sh - Entrypoint for the GCS-Swarm Docker container

ARDUPILOT_DIR="${ARDUPILOT_HOME:-/ardupilot}"
NUM_DRONES=${NUM_DRONES:-3}

echo "[INFO] Starting $NUM_DRONES ArduCopter SITL instances via sim_vehicle.py..."

for ((i=0; i<NUM_DRONES; i++)); do
  INSTANCE=$i
  SYSID=$((i + 1))
  PORT=$((14551 + i))

  if [ -d "/app/instance_$INSTANCE" ]; then
    rm -rf "/app/instance_$INSTANCE"
  fi
  mkdir -p "/app/instance_$INSTANCE"

  BEARING=$(awk "BEGIN {
    lat1 = 33.6844; lon1 = 73.0479;
    lat2 = 33.665137; lon2 = 73.027023;
    pi = 3.14159265;
    dy = lat2 - lat1;
    dx = (lon2 - lon1) * cos(lat1 * pi / 180.0);
    print atan2(dx, dy);
  }")

  case $i in
    0) DX_BODY=0;   DY_BODY=0   ;;
    1) DX_BODY=-25; DY_BODY=-10 ;;
    2) DX_BODY=25;  DY_BODY=-10 ;;
    3) DX_BODY=-50; DY_BODY=-20 ;;
    4) DX_BODY=50;  DY_BODY=-20 ;;
    5) DX_BODY=0;   DY_BODY=-20 ;;
    *) DX_BODY=0;   DY_BODY=0   ;;
  esac

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

  LAT_OFFSET=$(awk "BEGIN {print 33.6844 + ($DY * 0.00000898)}")
  LON_OFFSET=$(awk "BEGIN {print 73.0479 + ($DX * 0.0000108)}")

  echo "[INFO] Launching Drone $SYSID at Lat=$LAT_OFFSET, Lon=$LON_OFFSET → UDP port $PORT"

  # Launch sim_vehicle.py passing sitl_params.parm to ensure correct frame & parameters
  nohup python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I $INSTANCE \
    --custom-location=$LAT_OFFSET,$LON_OFFSET,540,0 \
    --sysid=$SYSID \
    --out=127.0.0.1:$PORT \
    --no-rebuild \
    --add-param-file=/app/sitl_params.parm \
    --use-dir=/app/instance_$INSTANCE \
    --mavproxy-args="--daemon" \
    > "/tmp/sitl_instance_$INSTANCE.log" 2>&1 &

  if [ $i -lt $((NUM_DRONES - 1)) ]; then
    echo "[INFO] Waiting 5s before next instance..."
    sleep 5
  fi
done

echo "[INFO] All $NUM_DRONES instances spawned."
echo "[INFO] Waiting 15s for SITL instances to initialize..."
sleep 15

echo "[INFO] Starting Telemetry Server on 0.0.0.0:5000..."
cd /app/mavlink_integration
exec python3 telemetry_server.py
