#!/bin/bash
# docker_start.sh - Entrypoint for the Docker container

echo "[INFO] Starting 3 ArduCopter instances..."

NUM_DRONES=3

for ((i=0; i<NUM_DRONES; i++)); do
  INSTANCE=$i
  SYSID=$((i + 1))
  PORT=$((14551 + i))
  TCP_PORT=$((5760 + i * 10))

  mkdir -p "instance_$INSTANCE"
  if [ -f "eeprom.bin" ]; then
    cp eeprom.bin "instance_$INSTANCE/eeprom.bin"
  fi

  # Calculate bearing to align formation
  BEARING=$(awk "BEGIN {
    lat1 = 33.6844; lon1 = 73.0479;
    lat2 = 33.665137; lon2 = 73.027023;
    pi = 3.14159265;
    dy = lat2 - lat1;
    dx = (lon2 - lon1) * cos(lat1 * pi / 180.0);
    print atan2(dx, dy);
  }")

  case $i in
    0) DX_BODY=0; DY_BODY=0 ;;
    1) DX_BODY=-25; DY_BODY=-10 ;;
    2) DX_BODY=25; DY_BODY=-10 ;;
    *) DX_BODY=0; DY_BODY=0 ;;
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

  echo "[INFO] Launching Drone $SYSID (Instance $INSTANCE) at Lat=$LAT_OFFSET, Lon=$LON_OFFSET → port $PORT"

  # Launch the precompiled ArduCopter binary
  cd instance_$INSTANCE
  /usr/local/bin/arducopter -S -I $INSTANCE \
    --sysid=$SYSID \
    --model + \
    --speedup 1 \
    --defaults eeprom.bin \
    --home $LAT_OFFSET,$LON_OFFSET,540,0 > /dev/null 2>&1 &
  cd ..

  # Launch MAVProxy to route the TCP port to the UDP port expected by the telemetry server
  mavproxy.py --master tcp:127.0.0.1:$TCP_PORT --out udp:127.0.0.1:$PORT --daemon > /dev/null 2>&1 &
done

echo "[INFO] Waiting 5 seconds for SITL instances to boot..."
sleep 5

echo "[INFO] Starting Telemetry Server..."
cd mavlink_integration
python3 telemetry_server.py
