# ─────────────────────────────────────────────────────────────────────────────
# GCS-Swarm SITL Docker Image
#
# Uses Ubuntu 22.04 + ArduPilot sim_vehicle.py (same path as manual launch).
# ─────────────────────────────────────────────────────────────────────────────
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ARDUPILOT_HOME=/ardupilot

# ── System dependencies ────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    sudo \
    python3 \
    python3-pip \
    python3-dev \
    gawk \
    procps \
    lsb-release \
    tzdata \
    screen \
    && rm -rf /var/lib/apt/lists/*

# ── Clone ArduPilot (shallow – we only need Tools/autotest/sim_vehicle.py) ─
RUN git clone --depth 1 --no-tags https://github.com/ArduPilot/ardupilot.git /ardupilot

# ── Install Python packages that sim_vehicle.py and MAVProxy need ──────────
RUN pip3 install --no-cache-dir \
    MAVProxy \
    pymavlink \
    future \
    pexpect \
    numpy \
    pyserial

# ── Create the expected binary directory, then download pre-built binary ───
RUN mkdir -p /ardupilot/build/sitl/bin && \
    wget -qO /ardupilot/build/sitl/bin/arducopter \
    https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter && \
    chmod +x /ardupilot/build/sitl/bin/arducopter

# ── App setup ──────────────────────────────────────────────────────────────
WORKDIR /app
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY . .
RUN chmod +x docker_start.sh

# ── Expose ports ───────────────────────────────────────────────────────────
# GCS web server
EXPOSE 5000
# SITL drone UDP ports — supports up to 10 drones (drone_1 … drone_10)
EXPOSE 14551/udp
EXPOSE 14552/udp
EXPOSE 14553/udp
EXPOSE 14554/udp
EXPOSE 14555/udp
EXPOSE 14556/udp
EXPOSE 14557/udp
EXPOSE 14558/udp
EXPOSE 14559/udp
EXPOSE 14560/udp

# ── Health check — queries the /api/health endpoint ───────────────────────
# Starts checking after 15s, polls every 10s, fails after 3 misses (30s).
HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD curl -f http://localhost:5000/api/health || exit 1

CMD ["./docker_start.sh"]
