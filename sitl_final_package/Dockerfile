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
EXPOSE 5000
EXPOSE 14551/udp
EXPOSE 14552/udp
EXPOSE 14553/udp

CMD ["./docker_start.sh"]
