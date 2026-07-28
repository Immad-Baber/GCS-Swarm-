# Use an official lightweight Python image
FROM python:3.10-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    procps \
    gawk \
    && rm -rf /var/lib/apt/lists/*

# Download pre-compiled ArduCopter Linux binary (SITL)
RUN wget -qO /usr/local/bin/arducopter https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter && \
    chmod +x /usr/local/bin/arducopter

# Set the working directory
WORKDIR /app

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY . .

# Ensure the startup script is executable
RUN chmod +x docker_start.sh

# Expose the telemetry server web port
EXPOSE 5000
# Expose MAVLink UDP ports if external connection is needed (optional, as they communicate internally)
EXPOSE 14551-14560/udp

# Set the entrypoint to the unified start script
CMD ["./docker_start.sh"]
