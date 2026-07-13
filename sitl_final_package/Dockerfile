# Use an official lightweight Python image
FROM python:3.10-slim

# Install necessary system packages (git, networking tools)
RUN apt-get update && apt-get install -y \
    git \
    iproute2 \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory inside the container
WORKDIR /app

# Copy the requirements file and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY . .

# Ensure bash scripts are executable
RUN chmod +x *.sh

# Expose the telemetry server port (5000)
EXPOSE 5000

# By default, start a bash shell so you can run the scripts
CMD ["/bin/bash"]
