FROM ros:jazzy-ros-base

# Dependencias que antes precisavamos reinstalar toda sessao do container
RUN apt-get update && apt-get install -y \
    python3-serial \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
