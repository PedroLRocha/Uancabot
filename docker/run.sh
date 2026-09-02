#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# Sobe um container Ubuntu 24.04 + ROS2 Jazzy com:
#   - acesso ao ESP32 (/dev/ttyACM0)
#   - o workspace ROS2 montado como volume (edita fora, roda dentro)
#   - modo interativo com terminal
# ─────────────────────────────────────────────────────────────────
set -e

# Descobre o diretório do projeto (um nível acima de docker/)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Verifica se o ESP32 está conectado
if [ ! -e /dev/ttyACM0 ]; then
  echo "[AVISO] /dev/ttyACM0 não encontrado. O ESP32 está plugado?"
  echo "        Rodando mesmo assim (útil pra build sem hardware)."
fi

docker run -it --rm \
  --name uancabot_ros2 \
  --device=/dev/ttyACM0 \
  -v "${PROJECT_DIR}/ros2_ws:/workspace/ros2_ws" \
  -w /workspace/ros2_ws \
  ros:jazzy-ros-base \
  bash
