#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# Sobe um container Ubuntu 24.04 + ROS2 Jazzy com:
#   - acesso ao ESP32 via /dev/ttyESP32 (symlink udev fixo, ver
#     docker/README.md "Nome fixo da porta serial" -- nao usa
#     /dev/ttyACM0 direto pq essa placa re-enumera a porta as vezes
#     (ttyACM0 -> ttyACM1 -> ...) quando o ESP32 sofre reset via
#     hardware, o que quebraria o mapeamento fixo do --device)
#   - o workspace ROS2 montado como volume (edita fora, roda dentro)
#   - modo interativo com terminal
# ─────────────────────────────────────────────────────────────────
set -e

# Descobre o diretório do projeto (um nível acima de docker/)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ESP32_DEV="/dev/ttyESP32"

# Verifica se o ESP32 está conectado
if [ ! -e "$ESP32_DEV" ]; then
  echo "[AVISO] $ESP32_DEV não encontrado."
  echo "        Se o ESP32 estiver plugado, confira a regra udev:"
  echo "        cat /etc/udev/rules.d/99-esp32-uancabot.rules"
  echo "        Rodando mesmo assim (útil pra build sem hardware)."
fi

docker run -it --rm \
  --name uancabot_ros2 \
  --device="$ESP32_DEV" \
  -p 8080:8080 \
  -p 8765:8765 \
  -v "${PROJECT_DIR}/ros2_ws:/workspace/ros2_ws" \
  -w /workspace/ros2_ws \
  ros:jazzy-ros-base \
  bash
