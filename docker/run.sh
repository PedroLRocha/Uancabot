#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# Sobe o container UancaBot (imagem propria, com deps ja embutidas):
#   - --privileged + -v /dev:/dev: o container enxerga /dev do HOST
#     diretamente, sempre atualizado. Resolve o problema de o ESP32
#     reenumerar (ttyACM0 -> ttyACM1) no meio da sessao e o container
#     ficar "cego" pro dispositivo -- antes isso exigia recriar o
#     container inteiro.
#   - porta 8080 exposta (painel web antigo, se ainda usar)
#   - porta 8765 exposta (foxglove_bridge)
#   - workspace ROS2 montado como volume
# ─────────────────────────────────────────────────────────────────
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="uancabot:jazzy"

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "[ERRO] Imagem $IMAGE_NAME nao encontrada."
  echo "       Construa primeiro com: ~/uancabot/docker/build.sh"
  exit 1
fi

docker run -it --rm \
  --name uancabot_ros2 \
  --privileged \
  -v /dev:/dev \
  -p 8080:8080 \
  -p 8765:8765 \
  -v "${PROJECT_DIR}/ros2_ws:/workspace/ros2_ws" \
  -w /workspace/ros2_ws \
  "$IMAGE_NAME" \
  bash
