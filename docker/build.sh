#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
docker build -t uancabot:jazzy .
echo "Imagem uancabot:jazzy construida com sucesso."
