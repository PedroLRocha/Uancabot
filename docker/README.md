# Docker — UancaBot ROS2

## Por quê Docker?

A Radxa Cubie A7Z roda Debian 11 nativamente, mas o projeto usa ROS2 Jazzy
(que requer Ubuntu 24.04). O container resolve isso sem precisar trocar o
SO da placa.

## Uso básico

```bash
cd docker
./run.sh
```

Isso abre um shell **dentro** do container, já com:
- Ubuntu 24.04 + ROS2 Jazzy instalado
- `/dev/ttyACM0` (ESP32) acessível
- `../ros2_ws` montado em `/workspace/ros2_ws` (mudanças no host aparecem
  instantaneamente dentro do container, e vice-versa)

Dentro do container:

```bash
# instala pyserial (não vem por padrão na imagem ros-base)
pip3 install --break-system-packages pyserial

# compila o pacote
cd /workspace/ros2_ws
colcon build --packages-select uancabot_odometry
source install/setup.bash

# roda o node
ros2 run uancabot_odometry uancabot_node
```

## Permissões da porta serial

Se aparecer `Permission denied` ao abrir `/dev/ttyACM0` dentro do container,
rode o `docker run` uma vez com `--privileged` no lugar de `--device`, ou
adicione `--group-add dialout` ao comando. O script `run.sh` já usa
`--device`, que costuma bastar.

## Persistência

- **Código-fonte**: fica em `ros2_ws/` no host (fora do container) — nunca
  se perde, mesmo se o container for destruído.
- **build/install/log**: são gerados dentro do container mas ficam no
  volume montado, então persistem entre execuções do `run.sh`. Se quiser
  forçar rebuild limpo: `rm -rf ros2_ws/build ros2_ws/install ros2_ws/log`.

## Rodando em background (uso normal, fora de debug)

```bash
docker run -d \
  --name uancabot_ros2 \
  --restart unless-stopped \
  --device=/dev/ttyACM0 \
  -v "$(pwd)/../ros2_ws:/workspace/ros2_ws" \
  -w /workspace/ros2_ws \
  ros:jazzy-ros-base \
  bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run uancabot_odometry uancabot_node"
```

Isso ainda vai exigir que o pacote já tenha sido compilado antes (via
`./run.sh` + `colcon build` interativo) pelo menos uma vez.
