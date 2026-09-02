# UancaBot

Robô diferencial com ESP32 + driver MD49 (Robot Electronics), controlado via ROS2.

## Arquitetura atual

```
Radxa Cubie A7Z (Debian 11, aarch64)
  └── Docker container (Ubuntu 24.04 + ROS2 Jazzy)
        └── uancabot_odometry (node Python)
              └── Serial USB (/dev/ttyACM0)
                    └── ESP32 (drive_uanca.ino)
                          └── UART 9600 → MD49 → Motores DC + Encoders
```

## Estrutura do repositório

```
uancabot/
├── firmware/
│   └── drive_uanca/
│       └── drive_uanca.ino      # firmware ESP32, grava via Arduino IDE/arduino-cli
├── ros2_ws/
│   └── src/
│       └── uancabot_odometry/   # pacote ROS2 (ament_python)
│           └── uancabot_odometry/
│               └── uancabot_serial_node.py   # lê serial, publica /odom + /tf
└── docker/
    └── run.sh                   # helper pra subir o container com acesso ao ESP32
```

## Hardware

- **SBC**: Radxa Cubie A7Z (Allwinner A733, aarch64, Debian 11 no host)
- **Microcontrolador**: ESP32 (UART2: GPIO16=RX, GPIO17=TX)
- **Driver de motor**: MD49 (Robot Electronics), UART 9600 baud, modo 0
- **Motores**: EMG49 com encoder (~980 ticks/revolução no eixo de saída)
- **Roda**: diâmetro 12.5 cm
- **Wheelbase** (distância entre rodas): 33 cm

## Protocolo MD49 (via ESP32)

Sync byte `0x00` + registrador de comando [+ dado]:

| Comando | Registrador | Descrição |
|---|---|---|
| GET_ENCODER1 | 0x23 | 4 bytes signed, big-endian |
| GET_ENCODER2 | 0x24 | 4 bytes signed, big-endian |
| SET_SPEED1   | 0x31 | 1 byte, 128 = parado |
| SET_SPEED2   | 0x32 | 1 byte, 128 = parado |
| RESET_ENC    | 0x20 | zera encoders |

## Formato serial ESP32 → ROS2

O firmware imprime a cada 500ms:
```
-------------------
Encoder 1: <valor>
Encoder 2: <valor>
```

O node `uancabot_serial_node.py` faz parse via regex `Encoder (\d+): (-?\d+)`.
**Não altere o formato do print sem atualizar o node em conjunto.**

## Quick start

Ver `docker/README.md` para instruções completas de build/run do container ROS2.

```bash
cd docker
./run.sh
# dentro do container:
cd /workspace/ros2_ws
colcon build --packages-select uancabot_odometry
source install/setup.bash
ros2 run uancabot_odometry uancabot_node
```

## Status do projeto

- [x] Firmware ESP32 validado (leitura de encoders + controle de motor)
- [x] Node ROS2 de odometria criado
- [ ] Calibração de TICKS_PER_REV confirmada empiricamente
- [ ] Teste de odometria em linha reta (comparar com trena)
- [ ] Integração com `/cmd_vel` (controle via teleop)
- [ ] Visualização em RViz
