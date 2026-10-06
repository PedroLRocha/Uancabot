# UancaBot

Robô diferencial com controle de trajetória em malha fechada, odometria por encoders e um planejador de rotas no navegador.

## Visão geral

```
Navegador (Mac/celular) ── http :8080 ──┐
Foxglove Studio ───────── ws   :8765 ──┤
                                         │  Radxa Cubie A7Z (Debian 11)
                                         │  └── Docker: Ubuntu 24.04 + ROS 2 Jazzy
                                         │       ├── route_ui        página de rotas (porta 8080)
                                         │       ├── path_generator  formas e desenho via tópicos/Foxglove
                                         │       ├── path_follower   Pure Pursuit, publica /cmd_vel
                                         │       ├── motor_bridge    /cmd_vel → PWM por roda, /odom a 20 Hz
                                         │       └── foxglove_bridge
                                         │
                                         └── USB /dev/ttyESP32 → ESP32 (firmware v2) → UART 9600 → MD49 → motores EMG49
```

## Hardware

| Item | Valor |
|---|---|
| Computador | Radxa Cubie A7Z (Allwinner A733, aarch64) |
| Microcontrolador | ESP32 (USB CH340/CH9102), firmware `uancabot_firmware_v2` |
| Driver de motor | MD49 (Robot Electronics), UART 9600, modo 0 |
| Motores | EMG49, ~980 ticks por volta no eixo de saída |
| Roda | diâmetro 12,5 cm |
| Distância entre rodas | 33 cm (nominal, ainda não calibrada no chão) |

Mapeamento validado em bancada: motor1/enc1 = roda **direita**, motor2/enc2 = roda **esquerda**; PWM negativo move a roda **para a frente** e o encoder diminui. O `motor_bridge` converte tudo para a convenção ROS (REP-103).

## Início rápido

No host da Radxa:

```bash
ls -la /dev/ttyESP32          # o ESP32 precisa aparecer
cd ~/uancabot/docker
./build.sh                    # só na primeira vez ou se o Dockerfile mudar
./run.sh
```

Dentro do container:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
colcon build --packages-select uancabot_odometry
source install/setup.bash
ros2 launch uancabot_odometry bringup.launch.py 2>&1 | tee /tmp/uancabot_log_$(date +%Y%m%d_%H%M%S).log
```

Depois abra `http://<ip-da-radxa>:8080` no navegador. Para o Foxglove, conecte em `ws://<ip-da-radxa>:8765`.

Se um node novo morrer na partida com `StopIteration` em `load_entry_point`, faça um build limpo do pacote:

```bash
rm -rf build/uancabot_odometry install/uancabot_odometry src/uancabot_odometry/*.egg-info
colcon build --packages-select uancabot_odometry
```

## Planejador de rotas (navegador)

A página mostra o robô ao vivo num mapa com grade e tem quatro modos:

- **Desenhar**: clique no mapa para marcar pontos (encaixe opcional na grade de 10 cm). Arraste para mover, toque num ponto para apagar. Opções de curvas suaves e de voltar ao ponto de partida.
- **Por distância**: lista de passos, como "andar 1 m", "virar à esquerda 90°" ou "curva à direita de 90° com raio 0,3 m".
- **Formas**: reta, arco, círculo, quadrado, retângulo, oito e zigue-zague, com medidas em metros.
- **Salvas**: rotas guardadas no navegador, com exportar e importar arquivo.

O robô só se move ao apertar **Executar rota**. O botão **PARAR** fica sempre visível (a barra de espaço também para) e trava o robô até ser liberado. Depois da execução, a rota planejada fica tracejada sobre o trajeto real, para comparar.

Nos cantos da rota (mudança de rumo acima de 35°), o seguidor para, gira no lugar e continua, mantendo a geometria exata. Curvas suaves são seguidas continuamente.

## Pacote ROS 2 `uancabot_odometry`

| Executável | Função |
|---|---|
| `motor_bridge` | Único dono da serial. Recebe `/cmd_vel`, controla a velocidade de cada roda (feedforward + PI com anti-windup) e publica `/odom`, `/tf`, `/actual_path` e `/uancabot/wheel_debug` |
| `path_follower` | Segue `/uancabot/follow_path` com Pure Pursuit e pivôs nos cantos |
| `route_ui` | Página web na porta 8080 (servidor HTTP da biblioteca padrão) |
| `path_generator` | Mesma geometria via tópicos (JSON em `/uancabot/generate_path`) e desenho por cliques no Foxglove |

A geometria e o controlador ficam em `path_tools.py`, sem dependência de ROS, para poderem ser testados fora do robô.

### Tópicos principais

| Tópico | Tipo | Uso |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | velocidade desejada |
| `/odom`, `/tf` | `nav_msgs/Odometry` | pose a 20 Hz |
| `/actual_path`, `/planned_path` | `nav_msgs/Path` | trajeto real e rota planejada |
| `/uancabot/follow_path` | `nav_msgs/Path` | rota para o seguidor |
| `/uancabot/follow_status` | `std_msgs/String` | `idle`, `running`, `done`, `aborted: motivo` |
| `/uancabot/estop` | `std_msgs/Bool` | `true` para tudo e trava; `false` libera |
| `/uancabot/abort_path` | `std_msgs/Empty` | cancela a rota |
| `/uancabot/reset_odom` | `std_msgs/Empty` | zera a pose e limpa o trajeto |

## Firmware v2 (ESP32)

Protocolo serial USB a 115200, uma mensagem por linha:

| Direção | Mensagem | Significado |
|---|---|---|
| Radxa → ESP32 | `M<motor1>,<motor2>` | PWM contínuo, −100 a 100 |
| Radxa → ESP32 | `S` | parar |
| Radxa → ESP32 | `Z` | zerar encoders |
| ESP32 → Radxa | `E,<enc1>,<enc2>,<millis>` | encoders a 20 Hz |
| ESP32 → Radxa | `READY uancabot_firmware_v2` | boot concluído |
| ESP32 → Radxa | `WATCHDOG_STOP` | parou sozinho após 0,5 s sem comando |

Gravação, a partir do host:

```bash
cd ~/uancabot/firmware/uancabot_firmware_v2
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyESP32 .
```

Os scripts em `firmware/` testam o firmware sem ROS: `test_firmware_v2.py` (telemetria, motores e watchdog), `characterize_motors.py` (velocidade por PWM) e `confirm_mapping.py` (qual motor é qual roda).

Caracterização em bancada: ~15,3 ticks/s por unidade de PWM, linear, com menos de 0,5% de diferença entre as rodas.

## Camadas de segurança

1. Botão PARAR na página e no Foxglove (`/uancabot/estop`), que trava até ser liberado.
2. `motor_bridge` zera a velocidade se o `/cmd_vel` parar por 0,5 s ou se a telemetria do ESP32 parar por 0,3 s.
3. `path_follower` cancela a rota se o `/odom` parar ou se não houver progresso por 8 s.
4. Watchdog do firmware para os motores após 0,5 s sem comando, mesmo que a Radxa trave.

## Estrutura

```
docker/                      Dockerfile, build.sh, run.sh
firmware/
  uancabot_firmware_v2/      firmware atual (PWM contínuo, telemetria 20 Hz, watchdog)
  drive_uanca/               firmware antigo, só histórico
  *.py                       scripts de teste de bancada
ros2_ws/src/uancabot_odometry/
  uancabot_odometry/         nodes e path_tools.py
  launch/bringup.launch.py   sobe tudo, incluindo o foxglove_bridge
  web/route_ui.html          página de rotas
```

## Limitações e próximos passos

- Só rotas para a frente (sem ré) e sem orientação final garantida.
- Calibrar no chão a distância efetiva entre rodas (afeta curvas e pivôs) e reconfirmar a escala linear com trena.
- O desvio lateral acumulado ainda depende só de odometria; não há correção por sensor externo.
