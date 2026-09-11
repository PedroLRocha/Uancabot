# UancaBot

Robô diferencial com ESP32 + driver MD49 (Robot Electronics), controlado via ROS2.

## Arquitetura atual

```
Radxa Cubie A7Z (Debian 11, aarch64)
  └── Docker container (Ubuntu 24.04 + ROS2 Jazzy)
        └── uancabot_odometry (node Python)
              ├── LÊ: Serial USB (/dev/ttyACM0) → parse "ENC,e1,e2,t" → /odom + /tf
              └── ESCREVE: /uancabot/raw_cmd (std_msgs/String) → Serial USB → ESP32
                    └── ESP32 (firmware real, ver nota abaixo)
                          └── UART 9600 → MD49 → Motores DC + Encoders
```

## ⚠️ Nota importante sobre o firmware

O `firmware/drive_uanca/drive_uanca.ino` deste repositório é uma versão
**simplificada e antiga**. O firmware que está de fato gravado no ESP32 em uso
é mais avançado (tem calibração de rota quadrada, giro de 90°, etc) e seu
código-fonte `.ino` **foi perdido** — só existe compilado na flash do chip.

O protocolo serial real foi reconstruído por engenharia reversa (ver seção
"Formato serial" abaixo). **Não regrave o ESP32** com o `.ino` deste repo
sem ter certeza de que quer substituir o firmware atual — você perderia a
calibração embutida (`2495 ticks/100cm`, `540 ticks/giro de 90°`) e teria
que recalibrar do zero.

## Estrutura do repositório

```
uancabot/
├── firmware/
│   └── drive_uanca/
│       └── drive_uanca.ino      # versão simplificada/antiga -- NÃO é o firmware em uso, ver nota acima
├── ros2_ws/
│   └── src/
│       └── uancabot_odometry/   # pacote ROS2 (ament_python)
│           └── uancabot_odometry/
│               └── uancabot_serial_node.py   # lê serial (odom) + escreve serial (raw_cmd)
└── docker/
    ├── run.sh                   # helper pra subir o container com acesso ao ESP32
    └── README.md                 # instruções detalhadas de build/run
```

## Hardware

- **SBC**: Radxa Cubie A7Z (Allwinner A733, aarch64, Debian 11 no host)
- **Microcontrolador**: ESP32 (UART2: GPIO16=RX, GPIO17=TX)
- **Driver de motor**: MD49 (Robot Electronics), UART 9600 baud, modo 0
- **Motores**: EMG49 com encoder (~980 ticks/revolução no eixo de saída)
- **Roda**: diâmetro 12.5 cm
- **Wheelbase** (distância entre rodas): 33 cm

## Calibração (validada empiricamente em 2026-09)

| Valor | Teórico | Embutido no firmware real | Validado com trena |
|---|---|---|---|
| Ticks/revolução | 980 (datasheet) | — | — |
| Ticks/100cm | 2495 (calculado) | 2495 | ✅ 164cm medido vs 164.3cm odometria (erro 0.2%) |
| Ticks/giro 90° | ~695 (calc. c/ wheelbase=0.33m) | 540 | ainda não testado com trena/transferidor |

O giro (540 vs ~695 esperado) sugere que o **wheelbase efetivo** (considerando
derrapagem da roda ao girar) é diferente do wheelbase físico medido — comum em
robôs diferenciais. Testar e calibrar quando fizer o teste de giro.

## Formato serial ESP32 → ROS2

O firmware imprime a cada ~700ms um bloco assim:
```
-------------------
Encoder 1: <valor>
Encoder 2: <valor>
ENC,<encoder1>,<encoder2>,<timestamp_ms>
```

O node usa **apenas a linha `ENC,...`** (formato CSV atômico — os dois
encoders numa única mensagem, evita o risco de "pareamento errado" entre
um encoder de um ciclo e o outro de um ciclo diferente). As linhas
`Encoder 1:` / `Encoder 2:` separadas são ignoradas pelo parser.

Quando o MD49 não responde a tempo, o firmware imprime `-1` naquele
encoder — o node descarta essas leituras (não integra na pose).

## Comandos aceitos pelo firmware (via serial, 1 char, sem Enter)

| Tecla | Ação |
|---|---|
| `f` | frente (ambos motores) |
| `r` | ré |
| `s` | parar |
| `1` / `2` | só motor 1 / só motor 2 pra frente |
| `q` / `w` | parar motor 1 / motor 2 |
| `g` | executa rota quadrada 100x100cm (built-in) |
| `y` | zera os encoders |
| `p` | gira livremente (calibração manual de giro, pare com `s`) |
| `l` | executa giro de 90° (usa TICKS_TURN90=540 interno) |

## Enviando comandos pelo ROS2 (sem abrir a serial de novo)

O node já mantém a porta serial aberta pra leitura de odometria. Pra mandar
comandos de movimento, publique no tópico `/uancabot/raw_cmd` em vez de abrir
a porta serial direto (evita conflito de porta):

```bash
ros2 topic pub --once /uancabot/raw_cmd std_msgs/String "data: 'f'"
ros2 topic pub --once /uancabot/raw_cmd std_msgs/String "data: 's'"
```

## Quick start

Ver `docker/README.md` para instruções completas de build/run do container ROS2.

```bash
cd docker
./run.sh
# dentro do container:
apt update && apt install -y python3-serial   # necessário toda vez (container e-fêmero)
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
colcon build --packages-select uancabot_odometry
source install/setup.bash
ros2 run uancabot_odometry uancabot_node
```

## Problemas conhecidos / Troubleshooting

### ESP32 fica "mudo" depois de Ctrl+C no node

**Sintoma:** você mata o node com `Ctrl+C`, roda `ros2 run` de novo, o log
mostra `[SERIAL] Conectado em /dev/ttyACM0` mas nenhuma leitura chega depois
disso (silêncio total).

**Causa provável:** ao abrir a porta serial, o `pyserial` faz um toggle nas
linhas DTR/RTS (padrão do driver). Placas ESP32 costumam usar esse sinal pra
resetar o chip automaticamente — só que às vezes esse reset via software não
deixa o firmware num estado limpo (fica preso, sem printar nada).

**Fix confirmado:** aperta o botão físico **EN/RST** do ESP32 logo depois de
rodar o `ros2 run`. Em poucos segundos o firmware reinicia de verdade e volta
a mandar dados.

**Fix experimental a testar** (deve reduzir a necessidade do botão físico):
desabilitar o "hang up on close" da porta, que evita o toggle de DTR na hora
de FECHAR a conexão (uma das causas mais comuns desse tipo de estado preso):
```bash
sudo stty -F /dev/ttyACM0 -hupcl
```
Rodar esse comando uma vez (no host) antes de iniciar o node. Se resolver de
vez, automatizar via regra `udev` que roda isso toda vez que o ESP32 for
conectado.

### `python3-serial` não persiste entre execuções do `run.sh`

O container é criado com `--rm` (efêmero) — toda vez que você roda
`./run.sh`, é um container novo, sem os pacotes apt instalados antes.
Preciso reinstalar `python3-serial` toda sessão. Solução definitiva (ainda
não feita): criar um `Dockerfile` próprio com essa dependência já embutida
na imagem, em vez de instalar toda vez.

### Reboot da Radxa deixa o ESP32 num estado "mudo"

Se a Radxa reiniciar (queda de energia, brownout, etc) enquanto o ESP32
está conectado, o barramento USB reinicia junto e o ESP32 pode ficar preso
sem responder. O botão físico EN/RST resolve (mesmo procedimento do item
acima).

## Roadmap

- [x] Firmware ESP32 validado (leitura de encoders + controle de motor)
- [x] Node ROS2 de odometria criado
- [x] Calibração de ticks/distância confirmada empiricamente (erro 0.2% em teste de 164cm)
- [x] Teste de odometria em linha reta (comparado com trena)
- [x] Bridge de comandos via tópico ROS2 (`/uancabot/raw_cmd`)
- [ ] Calibração do giro de 90° confirmada com transferidor/trena
- [ ] Ferramenta de teste: desenhar trajetória (distância+ângulo) e o robô
      executar, usando feedback de `/odom` em malha fechada (antes de partir
      pra SLAM/navegação — objetivo é decodificar/testar, não produção)
- [ ] Integração com `/cmd_vel` padrão (Twist) pra usar com teleop/nav2
- [ ] `Dockerfile` próprio (evitar reinstalar deps toda sessão)
- [ ] Visualização em RViz
