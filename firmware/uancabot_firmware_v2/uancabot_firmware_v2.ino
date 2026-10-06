/*
 * uancabot_firmware_v2.ino
 * ─────────────────────────────────────────────────────────────────
 * Firmware novo com controle CONTINUO de velocidade (PWM proporcional)
 * e telemetria rapida. Substitui o firmware anterior, que so aceitava
 * comandos discretos (liga/desliga) e mandava telemetria devagar
 * (~1.4s por leitura) -- essas duas limitacoes tornavam impossivel ter
 * precisao real no controle de trajetoria (documentado em detalhe no
 * path_executor_node.py do lado ROS2, historico de bugs 2026-09).
 *
 * Protocolo serial (USB, 115200 baud), 1 linha = 1 mensagem, \n no fim:
 *
 *   RX (Radxa -> ESP32):
 *     M<pwm_l>,<pwm_r>\n   velocidade continua, cada valor -127..127
 *                          (0=parado, positivo=frente, negativo=re,
 *                          magnitude = intensidade do PWM)
 *     S\n                  para (equivale a M0,0)
 *     Z\n                  zera os encoders
 *
 *   TX (ESP32 -> Radxa), a cada TELEMETRY_INTERVAL_MS (default 50ms = 20Hz):
 *     E,<enc1>,<enc2>,<millis>\n
 *
 * Watchdog de seguranca: se o PWM atual for diferente de zero e nenhum
 * comando valido chegar em WATCHDOG_TIMEOUT_MS, para os motores sozinho
 * e avisa via "WATCHDOG_STOP" na serial.
 *
 * Hardware (igual as versoes anteriores):
 *   ESP32 GPIO17 (TX2) -> MD49 RX
 *   ESP32 GPIO16 (RX2) -> MD49 TX
 *   GND comum ESP32 <-> MD49
 *   MD49 com fonte 24V propria, jumper de baudrate ABERTO (9600 baud)
 *
 * Protocolo MD49 usado (datasheet oficial Robot Electronics):
 *   Sync byte 0x00 + registrador [+ dado]
 *   0x25 = GET_ENCODERS   (8 bytes: enc1 big-endian, enc2 big-endian)
 *   0x31 = SET_SPEED1     (1 byte, 128 = parado, 0-255)
 *   0x32 = SET_SPEED2     (1 byte, 128 = parado, 0-255)
 *   0x34 = SET_MODE       (Modo 0: velocidade independente 0-255)
 *   0x35 = RESET_ENCODERS
 *   0x38 = DISABLE_TIMEOUT (desliga o watchdog interno do MD49 -- o
 *          nosso proprio watchdog, feito no ESP32, cobre essa funcao)
 * ─────────────────────────────────────────────────────────────────
 */

#define MD49_SERIAL Serial2
#define RX2_PIN 16
#define TX2_PIN 17
#define MD49_BAUD 9600

#define MD49_SYNC            0x00
#define MD49_GET_ENCODERS    0x25
#define MD49_SET_SPEED1      0x31
#define MD49_SET_SPEED2      0x32
#define MD49_SET_MODE        0x34
#define MD49_RESET_ENCODERS  0x35
#define MD49_DISABLE_TIMEOUT 0x38

#define PWM_STOP        128
#define PWM_MAX_OFFSET  100    // limite de seguranca: PWM fica entre 28 e 228

#define TELEMETRY_INTERVAL_MS 50     // 20 Hz -- bem mais rapido que a v1 (~1.4s)
#define WATCHDOG_TIMEOUT_MS   500    // para sozinho se ficar mudo por 0.5s

int current_pwm_l_offset = 0;   // -127..127, relativo a PWM_STOP (128)
int current_pwm_r_offset = 0;
unsigned long last_cmd_ms = 0;
unsigned long last_telemetry_ms = 0;

String rx_buffer = "";

// ───── Helpers MD49 ─────

void md49SetSpeed(uint8_t reg, uint8_t speed) {
  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(MD49_SYNC);
  MD49_SERIAL.write(reg);
  MD49_SERIAL.write(speed);
}

void applyMotorSpeeds() {
  int l = constrain(current_pwm_l_offset, -PWM_MAX_OFFSET, PWM_MAX_OFFSET);
  int r = constrain(current_pwm_r_offset, -PWM_MAX_OFFSET, PWM_MAX_OFFSET);
  md49SetSpeed(MD49_SET_SPEED1, (uint8_t)(PWM_STOP + l));
  md49SetSpeed(MD49_SET_SPEED2, (uint8_t)(PWM_STOP + r));
}

void stopMotors() {
  current_pwm_l_offset = 0;
  current_pwm_r_offset = 0;
  applyMotorSpeeds();
}

void resetEncoders() {
  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(MD49_SYNC);
  MD49_SERIAL.write(MD49_RESET_ENCODERS);
}

bool readEncoders(long &enc1, long &enc2) {
  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(MD49_SYNC);
  MD49_SERIAL.write(MD49_GET_ENCODERS);

  unsigned long t0 = millis();
  while (MD49_SERIAL.available() < 8 && millis() - t0 < 30) {
    delay(1);
  }
  if (MD49_SERIAL.available() < 8) return false;

  uint8_t buf[8];
  for (int i = 0; i < 8; i++) buf[i] = MD49_SERIAL.read();

  enc1 = ((long)buf[0] << 24) | ((long)buf[1] << 16) | ((long)buf[2] << 8) | buf[3];
  enc2 = ((long)buf[4] << 24) | ((long)buf[5] << 16) | ((long)buf[6] << 8) | buf[7];
  return true;
}

// ───── Comandos pela serial USB ─────

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  last_cmd_ms = millis();  // qualquer comando valido reseta o watchdog

  if (cmd == 'M') {
    int comma = line.indexOf(',');
    if (comma > 1) {
      int l = line.substring(1, comma).toInt();
      int r = line.substring(comma + 1).toInt();
      current_pwm_l_offset = constrain(l, -PWM_MAX_OFFSET, PWM_MAX_OFFSET);
      current_pwm_r_offset = constrain(r, -PWM_MAX_OFFSET, PWM_MAX_OFFSET);
      applyMotorSpeeds();
    }
  } else if (cmd == 'S') {
    stopMotors();
  } else if (cmd == 'Z') {
    resetEncoders();
  }
}

// ───── Arduino ─────

void setup() {
  Serial.begin(115200);
  delay(200);

  MD49_SERIAL.begin(MD49_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(500);

  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(MD49_SYNC);
  MD49_SERIAL.write(MD49_SET_MODE);
  MD49_SERIAL.write((uint8_t)0);       // Modo 0: velocidade independente 0-255
  delay(20);
  MD49_SERIAL.write(MD49_SYNC);
  MD49_SERIAL.write(MD49_DISABLE_TIMEOUT);
  delay(20);

  stopMotors();
  resetEncoders();

  last_cmd_ms = millis();
  last_telemetry_ms = millis();

  Serial.println("READY uancabot_firmware_v2");
}

void loop() {
  // ─── Le comandos da Radxa ───
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleLine(rx_buffer);
      rx_buffer = "";
    } else if (c != '\r') {
      rx_buffer += c;
    }
  }

  // ─── Watchdog: para sozinho se ficar sem comando novo ───
  bool motor_running = (current_pwm_l_offset != 0) || (current_pwm_r_offset != 0);
  if (motor_running && (millis() - last_cmd_ms > WATCHDOG_TIMEOUT_MS)) {
    stopMotors();
    Serial.println("WATCHDOG_STOP");
  }

  // ─── Telemetria periodica (20Hz -- 28x mais rapido que a v1) ───
  if (millis() - last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_ms = millis();
    long enc1, enc2;
    if (readEncoders(enc1, enc2)) {
      Serial.print("E,");
      Serial.print(enc1);
      Serial.print(",");
      Serial.print(enc2);
      Serial.print(",");
      Serial.println(millis());
    }
  }
}
