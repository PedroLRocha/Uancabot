/*
 * drive_uanca.ino
 * ─────────────────────────────────────────────────────────────────
 * Firmware ESP32 + MD49 (Robot Electronics) — versão de produção,
 * validada e em uso.
 *
 * Hardware:
 *   ESP32 GPIO17 (TX2) -> MD49 RX
 *   ESP32 GPIO16 (RX2) -> MD49 TX
 *   GND comum ESP32 <-> MD49
 *   MD49 com fonte 24V propria
 *   Jumper de baudrate do MD49 ABERTO -> 9600 baud
 *
 * Protocolo MD49 (datasheet Robot Electronics):
 *   Sync byte 0x00 + registrador de comando [+ dado, se houver]
 *   0x20 = RESET_ENCODERS (mesmo valor que 0x35 em outras revisoes
 *          de firmware do MD49 -- confirme a versao do seu modulo)
 *   0x23 = GET_ENCODER1 (4 bytes, signed, big-endian)
 *   0x24 = GET_ENCODER2 (4 bytes, signed, big-endian)
 *   0x31 = SET_SPEED1   (1 byte, 128 = parado)
 *   0x32 = SET_SPEED2   (1 byte, 128 = parado)
 *
 * Comandos via Serial USB (115200, sem Enter):
 *   f = frente (ambas)       r = re (ambas)
 *   s = parar (ambas)        1 = so motor 1 frente
 *   2 = so motor 2 frente    q = parar motor 1
 *   w = parar motor 2
 *
 * Saida serial (a cada 500ms):
 *   -------------------
 *   Encoder 1: <valor>
 *   Encoder 2: <valor>
 *
 * Este e o formato que o node ROS2 uancabot_serial_node.py espera
 * (parser via regex "Encoder (\d+): (-?\d+)"). Nao alterar o
 * formato do print sem atualizar o node em conjunto.
 * ─────────────────────────────────────────────────────────────────
 */

#define MD49_SERIAL Serial2
#define RX2_PIN 16
#define TX2_PIN 17

 void setSpeed(byte motor, byte speed) {
  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(0x00);
  MD49_SERIAL.write(motor);
  MD49_SERIAL.write(speed);
}

long readEncoder(uint8_t reg) {
  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(0x00);
  MD49_SERIAL.write(reg);
  delay(100);
  if (MD49_SERIAL.available() >= 4) {
    long val = 0;
    for (int i = 0; i < 4; i++)
      val = (val << 8) | MD49_SERIAL.read();
    return val;
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  MD49_SERIAL.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(2000);

  while (MD49_SERIAL.available()) MD49_SERIAL.read();
  MD49_SERIAL.write(0x00);
  MD49_SERIAL.write(0x20);
  delay(200);

  Serial.println("Pronto! Comandos:");
  Serial.println("  f = frente (ambas)");
  Serial.println("  r = re (ambas)");
  Serial.println("  s = parar (ambas)");
  Serial.println("  1 = so motor 1 frente");
  Serial.println("  2 = so motor 2 frente");
  Serial.println("  q = parar motor 1");
  Serial.println("  w = parar motor 2");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'f':
        Serial.println(">> Frente!");
        setSpeed(0x31, 180);
        setSpeed(0x32, 180);
        break;
      case 'r':
        Serial.println(">> Re!");
        setSpeed(0x31, 80);
        setSpeed(0x32, 80);
        break;
      case 's':
        Serial.println(">> Parado!");
        setSpeed(0x31, 128);
        setSpeed(0x32, 128);
        break;
      case '1':
        Serial.println(">> So motor 1 frente!");
        setSpeed(0x31, 180);
        setSpeed(0x32, 128); // motor 2 parado
        break;
      case '2':
        Serial.println(">> So motor 2 frente!");
        setSpeed(0x31, 128); // motor 1 parado
        setSpeed(0x32, 180);
        break;
      case 'q':
        Serial.println(">> Parar motor 1!");
        setSpeed(0x31, 128);
        break;
      case 'w':
        Serial.println(">> Parar motor 2!");
        setSpeed(0x32, 128);
        break;
    }
  }

  long enc1 = readEncoder(0x23);
  long enc2 = readEncoder(0x24);

  Serial.println("-------------------");
  Serial.print("Encoder 1: ");
  Serial.println(enc1);
  Serial.print("Encoder 2: ");
  Serial.println(enc2);

  delay(500);
}
