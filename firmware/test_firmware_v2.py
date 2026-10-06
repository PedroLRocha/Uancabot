import serial, time, sys

s = serial.Serial()
s.port = '/dev/ttyESP32'
s.baudrate = 115200
s.timeout = 0.05
s.dtr = False
s.rts = False
s.open()

buf = b''
def read_lines(duration):
    global buf
    out = []
    t0 = time.time()
    while time.time() - t0 < duration:
        data = s.read(256)
        if data:
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                out.append((time.time(), line.decode('utf-8', 'ignore').strip()))
    return out

def last_enc(lines):
    for _, l in reversed(lines):
        p = l.split(',')
        if l.startswith('E,') and len(p) == 4:
            return int(p[1]), int(p[2])
    return None

print('=== FASE 1: telemetria por 3s (motores parados) ===')
s.write(b'S\n')
time.sleep(0.2)
s.reset_input_buffer()
lines = read_lines(3.0)
enc = [l for _, l in lines if l.startswith('E,')]
outras = [l for _, l in lines if not l.startswith('E,')]
print(f'Linhas E: {len(enc)} em 3s  ->  ~{len(enc)/3:.1f} Hz (esperado ~20 Hz)')
print('Exemplos:', enc[:3])
if outras:
    print('Outras linhas:', outras[:5])
if len(enc) < 30:
    print('FALHA: telemetria ausente ou lenta. Nao vou mover os motores.')
    s.write(b'S\n'); s.close(); sys.exit(1)

resp = input('\nRobo SUSPENSO e seguro? Digite "sim" para testar os motores: ')
if resp.strip().lower() != 'sim':
    s.write(b'S\n'); s.close(); sys.exit(0)

print('\n=== FASE 2: PWM 30/30 por 2s (comando reenviado a cada 100ms) ===')
s.write(b'Z\n')
time.sleep(0.3)
inicio = last_enc(read_lines(0.3))
print('Encoders apos zerar:', inicio)
t0 = time.time()
lines = []
while time.time() - t0 < 2.0:
    s.write(b'M30,30\n')
    lines += read_lines(0.1)
s.write(b'S\n')
lines += read_lines(0.8)
fim = last_enc(lines)
print('Encoders apos 2s + parada:', fim)
if inicio and fim:
    d1, d2 = fim[0] - inicio[0], fim[1] - inicio[1]
    print(f'Delta enc1={d1}  enc2={d2}  (~{d1/2:.0f} e {d2/2:.0f} ticks/s)')

print('\n=== FASE 3: watchdog (manda M30,30 UMA vez e para de enviar) ===')
s.reset_input_buffer()
t_cmd = time.time()
s.write(b'M30,30\n')
lines = read_lines(1.5)
wd = [t for t, l in lines if l == 'WATCHDOG_STOP']
if wd:
    print(f'WATCHDOG_STOP recebido {wd[0]-t_cmd:.2f}s apos o ultimo comando (esperado ~0.5s)')
else:
    print('FALHA: watchdog NAO disparou')
s.write(b'S\n')
s.close()
print('\nTeste concluido. Motores parados.')
