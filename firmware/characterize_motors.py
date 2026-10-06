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

def enc_samples(lines):
    out = []
    for t, l in lines:
        p = l.split(',')
        if l.startswith('E,') and len(p) == 4:
            out.append((t, int(p[1]), int(p[2])))
    return out

def run(pwm_l, pwm_r, duration=2.0, settle=0.7):
    s.write(b'Z\n')
    read_lines(0.4)
    t0 = time.time()
    lines = []
    cmd = f'M{pwm_l},{pwm_r}\n'.encode()
    while time.time() - t0 < duration:
        s.write(cmd)
        lines += read_lines(0.1)
    s.write(b'S\n')
    lines += read_lines(0.8)
    samples = [x for x in enc_samples(lines) if t0 + settle <= x[0] <= t0 + duration]
    if len(samples) < 5:
        return None
    (ta, a1, a2), (tb, b1, b2) = samples[0], samples[-1]
    dt = tb - ta
    return (b1 - a1) / dt, (b2 - a2) / dt

def countdown():
    for i in (3, 2, 1):
        print(f'  ...{i}', flush=True)
        read_lines(1.0)
    print('  GIRANDO', flush=True)

def visual_test(titulo, pwm_l, pwm_r, pergunta, respostas_validas):
    while True:
        print(f'\n=== {titulo} ===')
        input('Posicione-se pra ver as rodas e aperte Enter...')
        countdown()
        r = run(pwm_l, pwm_r, duration=3.0)
        print('  Parado. Velocidades medidas (enc1, enc2):',
              None if r is None else (round(r[0]), round(r[1])))
        resp = input(f'{pergunta} ({"/".join(respostas_validas)}, ou "r" pra repetir): ').strip().lower()
        if resp == 'r':
            continue
        if resp in respostas_validas:
            return resp
        print('  Resposta nao reconhecida, repetindo o teste.')

print('Abrindo porta (o ESP32 reinicia, aguardando boot)...')
read_lines(2.0)
s.write(b'S\n')

resp = input('Robo SUSPENSO e seguro? Digite "sim" para comecar: ')
if resp.strip().lower() != 'sim':
    s.write(b'S\n'); s.close(); sys.exit(0)

if input('\nRodar a tabela de velocidades? (s/n): ').strip().lower() == 's':
    print('\n=== Velocidade em regime (ticks/s) por PWM ===')
    print(f'{"PWM":>5} | {"enc1":>8} | {"enc2":>8} | {"dif %":>6}')
    for pwm in [15, 30, 50, 70, -30]:
        r = run(pwm, pwm)
        if r is None:
            print(f'{pwm:>5} | sem dados suficientes')
            continue
        v1, v2 = r
        dif = 100 * (v1 - v2) / ((v1 + v2) / 2) if (v1 + v2) != 0 else 0
        print(f'{pwm:>5} | {v1:>8.0f} | {v2:>8.0f} | {dif:>5.1f}%')
        time.sleep(0.5)

r1 = visual_test(
    'Teste de direcao: as duas rodas pra FRENTE (PWM 20, 3s)',
    20, 20,
    'As duas rodas giraram no sentido de FRENTE do robo?',
    ['s', 'n'])

r2 = visual_test(
    'Teste de lado: motor1 pra TRAS, motor2 pra FRENTE (PWM 20, 3s)',
    -20, 20,
    'Olhando o robo POR TRAS, qual roda girou PRA TRAS?',
    ['esquerda', 'direita'])

s.write(b'S\n')
s.close()
print('\n=== RESUMO ===')
print('M+ = frente do robo?      ', r1)
print('Motor1 (enc1) e a roda:   ', r2)
print('Motores parados.')
