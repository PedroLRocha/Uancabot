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
                out.append(line.decode('utf-8', 'ignore').strip())
    return out

def run(pwm1, pwm2, duration=3.0):
    t0 = time.time()
    cmd = f'M{pwm1},{pwm2}\n'.encode()
    while time.time() - t0 < duration:
        s.write(cmd)
        read_lines(0.1)
    s.write(b'S\n')
    read_lines(0.8)

REFERENCIA = 'ATRAS do robo, olhando na mesma direcao que a frente dele'

def visual_test(titulo, pwm1, pwm2, pergunta, validas):
    while True:
        print(f'\n=== {titulo} ===')
        print(f'  Posicao: {REFERENCIA}')
        input('  Posicione-se e aperte Enter...')
        for i in (3, 2, 1):
            print(f'  ...{i}', flush=True)
            read_lines(1.0)
        print('  GIRANDO', flush=True)
        run(pwm1, pwm2)
        print('  Parado.')
        resp = input(f'{pergunta} ({"/".join(validas)}, ou "r" pra repetir): ').strip().lower()
        if resp == 'r':
            continue
        if resp in validas:
            return resp
        print('  Resposta nao reconhecida, repetindo.')

print('==========================================================')
print(' REFERENCIA PARA TODOS OS TESTES:')
print(f' Fique {REFERENCIA}.')
print(' "Esquerda" e "direita" = a SUA esquerda e direita nessa posicao.')
print(' "Frente" = a direcao para onde voce esta olhando.')
print('==========================================================')

print('\nAbrindo porta (o ESP32 reinicia, aguardando boot)...')
read_lines(2.0)
s.write(b'S\n')

if input('Robo SUSPENSO e seguro? Digite "sim" para comecar: ').strip().lower() != 'sim':
    s.write(b'S\n'); s.close(); sys.exit(0)

a = visual_test('Teste A: SO o motor1 gira (motor2 parado)', -20, 0,
                'Qual roda girou, a da SUA esquerda ou a da SUA direita?',
                ['esquerda', 'direita'])
b = visual_test('Teste B: SO o motor2 gira (motor1 parado)', 0, -20,
                'Qual roda girou, a da SUA esquerda ou a da SUA direita?',
                ['esquerda', 'direita'])
c = visual_test('Teste C: as duas rodas com PWM NEGATIVO', -20, -20,
                'As duas rodas giraram pra FRENTE (pra onde voce esta olhando)?',
                ['s', 'n'])

s.write(b'S\n')
s.close()
print('\n=== RESUMO ===')
print('Motor1 e a roda:', a, '  (esperado: direita)')
print('Motor2 e a roda:', b, '  (esperado: esquerda)')
print('PWM negativo = frente?', c, '  (esperado: s)')
