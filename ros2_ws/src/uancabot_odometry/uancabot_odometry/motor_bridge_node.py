#!/usr/bin/env python3
"""
UancaBot Motor Bridge (ROS2 Jazzy) -- para o firmware uancabot_firmware_v2

Unico processo que abre a porta serial do ESP32. O resto do sistema fala
com o robo so por topicos ROS:

  Entrada:
    /cmd_vel               geometry_msgs/Twist  (linear.x m/s, angular.z rad/s)
    /uancabot/estop        std_msgs/Bool        true = para tudo e ignora /cmd_vel
                                                ate receber false
    /uancabot/reset_odom   std_msgs/Empty       zera a pose e limpa /actual_path

  Saida:
    /odom                  nav_msgs/Odometry    (pose + velocidades)
    /tf                    odom -> base_link
    /actual_path           nav_msgs/Path        (trajetoria percorrida, decimada)
    /uancabot/wheel_debug  std_msgs/Float32MultiArray
                           [alvo_esq, alvo_dir, medido_esq, medido_dir,
                            pwm_esq, pwm_dir]  (ticks/s e PWM, convencao ROS)

Protocolo do firmware v2 (ver firmware/uancabot_firmware_v2):
  TX: M<motor1>,<motor2>  (-100..100)   S (para)   Z (zera encoders)
  RX: E,<enc1>,<enc2>,<millis> a 20 Hz   READY ...   WATCHDOG_STOP

MAPEAMENTO FISICO (validado com testes de bancada em 2026-09):
  motor1 / enc1 = roda DIREITA
  motor2 / enc2 = roda ESQUERDA
  PWM negativo  = roda gira pra FRENTE do robo, e o encoder DIMINUI
Este node converte tudo pra convencao ROS (REP-103): positivo = frente,
theta positivo = giro anti-horario (pra esquerda) visto de cima.

Controle de velocidade por roda (malha fechada, 20 Hz):
  pwm = alvo/ff_ticks_per_pwm + kp*erro + integral(ki*erro)
  ff_ticks_per_pwm = 15.3 medido em bancada (relacao linear, <0.5% de
  diferenca entre as rodas). O feedforward faz quase todo o trabalho, o PI
  so corrige carga/atrito (robo no chao vs suspenso).

Camadas de seguranca:
  - /cmd_vel parado por cmd_timeout_s (0.5s) -> velocidade alvo zero
  - telemetria parada por telemetry_timeout_s (0.3s) -> velocidade alvo zero
  - /uancabot/estop true -> manda S imediatamente e trava ate receber false
  - watchdog do proprio firmware -> para os motores se ficar 0.5s sem comando
"""

import math
import time

import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Bool, Empty, Float32MultiArray
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster


def wheels_from_encoders(enc1, enc2):
    """Converte encoders crus em (esquerda, direita) com positivo = frente."""
    return -enc2, -enc1


def motors_from_wheel_pwm(pwm_left, pwm_right):
    """Converte PWM por roda (positivo = frente) em (motor1, motor2) do firmware."""
    return -pwm_right, -pwm_left


class MotorBridge(Node):
    def __init__(self):
        super().__init__('uancabot_motor_bridge')

        defaults = {
            'serial_port': '/dev/ttyESP32',
            'serial_baud': 115200,
            'wheel_diameter': 0.125,
            'wheelbase': 0.33,
            'ticks_per_rev': 980.0,
            'ff_ticks_per_pwm': 15.3,
            'kp': 0.02,
            'ki': 0.05,
            'i_limit_pwm': 25.0,
            'max_pwm': 80,
            'max_wheel_speed': 0.40,
            'cmd_timeout_s': 0.5,
            'telemetry_timeout_s': 0.3,
            'speed_filter_alpha': 0.5,
            'path_decimation_m': 0.02,
            'path_max_poses': 5000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = lambda n: self.get_parameter(n).value

        self.port = g('serial_port')
        self.baud = g('serial_baud')
        self.wheelbase = float(g('wheelbase'))
        self.m_per_tick = math.pi * float(g('wheel_diameter')) / float(g('ticks_per_rev'))
        self.ff = float(g('ff_ticks_per_pwm'))
        self.kp = float(g('kp'))
        self.ki = float(g('ki'))
        self.i_limit = float(g('i_limit_pwm'))
        self.max_pwm = int(g('max_pwm'))
        self.max_wheel_speed = float(g('max_wheel_speed'))
        self.cmd_timeout = float(g('cmd_timeout_s'))
        self.tel_timeout = float(g('telemetry_timeout_s'))
        self.alpha = float(g('speed_filter_alpha'))
        self.path_decimation = float(g('path_decimation_m'))
        self.path_max = int(g('path_max_poses'))
        self.ctrl_dt = 0.05

        # Serial
        self.ser = None
        self.rx_buf = b''
        self.last_open_attempt = 0.0

        # Encoders / odometria
        self.first_reading = True
        self.prev_left = 0
        self.prev_right = 0
        self.last_ms = 0
        self.last_telemetry = 0.0
        self.meas_left = 0.0     # ticks/s filtrado, positivo = frente
        self.meas_right = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

        # Comando
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = 0.0
        self.estop = False
        self.int_left = 0.0
        self.int_right = 0.0

        # /actual_path
        self.path_poses = []
        self.last_path_xy = None
        self.path_capped_warned = False

        # ROS
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/uancabot/wheel_debug', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Bool, '/uancabot/estop', self.estop_cb, 10)
        self.create_subscription(Empty, '/uancabot/reset_odom', self.reset_odom_cb, 10)

        self.create_timer(0.01, self.read_serial)
        self.create_timer(self.ctrl_dt, self.control_loop)

        self.get_logger().info('Motor bridge (firmware v2) iniciado.')
        self.get_logger().info(
            f'  m/tick={self.m_per_tick:.6f}  wheelbase={self.wheelbase}  '
            f'ff={self.ff} ticks/s por PWM  kp={self.kp} ki={self.ki}  '
            f'max_wheel_speed={self.max_wheel_speed} m/s  max_pwm={self.max_pwm}'
        )
        self.open_serial()

    # ───────── Serial ─────────

    def open_serial(self):
        self.last_open_attempt = time.monotonic()
        try:
            s = serial.Serial()
            s.port = self.port
            s.baudrate = self.baud
            s.timeout = 0
            s.dtr = False
            s.rts = False
            s.open()
            self.ser = s
            self.rx_buf = b''
            self.first_reading = True
            self.get_logger().info(f'[SERIAL] Conectado em {self.port}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'[SERIAL] Falha ao abrir {self.port}: {e}',
                                    throttle_duration_sec=5.0)

    def close_serial(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def send(self, text):
        if self.ser is None:
            return
        try:
            self.ser.write((text + '\n').encode())
        except Exception as e:
            self.get_logger().warn(f'[SERIAL] Erro ao enviar, reconectando: {e}')
            self.close_serial()

    def read_serial(self):
        if self.ser is None:
            if time.monotonic() - self.last_open_attempt > 1.0:
                self.open_serial()
            return
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b''
        except Exception as e:
            self.get_logger().warn(f'[SERIAL] Erro de leitura, reconectando: {e}')
            self.close_serial()
            return
        if not data:
            return
        self.rx_buf += data
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            self.handle_line(raw.decode('utf-8', errors='ignore').strip())
        if len(self.rx_buf) > 4096:
            self.rx_buf = b''

    def handle_line(self, line):
        if line.startswith('E,'):
            parts = line.split(',')
            if len(parts) != 4:
                return
            try:
                enc1, enc2, ms = int(parts[1]), int(parts[2]), int(parts[3])
            except ValueError:
                return
            self.on_encoders(enc1, enc2, ms)
        elif line.startswith('READY'):
            self.get_logger().info(f'[ESP32] {line}')
            self.first_reading = True
        elif line == 'WATCHDOG_STOP':
            self.get_logger().warn('[ESP32] Watchdog do firmware parou os motores '
                                   '(ficou 0.5s sem receber comando)')

    # ───────── Odometria ─────────

    def on_encoders(self, enc1, enc2, ms):
        left, right = wheels_from_encoders(enc1, enc2)
        now = time.monotonic()

        # Primeira leitura, reboot do ESP32 (millis voltou) ou salto absurdo:
        # so redefine a referencia, sem integrar.
        if self.first_reading or ms <= self.last_ms:
            self._rebase(left, right, ms, now)
            return
        d_left = left - self.prev_left
        d_right = right - self.prev_right
        if abs(d_left) > 2000 or abs(d_right) > 2000:
            self.get_logger().warn(f'[ODOM] Salto de encoder descartado: {d_left}, {d_right}')
            self._rebase(left, right, ms, now)
            return

        dt = (ms - self.last_ms) / 1000.0
        self.prev_left, self.prev_right, self.last_ms = left, right, ms
        self.last_telemetry = now

        a = self.alpha
        self.meas_left = a * (d_left / dt) + (1 - a) * self.meas_left
        self.meas_right = a * (d_right / dt) + (1 - a) * self.meas_right

        dl = d_left * self.m_per_tick
        dr = d_right * self.m_per_tick
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / self.wheelbase
        mid = self.theta + dth / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))
        self.v = ds / dt
        self.w = dth / dt

        self.publish_odom()

    def _rebase(self, left, right, ms, now):
        self.prev_left, self.prev_right, self.last_ms = left, right, ms
        self.first_reading = False
        self.last_telemetry = now
        self.meas_left = self.meas_right = 0.0

    def publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        cov = [0.0] * 36
        for i in (0, 7, 35):
            cov[i] = 0.01
        for i in (14, 21, 28):
            cov[i] = 1e6
        odom.pose.covariance = cov
        odom.twist.covariance = cov
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        self.update_path(stamp, qz, qw)

        self.get_logger().info(
            f'Pose: x={self.x:.3f}m y={self.y:.3f}m theta={math.degrees(self.theta):.1f}deg  '
            f'v={self.v:.3f}m/s w={math.degrees(self.w):.1f}deg/s',
            throttle_duration_sec=1.0)

    def update_path(self, stamp, qz, qw):
        if self.last_path_xy is not None:
            if math.hypot(self.x - self.last_path_xy[0],
                          self.y - self.last_path_xy[1]) < self.path_decimation:
                return
        if len(self.path_poses) >= self.path_max:
            if not self.path_capped_warned:
                self.get_logger().warn('[PATH] /actual_path atingiu o limite de poses '
                                       '(use /uancabot/reset_odom pra limpar)')
                self.path_capped_warned = True
            return
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.path_poses.append(pose)
        self.last_path_xy = (self.x, self.y)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'odom'
        path.poses = self.path_poses
        self.path_pub.publish(path)

    # ───────── Comandos ─────────

    def cmd_vel_cb(self, msg):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()

    def estop_cb(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('[ESTOP] Parada de emergencia ATIVADA')
        elif not msg.data and self.estop:
            self.get_logger().info('[ESTOP] Parada de emergencia liberada')
        self.estop = bool(msg.data)
        self.cmd_v = self.cmd_w = 0.0
        self.int_left = self.int_right = 0.0
        if self.estop:
            self.send('S')

    def reset_odom_cb(self, msg):
        self.x = self.y = self.theta = 0.0
        self.path_poses = []
        self.last_path_xy = None
        self.path_capped_warned = False
        self.get_logger().info('[ODOM] Pose zerada e /actual_path limpo')
        self.publish_odom()

    # ───────── Controle de velocidade (20 Hz) ─────────

    def wheel_pwm(self, target, measured, integ):
        err = target - measured
        # Anti-windup: so integra perto do regime. Durante a aceleracao o
        # erro e grande (o MD49 tem rampa interna) e integrar ali acumulava
        # um excesso que causava ~10% de sobrevelocidade (teste 2026-10).
        if abs(err) < 0.3 * abs(target):
            integ += self.ki * err * self.ctrl_dt
            integ = max(-self.i_limit, min(self.i_limit, integ))
        pwm = target / self.ff + self.kp * err + integ
        pwm = max(-self.max_pwm, min(self.max_pwm, pwm))
        return pwm, integ

    def control_loop(self):
        if self.ser is None:
            return
        if self.estop:
            self.send('S')
            return

        now = time.monotonic()
        v, w = self.cmd_v, self.cmd_w
        if now - self.last_cmd_time > self.cmd_timeout:
            v = w = 0.0
        if (v != 0.0 or w != 0.0) and now - self.last_telemetry > self.tel_timeout:
            self.get_logger().warn('[CTRL] Sem telemetria do ESP32 -- mantendo motores parados',
                                   throttle_duration_sec=2.0)
            v = w = 0.0

        v_left = v - w * self.wheelbase / 2.0
        v_right = v + w * self.wheelbase / 2.0
        peak = max(abs(v_left), abs(v_right))
        if peak > self.max_wheel_speed:
            k = self.max_wheel_speed / peak   # preserva a curvatura pedida
            v_left *= k
            v_right *= k

        t_left = v_left / self.m_per_tick
        t_right = v_right / self.m_per_tick

        if t_left == 0.0 and t_right == 0.0:
            self.int_left = self.int_right = 0.0
            pwm_left = pwm_right = 0.0
        else:
            pwm_left, self.int_left = self.wheel_pwm(t_left, self.meas_left, self.int_left)
            pwm_right, self.int_right = self.wheel_pwm(t_right, self.meas_right, self.int_right)

        m1, m2 = motors_from_wheel_pwm(int(round(pwm_left)), int(round(pwm_right)))
        self.send(f'M{m1},{m2}')

        dbg = Float32MultiArray()
        dbg.data = [float(t_left), float(t_right), float(self.meas_left),
                    float(self.meas_right), float(pwm_left), float(pwm_right)]
        self.debug_pub.publish(dbg)

    def shutdown(self):
        self.send('S')
        self.close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
