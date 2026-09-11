#!/usr/bin/env python3
"""
UancaBot Serial Odometry Node (ROS2 Jazzy)
Lê encoders da serial do ESP32 e publica /odom + /tf.

ATENCAO: o firmware atualmente gravado no ESP32 NAO e o
firmware/drive_uanca/drive_uanca.ino deste repositorio -- e uma versao
mais avancada (com calibracao de rota quadrada, giro de 90, etc) cujo
codigo-fonte .ino foi perdido (so existe compilado na flash). O parser
abaixo foi feito por engenharia reversa do que a serial realmente envia.
Ver ENC,<enc1>,<enc2>,<timestamp_ms> em parse_enc_line().
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
import math
import re

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class UancabotSerialNode(Node):
    def __init__(self):
        super().__init__('uancabot_odometry_node')

        # ───── Parâmetros da plataforma (declarados como ROS params) ─────
        self.declare_parameter('wheel_diameter', 0.125)   # 12.5 cm
        self.declare_parameter('wheelbase', 0.33)          # 33 cm
        self.declare_parameter('ticks_per_rev', 980.0)     # datasheet EMG49
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)

        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value

        self.distance_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        self.get_logger().info("Parametros do robo:")
        self.get_logger().info(f"  Diametro roda: {self.wheel_diameter:.4f} m")
        self.get_logger().info(f"  Wheelbase:     {self.wheelbase:.4f} m")
        self.get_logger().info(f"  Ticks/rev:     {self.ticks_per_rev}")
        self.get_logger().info(f"  Dist/tick:     {self.distance_per_tick:.6f} m")

        # ───── Serial ─────
        self.serial = None
        self.last_enc1 = 0
        self.last_enc2 = 0
        self.first_reading = True  # evita salto grande na primeira leitura

        # ───── Odometria (estado acumulado) ─────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw em radianos

        # ───── Publishers ─────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ───── Subscriber pra mandar comandos pro ESP32 (mesma porta serial) ─────
        # Uso: ros2 topic pub --once /uancabot/raw_cmd std_msgs/String "data: 'f'"
        # Aceita qualquer comando de 1 char que o firmware entenda: f, r, s,
        # 1, 2, q, w, g, y, p, l (ver menu do firmware).
        self.cmd_sub = self.create_subscription(
            String, '/uancabot/raw_cmd', self.cmd_callback, 10
        )

        # ───── Timer pra ler serial ─────
        self.create_timer(0.05, self.serial_callback)  # 20 Hz

        self.connect_serial()

    def cmd_callback(self, msg):
        """Escreve o comando recebido direto na serial do ESP32."""
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn("[CMD] Serial nao conectada, comando descartado")
            return
        try:
            self.serial.write(msg.data.encode('utf-8'))
            self.get_logger().info(f"[CMD] Enviado ao ESP32: {msg.data!r}")
        except Exception as e:
            self.get_logger().warn(f"[CMD] Erro ao enviar: {e}")

    def connect_serial(self):
        try:
            self.serial = serial.Serial(self.serial_port, self.serial_baud, timeout=1.0)
            self.get_logger().info(f"[SERIAL] Conectado em {self.serial_port}")
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"[SERIAL] Falha ao conectar: {e}")
            self.serial = None

    def parse_enc_line(self, line):
        """
        Formato observado na serial (firmware atual, gravado por terceiros,
        fonte .ino nao disponivel -- reconstruido por engenharia reversa
        do output serial em 2026-09):

            ENC,<enc1>,<enc2>,<timestamp_ms>

        Essa linha vem ATOMICA (os dois encoders juntos, numa unica
        mensagem), diferente do formato antigo "Encoder 1: X" / "Encoder 2: Y"
        em duas linhas separadas -- que tinha risco de "pareamento errado"
        entre um enc1 de um ciclo e um enc2 de outro. Preferir sempre esta
        linha quando disponivel.
        """
        match = re.match(r'ENC,(-?\d+),(-?\d+),(-?\d+)', line)
        if match:
            return int(match.group(1)), int(match.group(2))
        return None

    def serial_callback(self):
        if not self.serial or not self.serial.is_open:
            self.connect_serial()
            return

        try:
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parsed = self.parse_enc_line(line)
                if parsed is None:
                    continue  # linha de menu/debug do firmware, ignora

                enc1, enc2 = parsed

                # O firmware retorna -1 quando o MD49 nao responde a tempo
                # (timeout de leitura). Nunca integrar essa leitura na pose --
                # um -1 isolado criaria um salto falso e permanente.
                if enc1 == -1 or enc2 == -1:
                    self.get_logger().warn(
                        "[SERIAL] Leitura invalida (timeout do MD49), descartada"
                    )
                    continue

                self.update_odometry(enc1, enc2)

        except Exception as e:
            self.get_logger().warn(f"[SERIAL] Erro na leitura: {e}")

    def update_odometry(self, enc1, enc2):
        # Primeira leitura só define baseline, não integra (evita salto)
        if self.first_reading:
            self.last_enc1 = enc1
            self.last_enc2 = enc2
            self.first_reading = False
            return

        delta_enc1 = enc1 - self.last_enc1
        delta_enc2 = enc2 - self.last_enc2
        self.last_enc1 = enc1
        self.last_enc2 = enc2

        dist_left = delta_enc1 * self.distance_per_tick
        dist_right = delta_enc2 * self.distance_per_tick

        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheelbase if self.wheelbase > 0 else 0.0

        self.theta += delta_theta
        self.x += dist_center * math.cos(self.theta)
        self.y += dist_center * math.sin(self.theta)

        self.publish_odom()

    def publish_odom(self):
        now = self.get_clock().now()

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = q_z
        odom_msg.pose.pose.orientation.w = q_w

        odom_msg.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]

        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = q_z
        tf_msg.transform.rotation.w = q_w

        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f"Pose: x={self.x:.3f}m  y={self.y:.3f}m  theta={math.degrees(self.theta):.1f}deg",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = UancabotSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()