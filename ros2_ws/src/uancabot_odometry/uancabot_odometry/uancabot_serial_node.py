#!/usr/bin/env python3
"""
UancaBot Serial Odometry Node (ROS2 Jazzy)
Lê encoders da serial do ESP32 (drive_uanca.ino) e publica /odom + /tf
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
import math
import re

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

        # ───── Timer pra ler serial ─────
        self.create_timer(0.05, self.serial_callback)  # 20 Hz

        self.connect_serial()

    def connect_serial(self):
        try:
            self.serial = serial.Serial(self.serial_port, self.serial_baud, timeout=1.0)
            self.get_logger().info(f"[SERIAL] Conectado em {self.serial_port}")
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"[SERIAL] Falha ao conectar: {e}")
            self.serial = None

    def parse_encoders(self, line):
        # Espera linhas como "Encoder 1: 328" / "Encoder 2: 328"
        match = re.search(r'Encoder\s+(\d+):\s+(-?\d+)', line)
        if match:
            return int(match.group(1)), int(match.group(2))
        return None

    def serial_callback(self):
        if not self.serial or not self.serial.is_open:
            self.connect_serial()
            return

        try:
            enc1_data = None
            enc2_data = None

            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith('---'):
                    continue
                parsed = self.parse_encoders(line)
                if parsed:
                    enc_num, enc_val = parsed
                    if enc_num == 1:
                        enc1_data = enc_val
                    elif enc_num == 2:
                        enc2_data = enc_val

            if enc1_data is not None and enc2_data is not None:
                self.update_odometry(enc1_data, enc2_data)

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
