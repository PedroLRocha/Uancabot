#!/usr/bin/env python3
"""
UancaBot Path Generator Node (ROS2 Jazzy)

Gera trajetorias geometricas prontas (reta, quadrado, circulo, figura 8,
slalom) e publica como nav_msgs/Path em /planned_path -- para comparar
visualmente com /actual_path no Foxglove e calibrar odometria/controle.

Uso (via topico):
  ros2 topic pub --once /uancabot/generate_path std_msgs/String \
    'data: "{\"shape\": \"circle\", \"radius_m\": 0.5}"'

Formas suportadas e parametros (tudo em metros, angulos internos):
  line:     {"shape": "line", "length_m": 1.0}
  square:   {"shape": "square", "side_m": 1.0}
  circle:   {"shape": "circle", "radius_m": 0.5}
  figure8:  {"shape": "figure8", "radius_m": 0.5}
  slalom:   {"shape": "slalom", "length_m": 2.0, "amplitude_m": 0.3, "wavelength_m": 0.5}

Parametro comum opcional: "spacing_m" (default 0.05 = 5cm) -- espacamento
entre pontos gerados ao longo do caminho.

Parametro opcional "execute": true -- alem de publicar /planned_path,
tambem envia os waypoints [x,y] pro path_executor existente via
/uancabot/execute_path, fazendo o robo seguir de verdade. Default false
(so gera e visualiza, NAO move o robo) -- opt-in explicito por seguranca.
"""

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def make_pose(x, y, heading_rad, frame_id="odom"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(heading_rad / 2.0)
    pose.pose.orientation.w = math.cos(heading_rad / 2.0)
    return pose


def gen_line(length_m, spacing_m):
    n = max(2, round(length_m / spacing_m)) + 1
    pts = []
    for i in range(n):
        x = length_m * i / (n - 1)
        pts.append((x, 0.0, 0.0))
    return pts


def gen_square(side_m, spacing_m):
    """Quadrado CCW comecando em (0,0), heading 0 (olhando +x)."""
    pts = []
    headings = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
    starts = [(0.0, 0.0), (side_m, 0.0), (side_m, side_m), (0.0, side_m)]
    n_side = max(2, round(side_m / spacing_m))
    for side_idx in range(4):
        sx, sy = starts[side_idx]
        heading = headings[side_idx]
        dx = math.cos(heading)
        dy = math.sin(heading)
        for i in range(n_side):
            t = side_m * i / n_side
            pts.append((sx + dx * t, sy + dy * t, heading))
    pts.append((0.0, 0.0, 0.0))
    return pts


def gen_circle(radius_m, spacing_m):
    circumference = 2 * math.pi * radius_m
    n = max(8, round(circumference / spacing_m))
    pts = []
    for i in range(n + 1):
        s = circumference * i / n
        angle = s / radius_m
        x = radius_m * math.sin(angle)
        y = radius_m * (1 - math.cos(angle))
        pts.append((x, y, angle))
    return pts


def gen_figure8(radius_m, spacing_m):
    circumference = 2 * math.pi * radius_m
    n = max(8, round(circumference / spacing_m))
    pts = []
    for i in range(n + 1):
        s = circumference * i / n
        angle = s / radius_m
        x = radius_m * math.sin(angle)
        y = radius_m * (1 - math.cos(angle))
        pts.append((x, y, angle))
    for i in range(1, n + 1):
        s = circumference * i / n
        angle = s / radius_m
        x = radius_m * math.sin(angle)
        y = -radius_m * (1 - math.cos(angle))
        pts.append((x, y, -angle))
    return pts


def gen_slalom(length_m, amplitude_m, wavelength_m, spacing_m):
    n = max(8, round(length_m / spacing_m)) + 1
    pts = []
    for i in range(n):
        x = length_m * i / (n - 1)
        y = amplitude_m * math.sin(2 * math.pi * x / wavelength_m)
        dydx = amplitude_m * (2 * math.pi / wavelength_m) * math.cos(2 * math.pi * x / wavelength_m)
        heading = math.atan2(dydx, 1.0)
        pts.append((x, y, heading))
    return pts


class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('uancabot_path_generator')

        self.declare_parameter('default_spacing_m', 0.05)
        self.default_spacing_m = self.get_parameter('default_spacing_m').value

        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.execute_pub = self.create_publisher(String, '/uancabot/execute_path', 10)

        self.gen_sub = self.create_subscription(
            String, '/uancabot/generate_path', self.generate_callback, 10
        )

        self.get_logger().info("Path generator pronto.")
        self.get_logger().info(
            "Formas: line, square, circle, figure8, slalom."
        )
        self.get_logger().info(
            'Acrescente "execute": true no JSON pra tambem mandar o robo seguir '
            "(default false -- so gera e visualiza, nao move o robo)."
        )

    def generate_callback(self, msg):
        try:
            spec = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[PATH_GEN] JSON invalido: {e}")
            return

        shape = spec.get('shape')
        spacing = spec.get('spacing_m', self.default_spacing_m)

        try:
            if shape == 'line':
                pts = gen_line(spec['length_m'], spacing)
            elif shape == 'square':
                pts = gen_square(spec['side_m'], spacing)
            elif shape == 'circle':
                pts = gen_circle(spec['radius_m'], spacing)
            elif shape == 'figure8':
                pts = gen_figure8(spec['radius_m'], spacing)
            elif shape == 'slalom':
                pts = gen_slalom(
                    spec['length_m'], spec['amplitude_m'], spec['wavelength_m'], spacing
                )
            else:
                self.get_logger().error(
                    f"[PATH_GEN] Forma desconhecida: {shape!r}. "
                    "Use: line, square, circle, figure8, slalom."
                )
                return
        except KeyError as e:
            self.get_logger().error(f"[PATH_GEN] Parametro obrigatorio faltando: {e}")
            return

        now = self.get_clock().now()
        path_msg = Path()
        path_msg.header.stamp = now.to_msg()
        path_msg.header.frame_id = "odom"
        path_msg.poses = [make_pose(x, y, h) for (x, y, h) in pts]
        for pose in path_msg.poses:
            pose.header.stamp = now.to_msg()

        self.planned_path_pub.publish(path_msg)
        self.get_logger().info(
            f"[PATH_GEN] '{shape}' gerado: {len(pts)} pontos publicados em /planned_path"
        )

        if spec.get('execute', False):
            waypoints = [[round(x, 4), round(y, 4)] for (x, y, h) in pts]
            self.execute_pub.publish(String(data=json.dumps(waypoints)))
            self.get_logger().info(
                f"[PATH_GEN] Tambem enviado pro path_executor ({len(waypoints)} waypoints)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
