#!/usr/bin/env python3
"""
UancaBot Path Executor Node (ROS2 Jazzy)

Recebe uma lista de waypoints (x, y em metros, relativos a origem do /odom)
e guia o robo ate cada um, em malha fechada, usando o /odom real como
feedback -- nao usa tempo fixo, nao acumula erro como o comando 'g' do
firmware (que e open-loop).

Estrategia por waypoint ("turn-then-drive"):
  1. Calcula o rumo necessario (atan2) ate o alvo
  2. Gira ('p' = CCW puro, no lugar | '2' = CW, com deriva lateral --
     tudo bem, o proximo recalculo de rumo absorve qualquer deriva)
     ate o erro de rumo ficar dentro da tolerancia
  3. Anda ('f') ate a distancia ate o alvo ficar dentro da tolerancia
  4. Passa pro proximo waypoint

Uso (via topico, mesmo padrao do raw_cmd):
  ros2 topic pub --once /uancabot/execute_path std_msgs/String \
    'data: "[[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]"'

Abortar a qualquer momento:
  ros2 topic pub --once /uancabot/abort_path std_msgs/Empty "{}"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import math
import threading
import time

from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry


def angle_diff(target_deg, current_deg):
    """Menor diferenca angular com sinal, em graus, no range (-180, 180]."""
    d = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return d


class PathExecutorNode(Node):
    def __init__(self):
        super().__init__('uancabot_path_executor')

        # ───── Parametros (ajustaveis via ros2 param / launch) ─────
        self.declare_parameter('turn_tolerance_deg', 5.0)
        self.declare_parameter('distance_tolerance_m', 0.03)
        self.declare_parameter('poll_interval_s', 0.1)
        self.declare_parameter('waypoint_timeout_s', 20.0)

        self.turn_tolerance_deg = self.get_parameter('turn_tolerance_deg').value
        self.distance_tolerance_m = self.get_parameter('distance_tolerance_m').value
        self.poll_interval_s = self.get_parameter('poll_interval_s').value
        self.waypoint_timeout_s = self.get_parameter('waypoint_timeout_s').value

        # ───── Estado da pose atual (atualizado pelo /odom) ─────
        self.pose_lock = threading.Lock()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw_deg = 0.0
        self.pose_received = False

        # ───── Estado de execucao ─────
        self.abort_event = threading.Event()
        self.exec_thread = None
        self.last_cmd_sent = None  # evita reenviar o mesmo comando toda hora

        # ───── QoS compativel com o publisher do uancabot_odometry_node ─────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.cmd_pub = self.create_publisher(String, '/uancabot/raw_cmd', 10)

        self.path_sub = self.create_subscription(
            String, '/uancabot/execute_path', self.execute_path_callback, 10
        )
        self.abort_sub = self.create_subscription(
            Empty, '/uancabot/abort_path', self.abort_callback, 10
        )

        self.get_logger().info("Path executor pronto.")
        self.get_logger().info(
            f"  turn_tolerance={self.turn_tolerance_deg}deg  "
            f"dist_tolerance={self.distance_tolerance_m}m  "
            f"timeout/waypoint={self.waypoint_timeout_s}s"
        )
        self.get_logger().info(
            "Uso: ros2 topic pub --once /uancabot/execute_path std_msgs/String "
            "'data: \"[[1.0,0.0],[1.0,1.0]]\"'"
        )

    # ───── Callbacks ─────

    def odom_callback(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = 2.0 * math.atan2(qz, qw)
        with self.pose_lock:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_yaw_deg = math.degrees(yaw_rad)
            self.pose_received = True

    def abort_callback(self, msg):
        self.get_logger().warn("[ABORT] Sinal de aborto recebido")
        self.abort_event.set()

    def execute_path_callback(self, msg):
        if self.exec_thread is not None and self.exec_thread.is_alive():
            self.get_logger().warn(
                "[PATH] Ja existe uma trajetoria em execucao -- ignorando novo pedido. "
                "Aborte a atual primeiro (/uancabot/abort_path) se quiser trocar."
            )
            return

        try:
            waypoints = json.loads(msg.data)
            if not isinstance(waypoints, list) or not waypoints:
                raise ValueError("esperado uma lista nao-vazia de [x, y]")
            for wp in waypoints:
                if not (isinstance(wp, list) and len(wp) == 2):
                    raise ValueError(f"waypoint invalido: {wp!r} (esperado [x, y])")
        except Exception as e:
            self.get_logger().error(f"[PATH] JSON invalido: {e}")
            return

        self.abort_event.clear()
        self.exec_thread = threading.Thread(
            target=self.execute_path, args=(waypoints,), daemon=True
        )
        self.exec_thread.start()

    # ───── Helpers de baixo nivel ─────

    def get_pose(self):
        with self.pose_lock:
            return self.current_x, self.current_y, self.current_yaw_deg

    def send_cmd(self, char):
        """So publica se for diferente do ultimo comando (evita spam na serial)."""
        if char != self.last_cmd_sent:
            self.cmd_pub.publish(String(data=char))
            self.last_cmd_sent = char

    def stop(self):
        self.send_cmd('s')

    # ───── Logica de controle ─────

    def goto_point(self, target_x, target_y):
        """Gira ate alinhar o rumo, depois anda ate o ponto. Malha fechada."""
        wp_start = time.time()

        # ── Fase 1: girar ──
        while rclpy.ok() and not self.abort_event.is_set():
            if time.time() - wp_start > self.waypoint_timeout_s:
                self.get_logger().warn("[PATH] Timeout na fase de giro, abortando waypoint")
                self.stop()
                return False

            x, y, yaw = self.get_pose()
            dx, dy = target_x - x, target_y - y
            dist = math.hypot(dx, dy)
            if dist < self.distance_tolerance_m:
                break  # ja esta em cima do alvo, nao precisa girar

            target_heading = -math.degrees(math.atan2(dy, dx))
            err = angle_diff(target_heading, yaw)

            if abs(err) <= self.turn_tolerance_deg:
                break

            # err > 0 -> precisa girar CCW ('p', giro puro no lugar)
            # err < 0 -> precisa girar CW ('2', pivota com deriva -- ok,
            #            o proximo recalculo de rumo absorve a deriva)
            self.send_cmd('2' if err > 0 else 'p')
            time.sleep(self.poll_interval_s)

        self.stop()
        time.sleep(0.3)  # deixa o "coast" (inercia) assentar antes de medir de novo

        if self.abort_event.is_set():
            return False

        # ── Fase 2: andar ──
        wp_start = time.time()
        while rclpy.ok() and not self.abort_event.is_set():
            if time.time() - wp_start > self.waypoint_timeout_s:
                self.get_logger().warn("[PATH] Timeout na fase de avanco, abortando waypoint")
                self.stop()
                return False

            x, y, yaw = self.get_pose()
            dist = math.hypot(target_x - x, target_y - y)

            if dist <= self.distance_tolerance_m:
                break

            self.send_cmd('f')
            time.sleep(self.poll_interval_s)

        self.stop()
        time.sleep(0.3)
        return not self.abort_event.is_set()

    def execute_path(self, waypoints):
        self.get_logger().info(f"[PATH] Iniciando trajetoria com {len(waypoints)} waypoint(s)")

        for i, (wx, wy) in enumerate(waypoints):
            if self.abort_event.is_set():
                break
            self.get_logger().info(f"[PATH] Indo para waypoint {i+1}/{len(waypoints)}: ({wx:.3f}, {wy:.3f})")
            ok = self.goto_point(wx, wy)
            x, y, yaw = self.get_pose()
            if ok:
                self.get_logger().info(
                    f"[PATH] Waypoint {i+1} alcancado: pose=({x:.3f}, {y:.3f}, {yaw:.1f}deg)"
                )
            else:
                self.get_logger().warn(
                    f"[PATH] Waypoint {i+1} interrompido/timeout: pose=({x:.3f}, {y:.3f}, {yaw:.1f}deg)"
                )
                break

        self.stop()
        self.last_cmd_sent = None  # permite reenviar 's' numa proxima chamada se preciso

        if self.abort_event.is_set():
            self.get_logger().warn("[PATH] Trajetoria abortada.")
        else:
            self.get_logger().info("[PATH] Trajetoria concluida.")


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.abort_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
