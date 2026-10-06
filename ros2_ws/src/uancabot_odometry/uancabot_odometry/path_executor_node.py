#!/usr/bin/env python3
"""
UancaBot Path Executor Node (ROS2 Jazzy)

Recebe uma lista de waypoints (x, y em metros, relativos a origem do /odom)
e guia o robo ate cada um, em malha fechada, usando o /odom real como
feedback.

Estrategia por waypoint ("turn-then-drive"):
  1. Calcula o rumo necessario (atan2) ate o alvo
  2. Gira ('p' = CCW puro, no lugar | '2' = CW, com deriva lateral --
     tudo bem, o proximo recalculo de rumo absorve qualquer deriva)
     ate o erro de rumo ficar dentro da tolerancia
  3. Anda ('f') ate a DISTANCIA PERCORRIDA bater a distancia alvo --
     IMPORTANTE: nao usa "distancia ATE o alvo" (ver nota abaixo)
  4. Passa pro proximo waypoint

NOTA IMPORTANTE sobre a fase de avanco (2026-09, bug real encontrado e
corrigido): a primeira versao desse node checava "distancia ATE o alvo"
a cada leitura de /odom pra decidir quando parar. Isso parece certo, mas
tem uma falha fatal com feedback infrequente (~1.4s por leitura, ritmo do
firmware) e velocidade fixa do motor (sem controle de PWM continuo, so
liga/desliga): o robo anda ~35-40cm entre duas leituras consecutivas.
Se o alvo esta mais perto que isso, o robo PASSA por cima dele entre uma
leitura e outra -- e como e uma linha reta, a distancia ATE o alvo so
CRESCE depois que voce passa por cima, nunca mais volta a ficar pequena.
Resultado observado em teste real: o robo passou reto por um alvo a
0.25m e continuou andando por 4.9m antes do timeout de seguranca (20s)
interromper. Precisou de STOP manual.

Fix: em vez de checar distancia ATE o alvo (que so cresce depois que
passa), checa DISTANCIA PERCORRIDA desde o inicio da perna (que so
cresce enquanto anda, e sempre cruza o valor alvo em algum momento,
garantindo que o loop sempre termina sozinho). Precisao ainda fica
limitada pelo mesmo hardware (~35-40cm de "passo" entre leituras) --
ainda assim, o loop agora SEMPRE para sozinho, nunca mais roda ate o
timeout. Recomendado usar waypoints espacados generosamente (0.5m+)
ate que o firmware suporte controle continuo de velocidade.

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
    d = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return d


class PathExecutorNode(Node):
    def __init__(self):
        super().__init__('uancabot_path_executor')

        self.declare_parameter('turn_tolerance_deg', 5.0)
        self.declare_parameter('distance_tolerance_m', 0.03)
        self.declare_parameter('poll_interval_s', 0.1)
        self.declare_parameter('waypoint_timeout_s', 20.0)

        self.turn_tolerance_deg = self.get_parameter('turn_tolerance_deg').value
        self.distance_tolerance_m = self.get_parameter('distance_tolerance_m').value
        self.poll_interval_s = self.get_parameter('poll_interval_s').value
        self.waypoint_timeout_s = self.get_parameter('waypoint_timeout_s').value

        self.pose_lock = threading.Lock()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw_deg = 0.0
        self.pose_received = False

        self.abort_event = threading.Event()
        self.exec_thread = None
        self.last_cmd_sent = None

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

    def get_pose(self):
        with self.pose_lock:
            return self.current_x, self.current_y, self.current_yaw_deg

    def send_cmd(self, char):
        if char != self.last_cmd_sent:
            self.cmd_pub.publish(String(data=char))
            self.last_cmd_sent = char

    def stop(self):
        self.send_cmd('s')

    def goto_point(self, target_x, target_y):
        wp_start = time.time()

        while rclpy.ok() and not self.abort_event.is_set():
            if time.time() - wp_start > self.waypoint_timeout_s:
                self.get_logger().warn("[PATH] Timeout na fase de giro, abortando waypoint")
                self.stop()
                return False

            x, y, yaw = self.get_pose()
            dx, dy = target_x - x, target_y - y
            dist = math.hypot(dx, dy)
            if dist < self.distance_tolerance_m:
                break

            target_heading = -math.degrees(math.atan2(dy, dx))
            err = angle_diff(target_heading, yaw)

            if abs(err) <= self.turn_tolerance_deg:
                break

            self.send_cmd('2' if err > 0 else 'p')
            time.sleep(self.poll_interval_s)

        self.stop()
        time.sleep(0.3)

        if self.abort_event.is_set():
            return False

        x0, y0, _ = self.get_pose()
        target_dist = math.hypot(target_x - x0, target_y - y0)

        wp_start = time.time()
        while rclpy.ok() and not self.abort_event.is_set():
            if time.time() - wp_start > self.waypoint_timeout_s:
                self.get_logger().warn("[PATH] Timeout na fase de avanco, abortando waypoint")
                self.stop()
                return False

            x, y, yaw = self.get_pose()
            traveled = math.hypot(x - x0, y - y0)

            if traveled >= target_dist - self.distance_tolerance_m:
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
        self.last_cmd_sent = None

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
