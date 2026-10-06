#!/usr/bin/env python3
"""
UancaBot Path Follower (ROS2 Jazzy)

Segue uma rota (nav_msgs/Path) com Pure Pursuit, publicando /cmd_vel pro
motor_bridge. Nos cantos da rota o robo para, gira no lugar e continua,
mantendo a geometria exata (logica em path_tools.PathFollower).

Entradas:
  /uancabot/follow_path  nav_msgs/Path   rota em frame odom (ou base_link,
                                          que e convertido pela pose atual)
  /odom                  nav_msgs/Odometry
  /uancabot/abort_path   std_msgs/Empty  cancela a rota e para
  /uancabot/estop        std_msgs/Bool   true tambem cancela a rota

Saidas:
  /cmd_vel                geometry_msgs/Twist (so enquanto segue uma rota,
                          entao o Teleop do Foxglove continua funcionando)
  /uancabot/follow_status std_msgs/String  idle | running | done | aborted: ...
  /uancabot/lookahead     geometry_msgs/PointStamped (alvo do Pure Pursuit,
                          pra ver no painel 3D)

Seguranca: se o /odom parar de chegar por odom_timeout_s, a rota e
cancelada. Mesmo que este node trave, o motor_bridge zera a velocidade em
0.5s sem /cmd_vel, e o firmware para os motores sozinho.
"""

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty, Bool
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Path, Odometry

from uancabot_odometry.path_tools import PathFollower, to_frame


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('uancabot_path_follower')
        params = {}
        for name, value in PathFollower.DEFAULTS.items():
            self.declare_parameter(name, float(value))
            params[name] = float(self.get_parameter(name).value)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.odom_timeout = float(self.get_parameter('odom_timeout_s').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.ctrl = PathFollower(**params)
        self.pose = None
        self.last_odom = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/uancabot/follow_status', 10)
        self.look_pub = self.create_publisher(PointStamped, '/uancabot/lookahead', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/uancabot/follow_path', self.path_cb, 10)
        self.create_subscription(Empty, '/uancabot/abort_path', self.abort_cb, 10)
        self.create_subscription(Bool, '/uancabot/estop', self.estop_cb, 10)
        self.create_timer(1.0 / rate, self.loop)

        p = self.ctrl.p
        self.get_logger().info('Path follower pronto.')
        self.get_logger().info(
            f'  lookahead={p["lookahead_m"]}m  v_max={p["v_max"]}m/s  w_max={p["w_max"]}rad/s  '
            f'canto>{p["corner_deg"]}deg  chegada<{p["goal_tol_m"]*100:.1f}cm')
        self.publish_status('idle')

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def send(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        th = 2.0 * math.atan2(q.z, q.w)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, th)
        self.last_odom = time.monotonic()

    def path_cb(self, msg):
        if self.pose is None or time.monotonic() - self.last_odom > self.odom_timeout:
            self.get_logger().error('[FOLLOW] Sem /odom recente -- rota recusada')
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if msg.header.frame_id == 'base_link':
            pts = to_frame(pts, *self.pose)
        if self.ctrl.active:
            self.get_logger().warn('[FOLLOW] Nova rota recebida -- substituindo a atual')
        try:
            pieces, length = self.ctrl.start(pts, time.monotonic())
        except ValueError as e:
            self.get_logger().error(f'[FOLLOW] Rota invalida: {e}')
            return
        self.get_logger().info(f'[FOLLOW] Iniciando: {length:.2f} m, {pieces} trecho(s)')
        self.publish_status('running')

    def finish(self, status):
        self.ctrl.stop()
        for _ in range(3):
            self.send(0.0, 0.0)
        self.publish_status(status)
        x, y, th = self.pose if self.pose else (0.0, 0.0, 0.0)
        msg = f'[FOLLOW] {status}  pose=({x:.3f}, {y:.3f}, {math.degrees(th):.1f}deg)'
        if status == 'done':
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)

    def abort_cb(self, _msg):
        if self.ctrl.active:
            self.finish('aborted: pedido do usuario')

    def estop_cb(self, msg):
        if msg.data and self.ctrl.active:
            self.finish('aborted: parada de emergencia')

    def loop(self):
        if not self.ctrl.active:
            return
        now = time.monotonic()
        if now - self.last_odom > self.odom_timeout:
            self.finish('aborted: /odom parou de chegar')
            return
        v, w, status = self.ctrl.step(*self.pose, now)
        if status != 'running':
            self.finish(status)
            return
        self.send(v, w)

        lx, ly = self.ctrl.lookahead
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'odom'
        pt.point.x = lx
        pt.point.y = ly
        self.look_pub.publish(pt)

        self.get_logger().info(
            f'[FOLLOW] trecho {self.ctrl.pi + 1}/{len(self.ctrl.pieces)} {self.ctrl.state:6}  '
            f'v={v:.3f} w={w:.2f}  restante={self.ctrl.pieces[self.ctrl.pi].remaining[self.ctrl.idx]:.2f}m',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ctrl.active:
            for _ in range(3):
                node.send(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
