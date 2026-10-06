#!/usr/bin/env python3
"""
UancaBot Path Generator (ROS2 Jazzy)

Gera rotas e manda pro path_follower. Tres jeitos de criar uma rota:

1) FORMAS PRONTAS e ROTAS POR DISTANCIA  -> /uancabot/generate_path (std_msgs/String, JSON)
   A rota e relativa ao robo: comeca onde ele esta, olhando pra frente.
   Exemplos:
     {"shape": "line", "length_m": 1.0}
     {"shape": "square", "side_m": 1.0}
     {"shape": "rectangle", "width_m": 1.2, "height_m": 0.6, "corner_radius_m": 0.2}
     {"shape": "circle", "radius_m": 0.5, "direction": "left"}
     {"shape": "arc", "radius_m": 0.6, "angle_deg": 90}
     {"shape": "figure8", "radius_m": 0.4}
     {"shape": "slalom", "length_m": 2.0, "amplitude_m": 0.25, "wavelength_m": 1.0}
     {"shape": "segments", "segments": [{"forward": 1.0}, {"turn": 90}, {"forward": 0.5}]}
     {"shape": "polyline", "points": [[0.8, 0], [0.8, 0.6]], "smooth": false, "closed": false}
   Sem "execute": so mostra a previa em /planned_path (robo NAO se move).
   Com "execute": true, ja manda executar.

2) EXECUTAR A PREVIA -> /uancabot/execute_preview (std_msgs/Empty)
   Executa a ultima previa gerada, re-ancorada na pose atual do robo.

3) DESENHO PONTO A PONTO no painel 3D do Foxglove
   Cada clique com a ferramenta "Publish point" chega em /clicked_point e
   vira um ponto da rota (posicao absoluta no chao, frame odom). A rota
   sempre comeca na posicao atual do robo. Previa em /uancabot/draft_path.
   Comandos em /uancabot/draft_cmd (std_msgs/String):
     "undo"     remove o ultimo ponto
     "clear"    apaga o desenho
     "smooth"   liga/desliga suavizacao (curvas) -- desligado = cantos com pivo
     "close"    liga/desliga voltar ao ponto de partida no final
     "execute"  manda o desenho pro robo seguir

Saidas:
  /planned_path          rota que sera/esta sendo executada (frame odom)
  /uancabot/draft_path   previa do desenho em andamento (frame odom)
  /uancabot/follow_path  rota enviada pro path_follower (frame odom)
"""

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path, Odometry

from uancabot_odometry.path_tools import (
    build_shape, resample, to_frame, headings, chaikin, dedupe, path_length,
)


class PathGenerator(Node):
    def __init__(self):
        super().__init__('uancabot_path_generator')
        self.declare_parameter('spacing_m', 0.05)
        self.declare_parameter('corner_deg', 35.0)
        self.spacing = float(self.get_parameter('spacing_m').value)
        self.corner_deg = float(self.get_parameter('corner_deg').value)

        self.pose = None              # (x, y, theta) do /odom
        self.preview_local = None     # ultima forma gerada, relativa ao robo
        self.preview_name = ''
        self.draft = []               # pontos clicados, frame odom
        self.draft_smooth = False
        self.draft_closed = False
        self.warned_frames = set()

        self.planned_pub = self.create_publisher(Path, '/planned_path', 10)
        self.draft_pub = self.create_publisher(Path, '/uancabot/draft_path', 10)
        self.follow_pub = self.create_publisher(Path, '/uancabot/follow_path', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/uancabot/generate_path', self.generate_cb, 10)
        self.create_subscription(Empty, '/uancabot/execute_preview', self.execute_preview_cb, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_cb, 10)
        self.create_subscription(String, '/uancabot/draft_cmd', self.draft_cmd_cb, 10)
        self.create_timer(0.5, self.publish_draft)

        self.get_logger().info('Path generator pronto.')
        self.get_logger().info('  Formas: line, arc, circle, square, rectangle, figure8, slalom, '
                               'segments, polyline')
        self.get_logger().info('  Desenho: clique pontos no painel 3D (Publish point) e use '
                               '/uancabot/draft_cmd (undo, clear, smooth, close, execute)')

    # ───────── utilitarios ─────────

    def make_path(self, pts):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for (x, y), h in zip(pts, headings(pts)):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(h / 2.0)
            ps.pose.orientation.w = math.cos(h / 2.0)
            msg.poses.append(ps)
        return msg

    def need_pose(self):
        if self.pose is None:
            self.get_logger().error('Ainda nao recebi /odom -- o motor_bridge esta rodando?')
            return False
        return True

    def send_to_follower(self, pts_odom, name):
        msg = self.make_path(pts_odom)
        self.planned_pub.publish(msg)
        self.follow_pub.publish(msg)
        self.get_logger().info(f'[EXEC] "{name}" enviado pro seguidor: {len(pts_odom)} pontos, '
                               f'{path_length(pts_odom):.2f} m')

    # ───────── callbacks ─────────

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        th = 2.0 * math.atan2(q.z, q.w)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, th)

    def generate_cb(self, msg):
        try:
            spec = json.loads(msg.data)
            if not isinstance(spec, dict):
                raise ValueError('o JSON precisa ser um objeto {...}')
            local = resample(build_shape(spec), float(spec.get('spacing_m', self.spacing)),
                             self.corner_deg)
        except Exception as e:
            self.get_logger().error(f'[GEN] Rota invalida: {e}')
            return
        if not self.need_pose():
            return
        self.preview_local = local
        self.preview_name = str(spec.get('shape'))
        self.planned_pub.publish(self.make_path(to_frame(local, *self.pose)))
        self.get_logger().info(f'[GEN] Previa "{self.preview_name}": {len(local)} pontos, '
                               f'{path_length(local):.2f} m (robo parado)')
        if spec.get('execute', False):
            self.execute_preview_cb(None)

    def execute_preview_cb(self, _msg):
        if self.preview_local is None:
            self.get_logger().warn('[EXEC] Nenhuma previa gerada ainda')
            return
        if not self.need_pose():
            return
        self.send_to_follower(to_frame(self.preview_local, *self.pose), self.preview_name)

    def clicked_cb(self, msg):
        frame = msg.header.frame_id
        x, y = msg.point.x, msg.point.y
        if frame == 'base_link':
            if not self.need_pose():
                return
            x, y = to_frame([(x, y)], *self.pose)[0]
        elif frame not in ('odom', '') and frame not in self.warned_frames:
            self.warned_frames.add(frame)
            self.get_logger().warn(f'[DRAW] Ponto no frame "{frame}", tratando como odom. '
                                   'Use Display frame = odom no painel 3D.')
        self.draft.append((x, y))
        self.get_logger().info(f'[DRAW] Ponto {len(self.draft)}: ({x:.2f}, {y:.2f})')
        self.publish_draft()

    def draft_points(self):
        if self.pose is None or not self.draft:
            return []
        start = (self.pose[0], self.pose[1])
        pts = [start] + self.draft
        if self.draft_closed:
            pts.append(start)
        pts = dedupe(pts)
        if len(pts) < 2:
            return []
        if self.draft_smooth:
            pts = chaikin(pts)
        return resample(pts, self.spacing, self.corner_deg)

    def publish_draft(self):
        self.draft_pub.publish(self.make_path(self.draft_points()))

    def draft_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'undo':
            if self.draft:
                self.draft.pop()
            self.get_logger().info(f'[DRAW] Desfeito ({len(self.draft)} pontos)')
        elif cmd == 'clear':
            self.draft = []
            self.get_logger().info('[DRAW] Desenho apagado')
        elif cmd == 'smooth':
            self.draft_smooth = not self.draft_smooth
            self.get_logger().info(f'[DRAW] Suavizacao {"LIGADA" if self.draft_smooth else "DESLIGADA"}')
        elif cmd == 'close':
            self.draft_closed = not self.draft_closed
            self.get_logger().info(f'[DRAW] Voltar ao inicio {"LIGADO" if self.draft_closed else "DESLIGADO"}')
        elif cmd == 'execute':
            pts = self.draft_points()
            if len(pts) < 2:
                self.get_logger().warn('[DRAW] Desenho vazio: clique pelo menos 1 ponto')
                return
            self.send_to_follower(pts, 'desenho')
            self.draft = []
        else:
            self.get_logger().warn(f'[DRAW] Comando desconhecido: "{cmd}" '
                                   '(use undo, clear, smooth, close, execute)')
            return
        self.publish_draft()


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
