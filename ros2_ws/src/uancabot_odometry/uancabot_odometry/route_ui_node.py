#!/usr/bin/env python3
"""
UancaBot Route UI (ROS2 Jazzy)

Pagina web de planejamento de rotas, servida pelo proprio robo:
    http://<ip-da-radxa>:8080

So usa a biblioteca padrao do Python (http.server), sem dependencias novas.
A geometria das rotas e a mesma do path_generator (path_tools.py), e a
execucao vai pro path_follower via /uancabot/follow_path.

API (JSON):
  GET  /                  a pagina
  GET  /api/state         pose, trajeto real, rota planejada, status, STOP
  POST /api/preview       {"mode":"shape","spec":{...}} ou
                          {"mode":"draw","points":[[x,y],...],"smooth":b,"closed":b}
                          -> pontos da rota (frame odom), sem mover o robo
  POST /api/execute       mesmo corpo do preview; re-ancora na pose atual e executa
  POST /api/abort         cancela a rota atual
  POST /api/stop          parada de emergencia (trava)
  POST /api/release       libera a parada de emergencia
  POST /api/reset_odom    zera a odometria e limpa o trajeto real

Rotas por forma ("shape") sao relativas ao robo (comecam onde ele esta,
olhando pra frente). Rotas desenhadas ("draw") sao pontos absolutos no
chao (frame odom) e sempre comecam na posicao atual do robo.
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path, Odometry

from uancabot_odometry.path_tools import (
    build_shape, resample, to_frame, headings, chaikin, dedupe, path_length,
)


def find_html():
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('uancabot_odometry'),
                                       'web', 'route_ui.html'))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, '..', 'web', 'route_ui.html'))
    candidates.append(os.path.join(here, 'route_ui.html'))
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def decimate(pts, max_n):
    if len(pts) <= max_n:
        return pts
    step = len(pts) / max_n
    out = [pts[int(i * step)] for i in range(max_n)]
    out.append(pts[-1])
    return out


class RouteUI(Node):
    def __init__(self):
        super().__init__('uancabot_route_ui')
        self.declare_parameter('port', 8080)
        self.declare_parameter('spacing_m', 0.05)
        self.declare_parameter('corner_deg', 35.0)
        self.port = int(self.get_parameter('port').value)
        self.spacing = float(self.get_parameter('spacing_m').value)
        self.corner_deg = float(self.get_parameter('corner_deg').value)

        self.lock = threading.Lock()
        self.pose = None
        self.odom_time = 0.0
        self.actual = []
        self.planned = []
        self.lookahead = None
        self.status = 'idle'
        self.estop = False

        self.planned_pub = self.create_publisher(Path, '/planned_path', 10)
        self.follow_pub = self.create_publisher(Path, '/uancabot/follow_path', 10)
        self.abort_pub = self.create_publisher(Empty, '/uancabot/abort_path', 10)
        self.estop_pub = self.create_publisher(Bool, '/uancabot/estop', 10)
        self.reset_pub = self.create_publisher(Empty, '/uancabot/reset_odom', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/actual_path', self.actual_cb, 10)
        self.create_subscription(Path, '/planned_path', self.planned_cb, 10)
        self.create_subscription(String, '/uancabot/follow_status', self.status_cb, 10)
        self.create_subscription(Bool, '/uancabot/estop', self.estop_cb, 10)
        self.create_subscription(PointStamped, '/uancabot/lookahead', self.lookahead_cb, 10)

        self.html_path = find_html()
        if self.html_path is None:
            self.get_logger().error('route_ui.html nao encontrado -- confira o setup.py')
        self.server = ThreadingHTTPServer(('0.0.0.0', self.port), self.make_handler())
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Pagina de rotas em http://<ip-da-radxa>:{self.port}')

    # ───────── ROS ─────────

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        with self.lock:
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                         2.0 * math.atan2(q.z, q.w))
            self.odom_time = time.monotonic()

    def actual_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.actual = pts

    def planned_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.planned = pts

    def status_cb(self, msg):
        with self.lock:
            self.status = msg.data

    def estop_cb(self, msg):
        with self.lock:
            self.estop = bool(msg.data)

    def lookahead_cb(self, msg):
        with self.lock:
            self.lookahead = (msg.point.x, msg.point.y)

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

    # ───────── logica da API ─────────

    def current_pose(self):
        with self.lock:
            pose, age = self.pose, time.monotonic() - self.odom_time
        if pose is None or age > 1.0:
            raise ValueError('sem odometria do robo (o motor_bridge esta rodando?)')
        return pose

    def build_route(self, req):
        pose = self.current_pose()
        mode = req.get('mode')
        if mode == 'shape':
            spec = req.get('spec')
            if not isinstance(spec, dict):
                raise ValueError('"spec" precisa ser um objeto')
            local = resample(build_shape(spec), self.spacing, self.corner_deg)
            return to_frame(local, *pose)
        if mode == 'draw':
            raw = req.get('points') or []
            pts = [(pose[0], pose[1])] + [(float(p[0]), float(p[1])) for p in raw]
            if req.get('closed'):
                pts.append((pose[0], pose[1]))
            pts = dedupe(pts)
            if len(pts) < 2:
                raise ValueError('desenhe pelo menos 1 ponto')
            if req.get('smooth'):
                pts = chaikin(pts)
            return resample(pts, self.spacing, self.corner_deg)
        raise ValueError('mode precisa ser "shape" ou "draw"')

    def api_state(self):
        with self.lock:
            pose = self.pose
            age = time.monotonic() - self.odom_time if pose else None
            return {
                'pose': pose,
                'odom_age': age,
                'actual': decimate(self.actual, 1500),
                'planned': decimate(self.planned, 1500),
                'lookahead': self.lookahead if self.status == 'running' else None,
                'status': self.status,
                'estop': self.estop,
            }

    def api_preview(self, req):
        pts = self.build_route(req)
        self.planned_pub.publish(self.make_path(pts))
        return {'ok': True, 'points': pts, 'length': path_length(pts)}

    def api_execute(self, req):
        with self.lock:
            if self.estop:
                raise ValueError('STOP ativo: libere a parada de emergencia primeiro')
        pts = self.build_route(req)
        msg = self.make_path(pts)
        self.planned_pub.publish(msg)
        self.follow_pub.publish(msg)
        self.get_logger().info(f'[UI] Rota enviada: {len(pts)} pontos, {path_length(pts):.2f} m')
        return {'ok': True, 'points': pts, 'length': path_length(pts)}

    def api_simple(self, name):
        if name == 'abort':
            self.abort_pub.publish(Empty())
        elif name == 'stop':
            self.estop_pub.publish(Bool(data=True))
            with self.lock:
                self.estop = True
        elif name == 'release':
            self.estop_pub.publish(Bool(data=False))
            with self.lock:
                self.estop = False
        elif name == 'reset_odom':
            self.reset_pub.publish(Empty())
            with self.lock:
                self.planned = []
                self.actual = []
        self.get_logger().info(f'[UI] {name}')
        return {'ok': True}

    # ───────── HTTP ─────────

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def send_json(self, code, obj):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    if node.html_path is None:
                        self.send_error(500, 'route_ui.html nao encontrado')
                        return
                    with open(node.html_path, 'rb') as f:
                        body = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == '/api/state':
                    self.send_json(200, node.api_state())
                else:
                    self.send_error(404)

            def do_POST(self):
                try:
                    n = int(self.headers.get('Content-Length') or 0)
                    req = json.loads(self.rfile.read(n) or b'{}') if n else {}
                    if self.path == '/api/preview':
                        out = node.api_preview(req)
                    elif self.path == '/api/execute':
                        out = node.api_execute(req)
                    elif self.path in ('/api/abort', '/api/stop', '/api/release', '/api/reset_odom'):
                        out = node.api_simple(self.path.rsplit('/', 1)[1])
                    else:
                        self.send_error(404)
                        return
                    self.send_json(200, out)
                except (ValueError, KeyError, TypeError) as e:
                    self.send_json(400, {'ok': False, 'error': str(e)})
                except Exception as e:
                    node.get_logger().error(f'[UI] Erro interno: {e!r}')
                    self.send_json(500, {'ok': False, 'error': f'erro interno: {e}'})

        return Handler

    def shutdown(self):
        try:
            self.server.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RouteUI()
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
