"""
path_tools -- geometria de rotas e seguidor de trajetoria do UancaBot.

Modulo puro (sem ROS), pra poder ser testado e simulado fora do robo.
Usado pelo path_generator_node (formas e desenho) e pelo
path_follower_node (controle).

CONVENCAO DE COORDENADAS (REP-103):
  x = frente do robo, y = esquerda do robo, angulo positivo = anti-horario.
  Rotas geradas por forma sao RELATIVAS ao robo: comecam em (0,0) olhando
  pra +x, e depois sao ancoradas na pose atual do /odom.

CANTOS:
  Um vertice com mudanca de rumo acima de corner_deg (padrao 35 graus) e
  tratado como CANTO. O seguidor para exatamente no canto, gira no lugar
  ate alinhar com o proximo trecho e segue. Isso deixa quadrados e rotas
  por segmentos com a geometria exata (sem "cortar" a quina). Curvas
  suaves (arcos, circulos, desenho suavizado) nao geram cantos e sao
  seguidas continuamente pelo Pure Pursuit.
"""

import math


# ───────────────────────── utilitarios ─────────────────────────

def wrap(a):
    """Normaliza um angulo pra (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(pts):
    return sum(dist(a, b) for a, b in zip(pts, pts[1:]))


def dedupe(pts, eps=1e-6):
    out = []
    for p in pts:
        p = (float(p[0]), float(p[1]))
        if not out or dist(out[-1], p) > eps:
            out.append(p)
    return out


def corner_indices(pts, corner_deg):
    """Indices dos vertices onde o rumo muda mais que corner_deg."""
    lim = math.radians(corner_deg)
    out = []
    for i in range(1, len(pts) - 1):
        h1 = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
        h2 = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
        if abs(wrap(h2 - h1)) > lim:
            out.append(i)
    return out


def _resample_chunk(chunk, spacing):
    total = path_length(chunk)
    if total < 1e-9:
        return [chunk[0]]
    n = max(1, int(round(total / spacing)))
    step = total / n
    out = [chunk[0]]
    seg = 0
    seg_start = 0.0
    seg_len = dist(chunk[0], chunk[1])
    for k in range(1, n):
        target = k * step
        while seg < len(chunk) - 2 and seg_start + seg_len < target:
            seg_start += seg_len
            seg += 1
            seg_len = dist(chunk[seg], chunk[seg + 1])
        t = 0.0 if seg_len < 1e-12 else (target - seg_start) / seg_len
        a, b = chunk[seg], chunk[seg + 1]
        out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    out.append(chunk[-1])
    return out


def resample(pts, spacing=0.05, corner_deg=35.0):
    """Reamostra a cada `spacing` metros, preservando exatamente os cantos."""
    pts = dedupe(pts)
    if len(pts) < 2:
        return pts
    bounds = [0] + corner_indices(pts, corner_deg) + [len(pts) - 1]
    out = []
    for a, b in zip(bounds, bounds[1:]):
        part = _resample_chunk(pts[a:b + 1], spacing)
        out.extend(part if not out else part[1:])
    return out


def chaikin(pts, iterations=3):
    """Suaviza uma polilinha (cortando cantos), mantendo inicio e fim."""
    pts = list(pts)
    for _ in range(iterations):
        if len(pts) < 3:
            return pts
        new = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            new.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            new.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        new.append(pts[-1])
        pts = new
    return pts


def headings(pts):
    """Rumo (rad) em cada ponto, olhando pro proximo."""
    if len(pts) < 2:
        return [0.0] * len(pts)
    hs = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
    hs.append(hs[-1])
    return hs


def to_frame(pts, x0, y0, th0):
    """Leva pontos relativos ao robo pro frame do odom, dada a pose do robo."""
    c, s = math.cos(th0), math.sin(th0)
    return [(x0 + c * x - s * y, y0 + s * x + c * y) for x, y in pts]


def to_local(pts, x0, y0, th0):
    """Inverso de to_frame: pontos do odom pro frame do robo."""
    c, s = math.cos(th0), math.sin(th0)
    out = []
    for x, y in pts:
        dx, dy = x - x0, y - y0
        out.append((c * dx + s * dy, -s * dx + c * dy))
    return out


# ───────────────────────── geradores de forma ─────────────────────────

class Turtle:
    """Desenha rotas por comandos: andar X metros, virar Y graus, arco."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.pts = [(0.0, 0.0)]

    def forward(self, d):
        if d < 0:
            raise ValueError('distancia negativa nao suportada (o robo so segue rotas pra frente)')
        if d == 0:
            return
        self.x += d * math.cos(self.th)
        self.y += d * math.sin(self.th)
        self.pts.append((self.x, self.y))

    def turn(self, deg):
        """Giro no lugar (vira um CANTO na rota). Positivo = esquerda."""
        self.th = wrap(self.th + math.radians(deg))

    def arc(self, radius, deg):
        """Arco de raio `radius`. Angulo positivo = curva pra esquerda."""
        if radius <= 0:
            self.turn(deg)
            return
        if deg == 0:
            return
        sign = 1.0 if deg > 0 else -1.0
        a = math.radians(deg)
        cx = self.x - sign * radius * math.sin(self.th)
        cy = self.y + sign * radius * math.cos(self.th)
        th0 = self.th
        n = max(2, int(abs(deg) / 2.0) + 1)
        for i in range(1, n + 1):
            phi = a * i / n
            self.pts.append((cx + sign * radius * math.sin(th0 + phi),
                             cy - sign * radius * math.cos(th0 + phi)))
        self.x, self.y = self.pts[-1]
        self.th = wrap(th0 + a)


def _num(spec, key, default=None, positive=True):
    if key not in spec:
        if default is None:
            raise ValueError(f'parametro obrigatorio faltando: "{key}"')
        return float(default)
    v = float(spec[key])
    if positive and v <= 0:
        raise ValueError(f'"{key}" precisa ser maior que zero')
    return v


def build_shape(spec):
    """
    Monta a rota (pontos relativos ao robo) a partir de um dict.

    Formas aceitas (todas as medidas em metros, angulos em graus):
      line       length_m
      arc        radius_m, angle_deg (positivo = esquerda)
      circle     radius_m, direction ("left"/"right")
      square     side_m, corner_radius_m (0 = cantos exatos com pivo)
      rectangle  width_m, height_m, corner_radius_m, direction
      figure8    radius_m
      slalom     length_m, amplitude_m, wavelength_m
      segments   segments: lista de {"forward": m} | {"turn": graus}
                 | {"arc": [raio_m, graus]}, turn_radius_m (0 = pivo)
      polyline   points: [[x,y], ...] relativos ao robo (x frente, y esquerda),
                 smooth (bool), closed (bool, volta ao inicio)
    """
    shape = spec.get('shape')
    t = Turtle()

    if shape == 'line':
        t.forward(_num(spec, 'length_m'))

    elif shape == 'arc':
        t.arc(_num(spec, 'radius_m'), _num(spec, 'angle_deg', positive=False))

    elif shape == 'circle':
        sign = -1.0 if spec.get('direction', 'left') == 'right' else 1.0
        t.arc(_num(spec, 'radius_m'), 360.0 * sign)

    elif shape in ('square', 'rectangle'):
        if shape == 'square':
            w = h = _num(spec, 'side_m')
        else:
            w, h = _num(spec, 'width_m'), _num(spec, 'height_m')
        r = _num(spec, 'corner_radius_m', 0.0, positive=False)
        if r < 0 or 2 * r > min(w, h):
            raise ValueError('corner_radius_m precisa ficar entre 0 e metade do menor lado')
        turn = -90.0 if spec.get('direction', 'left') == 'right' else 90.0
        if r == 0:
            for side in (w, h, w, h):
                t.forward(side)
                t.turn(turn)
        else:
            t.forward(w - r)
            t.arc(r, turn)
            t.forward(h - 2 * r)
            t.arc(r, turn)
            t.forward(w - 2 * r)
            t.arc(r, turn)
            t.forward(h - 2 * r)
            t.arc(r, turn)
            t.forward(r)

    elif shape == 'figure8':
        r = _num(spec, 'radius_m')
        t.arc(r, 360.0)
        t.arc(r, -360.0)

    elif shape == 'slalom':
        length = _num(spec, 'length_m')
        amp = _num(spec, 'amplitude_m')
        wl = _num(spec, 'wavelength_m')
        n = max(10, int(length / 0.01))
        t.pts = [(length * i / n, amp * math.sin(2 * math.pi * (length * i / n) / wl))
                 for i in range(n + 1)]

    elif shape == 'segments':
        segs = spec.get('segments')
        if not isinstance(segs, list) or not segs:
            raise ValueError('"segments" precisa ser uma lista nao vazia')
        tr = _num(spec, 'turn_radius_m', 0.0, positive=False)
        for s in segs:
            if not isinstance(s, dict) or len(s) != 1:
                raise ValueError(f'segmento invalido: {s!r}')
            if 'forward' in s:
                t.forward(float(s['forward']))
            elif 'turn' in s:
                if tr > 0:
                    t.arc(tr, float(s['turn']))
                else:
                    t.turn(float(s['turn']))
            elif 'arc' in s:
                radius, deg = s['arc']
                t.arc(float(radius), float(deg))
            else:
                raise ValueError(f'segmento desconhecido: {s!r} (use forward, turn ou arc)')

    elif shape == 'polyline':
        raw = spec.get('points')
        if not isinstance(raw, list) or not raw:
            raise ValueError('"points" precisa ser uma lista de [x, y]')
        user = [(float(p[0]), float(p[1])) for p in raw]
        pts = [(0.0, 0.0)] + user
        if spec.get('closed', False):
            pts.append((0.0, 0.0))
        if spec.get('smooth', False):
            pts = chaikin(dedupe(pts))
        t.pts = pts

    else:
        raise ValueError(
            f'forma desconhecida: {shape!r}. Use: line, arc, circle, square, rectangle, '
            'figure8, slalom, segments, polyline')

    pts = dedupe(t.pts)
    if len(pts) < 2:
        raise ValueError('a rota resultante tem comprimento zero')
    return pts


# ───────────────────────── seguidor de trajetoria ─────────────────────────

class _Piece:
    def __init__(self, pts):
        self.pts = pts
        self.remaining = [0.0] * len(pts)
        for i in range(len(pts) - 2, -1, -1):
            self.remaining[i] = self.remaining[i + 1] + dist(pts[i], pts[i + 1])


class PathFollower:
    """
    Pure Pursuit com pivo nos cantos.

    Estados:
      ALIGN   gira no lugar ate o ponto de lookahead ficar a frente
      FOLLOW  Pure Pursuit: curvatura = 2*y_local / L^2, com reducao de
              velocidade perto do fim de cada trecho e em curvas fechadas
    A rota e quebrada em trechos nos cantos; cada trecho termina com o robo
    parado exatamente no canto (ou ao cruzar a linha de chegada).
    """

    DEFAULTS = {
        'lookahead_m': 0.25,
        'v_max': 0.15,
        'v_min': 0.03,
        'w_max': 1.0,
        'k_align': 1.5,
        'w_align_min': 0.2,
        'align_tol_deg': 6.0,
        'realign_deg': 60.0,
        'goal_tol_m': 0.015,
        'slow_k': 0.8,
        'corner_deg': 35.0,
        'stall_s': 8.0,
        'search_window_m': 0.6,
    }

    def __init__(self, **params):
        p = dict(self.DEFAULTS)
        p.update({k: v for k, v in params.items() if v is not None})
        self.p = p
        self.pieces = []
        self.active = False

    # ---- ciclo de vida ----

    def start(self, pts, t):
        pts = dedupe(pts)
        if len(pts) < 2:
            raise ValueError('rota com menos de 2 pontos')
        bounds = [0] + corner_indices(pts, self.p['corner_deg']) + [len(pts) - 1]
        self.pieces = []
        for a, b in zip(bounds, bounds[1:]):
            chunk = pts[a:b + 1]
            if len(chunk) >= 2 and path_length(chunk) > 1e-3:
                self.pieces.append(_Piece(chunk))
        if not self.pieces:
            raise ValueError('rota sem trechos validos')
        spacing = path_length(pts) / max(1, len(pts) - 1)
        self.window = max(3, int(self.p['search_window_m'] / max(spacing, 1e-3)))
        self.pi = 0
        self.idx = 0
        self.state = 'ALIGN'
        self.active = True
        self._reset_progress(t)
        self.lookahead = pts[0]
        return len(self.pieces), path_length(pts)

    def stop(self):
        self.active = False

    def _reset_progress(self, t):
        self.best_d = float('inf')
        self.best_h = float('inf')
        self.t_best = t

    # ---- passo de controle ----

    def step(self, x, y, th, t):
        """Retorna (v, w, status). status: 'running' | 'done' | 'aborted: motivo'."""
        if not self.active:
            return 0.0, 0.0, 'done'
        p = self.p
        piece = self.pieces[self.pi]
        P = piece.pts
        n = len(P)
        pos = (x, y)

        # ponto mais proximo, so olhando pra frente (evita pular em cruzamentos, ex: o 8)
        hi = min(n - 1, self.idx + self.window)
        best_i, best_d = self.idx, dist(pos, P[self.idx])
        for i in range(self.idx + 1, hi + 1):
            d = dist(pos, P[i])
            if d < best_d:
                best_i, best_d = i, d
        self.idx = best_i

        # fim do trecho: perto do ponto final, ou ja passou da linha de chegada
        d_end = dist(pos, P[-1])
        if self.idx >= n - 2:
            ex, ey = P[-1][0] - P[-2][0], P[-1][1] - P[-2][1]
            el = math.hypot(ex, ey) or 1.0
            along = ((x - P[-1][0]) * ex + (y - P[-1][1]) * ey) / el
            if d_end < p['goal_tol_m'] or along >= 0.0:
                self.pi += 1
                if self.pi >= len(self.pieces):
                    self.active = False
                    return 0.0, 0.0, 'done'
                self.idx = 0
                self.state = 'ALIGN'
                self._reset_progress(t)
                return 0.0, 0.0, 'running'

        # ponto de lookahead
        j = n - 1
        for k in range(self.idx, n):
            if dist(pos, P[k]) >= p['lookahead_m']:
                j = k
                break
        lx, ly = P[j]
        self.lookahead = (lx, ly)
        dx, dy = lx - x, ly - y
        xl = math.cos(th) * dx + math.sin(th) * dy
        yl = -math.sin(th) * dx + math.cos(th) * dy
        herr = math.atan2(yl, xl)

        # vigia de progresso
        remaining = piece.remaining[self.idx]
        if remaining < self.best_d - 0.01 or abs(herr) < self.best_h - math.radians(3):
            self.best_d = min(self.best_d, remaining)
            self.best_h = min(self.best_h, abs(herr))
            self.t_best = t
        elif t - self.t_best > p['stall_s']:
            self.active = False
            return 0.0, 0.0, f'aborted: sem progresso por {p["stall_s"]:.0f}s'

        if self.state == 'FOLLOW' and abs(herr) > math.radians(p['realign_deg']):
            self.state = 'ALIGN'
        if self.state == 'ALIGN':
            if abs(herr) <= math.radians(p['align_tol_deg']):
                self.state = 'FOLLOW'
            else:
                w = max(-p['w_max'], min(p['w_max'], p['k_align'] * herr))
                if abs(w) < p['w_align_min']:
                    w = math.copysign(p['w_align_min'], herr)
                return 0.0, w, 'running'

        # Pure Pursuit
        l2 = xl * xl + yl * yl
        k = 2.0 * yl / l2 if l2 > 1e-9 else 0.0
        v = min(p['v_max'], max(p['v_min'], p['slow_k'] * (remaining + d_end) / 2.0))
        if abs(k) * v > p['w_max']:
            v = p['w_max'] / abs(k)
        return v, v * k, 'running'
