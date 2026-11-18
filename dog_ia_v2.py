"""
DOG-IA V2.1 — Simulador Urbano de Cão-Guia Autônomo
=====================================================
Melhorias desta versão:
  • Piso tátil direcional como sensor real: robô detecta se está sobre a
    faixa amarela e recebe bônus de alinhamento + mensagem de feedback.
  • Semáforo respeitado com lógica clara: o robô aguarda na borda do
    meio-fio, não ultrapassa a linha de retenção, e só atravessa quando
    PEDESTRE + tempo suficiente.
  • Anti-bug de rotação: waypoint look-ahead, raio de chegada ajustado,
    limite de giro por frame e detecção de oscilação → teleporte de
    saída (kick) ao invés de ficar girando.
  • Obstáculos reposicionados para não bloquear a rota inicial.
  • Semáforo começa em PEDESTRE para facilitar a primeira travessia.

Uso normal  : python dog_ia_v2.py
Benchmark   : python dog_ia_v2.py --benchmark 20 --seed 1
"""

import pygame
import math
import random
import heapq
import sys
import time
import threading
import numpy as np
from collections import deque

# ──────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────
BENCHMARK_MODE = False
BENCHMARK_RUNS = 0
RNG_SEED = 42
_args = sys.argv[1:]
if "--benchmark" in _args:
    BENCHMARK_MODE = True
    idx = _args.index("--benchmark")
    BENCHMARK_RUNS = int(_args[idx + 1]) if idx + 1 < len(_args) else 20
if "--seed" in _args:
    idx = _args.index("--seed")
    RNG_SEED = int(_args[idx + 1]) if idx + 1 < len(_args) else 42
random.seed(RNG_SEED)
np.random.seed(RNG_SEED)

# ──────────────────────────────────────────────────────────────
# CONSTANTES DE LAYOUT
# ──────────────────────────────────────────────────────────────
LARGURA, ALTURA     = 1050, 680
FPS                 = 60
AREA_MAPA_LARGURA   = LARGURA - 260

RUA_Y      = 250
RUA_ALTURA = 170
FAIXA_X    = 580        # início X da faixa de pedestres
FAIXA_LARG = 95         # largura da faixa

# O robô precisa chegar aqui antes de cruzar (Y = borda superior da rua - margem)
BORDA_ESPERA_Y   = RUA_Y - 22   # ponto de parada antes de cruzar
CENTRO_FAIXA_X   = FAIXA_X + FAIXA_LARG // 2

# Física
GUIA            = 42
ACEL            = 0.08
DESACEL         = 0.12
DIST_SEGUIMENTO = 70

# A*
CELL = 10
COLS = AREA_MAPA_LARGURA // CELL
ROWS = ALTURA // CELL
_SQRT2 = math.sqrt(2)

# ──────────────────────────────────────────────────────────────
# PALETA
# ──────────────────────────────────────────────────────────────
BRANCO          = (245, 245, 245)
PRETO           = (20,  22,  28)
ASFALTO         = (42,  45,  52)
CALCADA_CLARA   = (130, 134, 142)
MEIO_FIO        = (175, 180, 188)
PISO_TATIL_COR  = (235, 190,  20)
GRAMA           = (46,  115,  65)
CORPO_ROBO      = (30,  136, 229)
DETALHE_ROBO    = (15,   76, 129)
CABECA_ROBO     = (230, 236, 245)
ARNES_GUIA      = (255, 215,   0)
VERDE_LIVRE     = (46,  204, 113)
LARANJA_ALERTA  = (255, 140,   0)
VERMELHO_PERIGO = (225,  55,  55)
ROXO_SUSPENSO   = (155,  89, 182)
AMARELO_SEM     = (255, 220,   0)
COR_VEICULO_1   = (200,  60,  60)
COR_VEICULO_2   = (60,   60, 200)
COR_INFRATOR    = (255,  80, 180)
TATIL_GLOW      = (255, 230,  80)   # halo quando robô está sobre o piso tátil

# ──────────────────────────────────────────────────────────────
# PISO TÁTIL — definido globalmente para uso em sensor e grade
# ──────────────────────────────────────────────────────────────
# Cada entrada: (rect, orientacao)  orientacao = ângulo preferido em radianos
#   0.0  = direita  |  math.pi/2 = baixo  |  -math.pi/2 = cima
PISO_TATIL_FAIXAS = [
    (pygame.Rect(40,  120, 595,  14),  0.0),           # calçada superior → direita
    (pygame.Rect(621, 120,  14, 130),  math.pi / 2),   # descida até borda da rua
    (pygame.Rect(621, RUA_Y + RUA_ALTURA, 14, 130), math.pi / 2),  # subida após rua
    (pygame.Rect(150, 536, 485,  14),  0.0),           # calçada inferior → direita
]
PISO_TATIL_RECTS = [r for r, _ in PISO_TATIL_FAIXAS]

# ──────────────────────────────────────────────────────────────
# DESTINOS
# ──────────────────────────────────────────────────────────────
DESTINOS = {
    pygame.K_1: {"nome": "Mercado",  "pos": [628, 543]},
    pygame.K_2: {"nome": "Farmácia", "pos": [200, 580]},
    pygame.K_3: {"nome": "Padaria",  "pos": [720, 580]},
    pygame.K_4: {"nome": "Casa",     "pos": [100, 580]},
}

# ──────────────────────────────────────────────────────────────
# UTILIDADES
# ──────────────────────────────────────────────────────────────
def rotacionar_ponto(cx, cy, px, py, ang):
    s, c = math.sin(ang), math.cos(ang)
    dx, dy = px - cx, py - cy
    return (int(cx + dx * c - dy * s), int(cy + dx * s + dy * c))


def angulo_norm(a):
    """Normaliza ângulo para [-π, π]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def ponto_sobre_tatil(x, y):
    """Retorna (True, ang_preferido) se (x,y) está sobre alguma faixa tátil."""
    pt = pygame.Rect(int(x) - 2, int(y) - 2, 4, 4)
    for rect, ang in PISO_TATIL_FAIXAS:
        if rect.colliderect(pt):
            return True, ang
    return False, None


def gerar_beep(freq=440, dur_ms=80, vol=0.4, pan=0.0):
    taxa = 44100
    n = int(taxa * dur_ms / 1000)
    t = np.linspace(0, dur_ms / 1000, n, endpoint=False)
    onda = (np.sin(2 * np.pi * freq * t) * 32767 * vol).astype(np.int16)
    l_g = min(1.0, 1.0 - pan)
    r_g = min(1.0, 1.0 + pan)
    return np.column_stack([(onda * l_g).astype(np.int16),
                             (onda * r_g).astype(np.int16)])

_sons_cache: dict = {}

def tocar_alerta(freq, pan, dur_ms=80):
    if BENCHMARK_MODE:
        return
    chave = (freq, round(pan, 1), dur_ms)
    if chave not in _sons_cache:
        _sons_cache[chave] = pygame.sndarray.make_sound(gerar_beep(freq, dur_ms, 0.35, pan))
    ch = pygame.mixer.find_channel(True)
    if ch:
        ch.play(_sons_cache[chave])

# ──────────────────────────────────────────────────────────────
# SEMÁFORO
# ──────────────────────────────────────────────────────────────
class Semaforo:
    FASES   = ["VERDE", "AMARELO", "TODOS_VERM_1", "PEDESTRE", "TODOS_VERM_2"]
    DURACAO = {"VERDE": 8.0, "AMARELO": 2.5,
               "TODOS_VERM_1": 1.5, "PEDESTRE": 7.0, "TODOS_VERM_2": 1.5}

    def __init__(self, fase_inicial="PEDESTRE"):
        # começa em PEDESTRE para o robô poder atravessar logo de início
        self.fase_idx   = self.FASES.index(fase_inicial)
        self.tempo_fase = 0.0

    @property
    def fase(self):
        return self.FASES[self.fase_idx]

    @property
    def tempo_restante(self):
        return self.DURACAO[self.fase] - self.tempo_fase

    def pedestre_pode_cruzar(self):
        return self.fase == "PEDESTRE"

    def veiculo_pode_avançar(self):
        return self.fase in ("VERDE", "AMARELO")

    def atualizar(self, dt):
        self.tempo_fase += dt
        if self.tempo_fase >= self.DURACAO[self.fase]:
            self.tempo_fase -= self.DURACAO[self.fase]
            self.fase_idx = (self.fase_idx + 1) % len(self.FASES)

    def desenhar(self, tela, fonte):
        bx, by = FAIXA_X + FAIXA_LARG + 12, RUA_Y - 72
        pygame.draw.rect(tela, PRETO, (bx, by, 26, 68), border_radius=5)
        mapa = {
            "VERDE":        (VERMELHO_PERIGO, AMARELO_SEM, VERDE_LIVRE),
            "AMARELO":      (VERMELHO_PERIGO, AMARELO_SEM, (40,40,40)),
            "TODOS_VERM_1": (VERMELHO_PERIGO, (40,40,40),  (40,40,40)),
            "PEDESTRE":     ((40,40,40),       (40,40,40),  VERDE_LIVRE),
            "TODOS_VERM_2": (VERMELHO_PERIGO, (40,40,40),  (40,40,40)),
        }
        for i, cor in enumerate(mapa[self.fase]):
            pygame.draw.circle(tela, cor, (bx + 13, by + 11 + i * 22), 8)
        tela.blit(fonte.render(f"{self.tempo_restante:.1f}s", True, BRANCO), (bx - 2, by + 70))

# ──────────────────────────────────────────────────────────────
# VEÍCULOS
# ──────────────────────────────────────────────────────────────
class Veiculo:
    LARG, ALT = 38, 20

    def __init__(self, x, y, direcao, vel_max, infrator=False):
        self.x, self.y    = float(x), float(y)
        self.dir          = direcao
        self.vel          = vel_max * 0.5
        self.vel_max      = vel_max
        self.infrator     = infrator
        self.cor          = COR_INFRATOR if infrator else (COR_VEICULO_1 if direcao > 0 else COR_VEICULO_2)

    @property
    def rect(self):
        return pygame.Rect(int(self.x - self.LARG // 2),
                           int(self.y - self.ALT  // 2),
                           self.LARG, self.ALT)

    def atualizar(self, dt, semaforo, outros):
        lr_dir = FAIXA_X - 10
        lr_esq = FAIXA_X + FAIXA_LARG + 10
        frear = False
        if not self.infrator and not semaforo.veiculo_pode_avançar():
            if self.dir > 0 and self.x < lr_dir and (lr_dir - self.x) < 90:
                frear = True
            if self.dir < 0 and self.x > lr_esq and (self.x - lr_esq) < 90:
                frear = True
        for v in outros:
            if v is self or v.dir != self.dir:
                continue
            if 0 < (v.x - self.x) * self.dir < DIST_SEGUIMENTO:
                frear = True; break
        self.vel = max(0.0, self.vel - DESACEL * self.vel_max) if frear \
              else min(self.vel_max, self.vel + ACEL * self.vel_max)
        self.x += self.dir * self.vel * dt * FPS
        if self.dir > 0 and self.x >  AREA_MAPA_LARGURA + 60: self.x = -60.0
        if self.dir < 0 and self.x < -60: self.x = float(AREA_MAPA_LARGURA + 60)

    def swept_rect(self, dt=1.0):
        dx = self.dir * self.vel * dt * FPS
        x1 = min(self.x, self.x + dx) - self.LARG // 2
        x2 = max(self.x, self.x + dx) + self.LARG // 2
        return pygame.Rect(int(x1), int(self.y - self.ALT // 2 - 6),
                           int(x2 - x1), self.ALT + 12)

    def desenhar(self, tela, fonte):
        pygame.draw.rect(tela, self.cor,   self.rect, border_radius=5)
        pygame.draw.rect(tela, BRANCO,     self.rect, 1, border_radius=5)
        if self.infrator:
            lbl = fonte.render("!", True, BRANCO)
            tela.blit(lbl, (self.rect.centerx - 3, self.rect.centery - 6))


def criar_veiculos():
    vs = []
    for i in range(3):
        vs.append(Veiculo(-60 - i * 180, RUA_Y + 42,                +1, 2.2))
    for i in range(3):
        vs.append(Veiculo(AREA_MAPA_LARGURA + 60 + i * 180, RUA_Y + RUA_ALTURA - 42, -1, 2.0))
    vs.append(Veiculo(-400,                      RUA_Y + 42,                +1, 2.8, infrator=True))
    vs.append(Veiculo(AREA_MAPA_LARGURA + 400,  RUA_Y + RUA_ALTURA - 42,  -1, 2.8, infrator=True))
    return vs

# ──────────────────────────────────────────────────────────────
# GRADE A*
# ──────────────────────────────────────────────────────────────
def construir_grade(obstaculos):
    grade  = np.zeros((ROWS, COLS), dtype=np.uint8)
    custos = np.ones( (ROWS, COLS), dtype=np.float32)
    inflacao = {"solo": 28, "suspenso": 36, "parede": 20}
    for obs in obstaculos:
        inf  = inflacao.get(obs["tipo"], 20)
        ri   = obs["rect"].inflate(inf * 2, inf * 2)
        cx0  = max(0, ri.left   // CELL)
        cx1  = min(COLS, ri.right  // CELL + 1)
        ry0  = max(0, ri.top    // CELL)
        ry1  = min(ROWS, ri.bottom // CELL + 1)
        grade[ry0:ry1, cx0:cx1] = 1
    # Custo reduzido sobre piso tátil
    for rect in PISO_TATIL_RECTS:
        c0 = max(0, rect.left   // CELL)
        c1 = min(COLS, rect.right  // CELL + 1)
        r0 = max(0, rect.top    // CELL)
        r1 = min(ROWS, rect.bottom // CELL + 1)
        m  = grade[r0:r1, c0:c1] == 0
        custos[r0:r1, c0:c1][m] = 0.4
    return grade, custos


def astar(grade, custos, inicio, fim):
    er, ec = fim
    sr, sc = inicio
    if grade[sr, sc] or grade[er, ec]:
        return []

    def h(r, c):
        dr, dc = abs(r - er), abs(c - ec)
        return max(dr, dc) + (_SQRT2 - 1) * min(dr, dc)

    g_arr   = np.full((ROWS, COLS), np.inf, dtype=np.float32)
    g_arr[sr, sc] = 0.0
    pais    = {}
    cnt     = 0
    aberto  = [(h(sr, sc), 0, sr, sc)]

    while aberto:
        f, _, r, c = heapq.heappop(aberto)
        if f > g_arr[r, c] + h(r, c) + 1e-5:
            continue
        if r == er and c == ec:
            path = []
            cur  = (r, c)
            while cur in pais:
                path.append(cur)
                cur = pais[cur]
            path.append((sr, sc))
            path.reverse()
            return [(col * CELL + CELL // 2, row * CELL + CELL // 2)
                    for row, col in path]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0: continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < ROWS and 0 <= nc < COLS and not grade[nr, nc]:
                    passo = _SQRT2 if dr and dc else 1.0
                    ng = g_arr[r, c] + passo * float(custos[nr, nc])
                    if ng < g_arr[nr, nc]:
                        g_arr[nr, nc] = ng
                        pais[(nr, nc)] = (r, c)
                        cnt += 1
                        heapq.heappush(aberto, (ng + h(nr, nc), cnt, nr, nc))
    return []

# ──────────────────────────────────────────────────────────────
# TUTOR
# ──────────────────────────────────────────────────────────────
class Tutor:
    def __init__(self, x, y):
        self.x, self.y = float(x), float(y)

    def atualizar(self, rx, ry, rang, dt):
        ax = rx - math.cos(rang) * GUIA
        ay = ry - math.sin(rang) * GUIA
        dx, dy = ax - self.x, ay - self.y
        dist = math.hypot(dx, dy)
        if dist > 1:
            f = min(1.0, dist / GUIA)
            self.x += dx * f * 0.18
            self.y += dy * f * 0.18

    def desenhar(self, tela, rx, ry):
        ux, uy = int(self.x), int(self.y)
        pygame.draw.line(tela, ARNES_GUIA, (int(rx), int(ry)), (ux, uy), 4)
        pygame.draw.circle(tela, (235, 235, 240), (ux, uy), 13)
        pygame.draw.circle(tela, PRETO,           (ux, uy), 13, 2)
        pygame.draw.arc(tela, PRETO, (ux - 9, uy - 4,  8, 6), 0, math.pi, 2)
        pygame.draw.arc(tela, PRETO, (ux + 1, uy - 4,  8, 6), 0, math.pi, 2)

# ──────────────────────────────────────────────────────────────
# ROBÔ CÃO-GUIA
# ──────────────────────────────────────────────────────────────
class RoboCaoGuia:
    def __init__(self, x, y):
        self.x, self.y    = float(x), float(y)
        self.angulo       = 0.0
        self.raio         = 20
        self.vel_base     = 2.5
        self.vel_atual    = 2.5

        # LiDAR
        self.alcance_lidar   = 165
        self.num_raios       = 24
        self.fov             = math.radians(150)
        self.leituras_lidar  = []
        self._med_buf        = np.full((3, self.num_raios),
                                       float(self.alcance_lidar), dtype=np.float32)
        self._med_ptr        = 0

        # Estado da máquina
        self.estado           = "NAVEGANDO"
        self.status_assistivo = "Iniciando rota pelo piso tátil."
        self.cor_alerta       = VERDE_LIVRE
        self.passo_animacao   = 0.0

        # Piso tátil — feedback
        self.sobre_tatil      = False
        self.ang_tatil        = 0.0
        self._tatil_frames    = 0   # frames consecutivos sobre o tátil

        # Semáforo — estado interno
        self._na_borda        = False   # robô está parado esperando
        self._aguardou_sinal  = False

        # A*
        self.caminho_global   = []
        self.waypoint_idx     = 0

        # Anti-bug de rotação
        self._ang_hist        = deque(maxlen=20)  # histórico de ângulos
        self._pos_hist        = deque(maxlen=60)  # histórico de posições (frames)
        self.tempo_preso      = 0.0
        self.pos_ant_replano  = (x, y)
        self._kick_cooldown   = 0.0  # segundos até permitir próximo kick

        # Métricas
        self.distancia_total   = 0.0
        self.alertas_vermelhos = 0
        self.freios_emergencia = 0
        self.viagens_completas = 0
        self.tempo_viagem      = 0.0

    # ── LiDAR ────────────────────────────────────────────────
    def ler_lidar(self, obstaculos):
        self.leituras_lidar = []
        ang0  = self.angulo - self.fov / 2
        passo = self.fov / (self.num_raios - 1)
        paredes = [
            pygame.Rect(0, 0, AREA_MAPA_LARGURA, 15),
            pygame.Rect(0, 0, 15, ALTURA),
            pygame.Rect(0, ALTURA - 15, AREA_MAPA_LARGURA, 15),
            pygame.Rect(AREA_MAPA_LARGURA - 15, 0, 15, ALTURA),
        ]
        todos = obstaculos + [{"rect": p, "tipo": "parede"} for p in paredes]
        raw   = np.empty(self.num_raios, dtype=np.float32)
        tipos = [None] * self.num_raios
        ox, oy = self.x, self.y
        for i in range(self.num_raios):
            ar = ang0 + i * passo
            fx = ox + math.cos(ar) * self.alcance_lidar
            fy = oy + math.sin(ar) * self.alcance_lidar
            dm, th = self.alcance_lidar, None
            for obs in todos:
                clip = obs["rect"].clipline(ox, oy, fx, fy)
                if clip:
                    hx, hy = clip[0]
                    d = math.hypot(hx - ox, hy - oy)
                    if d < dm:
                        dm, th = d, obs["tipo"]
            dm += np.random.normal(0, 1.5)
            if random.random() < 0.02:
                dm = self.alcance_lidar
            raw[i]   = max(0.0, min(self.alcance_lidar, dm))
            tipos[i] = th
        self._med_buf[self._med_ptr] = raw
        self._med_ptr = (self._med_ptr + 1) % 3
        fil = np.median(self._med_buf, axis=0)
        for i in range(self.num_raios):
            self.leituras_lidar.append({
                "angulo":    ang0 + i * passo,
                "distancia": float(fil[i]),
                "tipo":      tipos[i],
            })

    # ── Sensor de piso tátil ─────────────────────────────────
    def atualizar_sensor_tatil(self):
        """
        Detecta se o robô está sobre uma faixa tátil amarela e
        extrai o ângulo de alinhamento preferido daquela faixa.
        """
        sobre, ang = ponto_sobre_tatil(self.x, self.y)
        self.sobre_tatil = sobre
        if sobre:
            self.ang_tatil = ang
            self._tatil_frames += 1
        else:
            self._tatil_frames = 0

    # ── Swept-volume ─────────────────────────────────────────
    def verificar_swept(self, veiculos):
        zona = pygame.Rect(int(self.x) - 24, int(self.y) - 24, 48, 48)
        return any(v.swept_rect(1.0).colliderect(zona) for v in veiculos)

    # ── Barreira de meio-fio ──────────────────────────────────
    def aplicar_barreira(self, semaforo):
        na_faixa = FAIXA_X - 5 <= self.x <= FAIXA_X + FAIXA_LARG + 5
        # borda superior da rua
        if RUA_Y - self.raio - 2 < self.y < RUA_Y + 8:
            if not na_faixa or not semaforo.pedestre_pode_cruzar():
                self.y = float(RUA_Y - self.raio)
                # empurra para dentro da faixa se muito fora
                if not na_faixa:
                    self.x = float(max(FAIXA_X + 5, min(FAIXA_X + FAIXA_LARG - 5, self.x)))
        # borda inferior da rua
        if RUA_Y + RUA_ALTURA - 8 < self.y < RUA_Y + RUA_ALTURA + self.raio + 2:
            if not na_faixa or not semaforo.pedestre_pode_cruzar():
                self.y = float(RUA_Y + RUA_ALTURA + self.raio)
                if not na_faixa:
                    self.x = float(max(FAIXA_X + 5, min(FAIXA_X + FAIXA_LARG - 5, self.x)))

    # ── Anti-oscilação (kick) ─────────────────────────────────
    def _detectar_oscilacao(self):
        """
        Detecta se o robô está girando sem avançar (bug de rotação).
        Retorna True se deve aplicar kick.
        """
        if len(self._ang_hist) < 20:
            return False
        angs = list(self._ang_hist)
        # Variância alta + deslocamento baixo = oscilando
        variancia_ang = np.var(angs)
        if len(self._pos_hist) >= 30:
            px0, py0 = self._pos_hist[0]
            px1, py1 = self._pos_hist[-1]
            desloc = math.hypot(px1 - px0, py1 - py0)
            if variancia_ang > 0.08 and desloc < 12:
                return True
        return False

    def _aplicar_kick(self):
        """
        Tira o robô de um mínimo local: empurra na direção oposta ao
        ângulo médio atual por uma distância fixa.
        """
        ang_med = float(np.mean(list(self._ang_hist)))
        # empurra perpendicularmente para o lado com mais espaço
        if self.leituras_lidar:
            terco = self.num_raios // 3
            ml = min(r["distancia"] for r in self.leituras_lidar[:terco])
            mr = min(r["distancia"] for r in self.leituras_lidar[2*terco:])
            lado = 1 if mr > ml else -1
            ang_kick = ang_med + lado * math.pi / 2
        else:
            ang_kick = ang_med + math.pi / 2
        self.x += math.cos(ang_kick) * 30
        self.y += math.sin(ang_kick) * 30
        # Mantém dentro dos limites
        self.x = max(20, min(AREA_MAPA_LARGURA - 20, self.x))
        self.y = max(20, min(ALTURA - 20, self.y))
        self._ang_hist.clear()
        self._pos_hist.clear()
        self.tempo_preso    = 0.0
        self._kick_cooldown = 2.0  # s — evita kicks consecutivos

    # ── Navegação principal ───────────────────────────────────
    def decidir_navegacao(self, dt, semaforo, veiculos, alvo_final):
        self.tempo_viagem += dt
        self._kick_cooldown = max(0.0, self._kick_cooldown - dt)

        # Registra histórico para anti-oscilação
        self._ang_hist.append(self.angulo)
        self._pos_hist.append((self.x, self.y))

        # 1. Piso tátil — atualiza sensor
        self.atualizar_sensor_tatil()

        # 2. Swept-volume → FREIO_EMERGENCIA (só fora da rua ou se veículo muito perto)
        na_rua = RUA_Y < self.y < RUA_Y + RUA_ALTURA
        if self.verificar_swept(veiculos) and not na_rua:
            if self.estado != "FREIO_EMERGENCIA":
                self.freios_emergencia += 1
            self.estado          = "FREIO_EMERGENCIA"
            self.cor_alerta      = VERMELHO_PERIGO
            self.status_assistivo = "FREIO EMERGÊNCIA: Veículo próximo!"
            self.alertas_vermelhos += 1
            return

        # Sai do freio de emergência se ameaça passou
        if self.estado == "FREIO_EMERGENCIA" and not self.verificar_swept(veiculos):
            self.estado = "NAVEGANDO"

        # 3. Waypoint A*
        alvo_wp = None
        if self.caminho_global and self.waypoint_idx < len(self.caminho_global):
            # Look-ahead: pula waypoints já muito próximos
            while (self.waypoint_idx < len(self.caminho_global) - 1 and
                   math.hypot(self.caminho_global[self.waypoint_idx][0] - self.x,
                               self.caminho_global[self.waypoint_idx][1] - self.y) < 20):
                self.waypoint_idx += 1
            alvo_wp = self.caminho_global[self.waypoint_idx]
        else:
            # Sem caminho: vai direto ao alvo final
            alvo_wp = tuple(alvo_final)

        # 4. Lógica de semáforo — precisa cruzar a rua?
        destino_abaixo = alvo_final[1] > RUA_Y + RUA_ALTURA
        precisa_cruzar = (self.y < RUA_Y - 10) and destino_abaixo

        t_min = (RUA_ALTURA + 24 + GUIA) / (self.vel_base * FPS) * 1.35 + 0.4

        if precisa_cruzar:
            dist_borda = math.hypot(CENTRO_FAIXA_X - self.x, BORDA_ESPERA_Y - self.y)

            if not semaforo.pedestre_pode_cruzar():
                self.estado          = "AGUARDANDO_SINAL"
                self.cor_alerta      = AMARELO_SEM
                self.status_assistivo = (
                    f"Sinal vermelho. Aguardando... {semaforo.tempo_restante:.1f}s"
                )
                # Vai até o ponto de espera na borda da faixa
                if dist_borda > 18:
                    alvo_wp = (CENTRO_FAIXA_X, BORDA_ESPERA_Y)
                else:
                    # Já na borda — para e espera
                    self._na_borda = True
                    return
            else:
                # Sinal verde para pedestre
                if semaforo.tempo_restante < t_min:
                    self.estado = "AGUARDANDO_SINAL"
                    self.status_assistivo = "Sinal insuficiente. Aguardando próximo ciclo."
                    self.cor_alerta = AMARELO_SEM
                    return
                self._na_borda = False
                self.estado    = "ATRAVESSANDO"

        # 5. Anti-oscilação
        if self._kick_cooldown <= 0 and self._detectar_oscilacao():
            self._aplicar_kick()
            self.estado           = "REPLANEJANDO"
            self.status_assistivo = "Desvio de mínimo local..."
            self.cor_alerta       = LARANJA_ALERTA
            return

        # Detecção de preso pelo deslocamento
        dist_desde = math.hypot(self.x - self.pos_ant_replano[0],
                                self.y - self.pos_ant_replano[1])
        if dist_desde < 6:
            self.tempo_preso += dt
        else:
            self.tempo_preso      = 0.0
            self.pos_ant_replano  = (self.x, self.y)

        if self.tempo_preso > 3.0 and self.estado == "NAVEGANDO" and self._kick_cooldown <= 0:
            self._aplicar_kick()
            self.estado           = "REPLANEJANDO"
            self.status_assistivo = "Recalculando rota..."
            self.cor_alerta       = LARANJA_ALERTA
            return

        if alvo_wp is None:
            return

        # 6. Evasão reativa (LiDAR)
        t3 = self.num_raios // 3
        esq  = self.leituras_lidar[:t3]
        cen  = self.leituras_lidar[t3: 2 * t3]
        dire = self.leituras_lidar[2 * t3:]
        ml   = min(r["distancia"] for r in esq)
        mc   = min(r["distancia"] for r in cen)
        mr   = min(r["distancia"] for r in dire)
        susp = any(r["tipo"] == "suspenso" and r["distancia"] < 115 for r in cen)

        if mc < 80:
            self.cor_alerta = VERMELHO_PERIGO
            self.alertas_vermelhos += 1
            msg = "ALERTA CABEÇA: Galho/Toldo!" if susp else "OBSTÁCULO: Contornando..."
            self.status_assistivo = msg
            self.angulo += -0.07 if ml > mr else 0.07
            vel = self.vel_base * 0.35
        elif ml < 60:
            self.cor_alerta       = LARANJA_ALERTA
            self.status_assistivo = "Obstáculo à esquerda. Deslocando direita."
            self.angulo          += 0.04
            vel = self.vel_base * 0.85
        elif mr < 60:
            self.cor_alerta       = LARANJA_ALERTA
            self.status_assistivo = "Obstáculo à direita. Deslocando esquerda."
            self.angulo          -= 0.04
            vel = self.vel_base * 0.85
        else:
            # Caminho livre — direcionar ao waypoint
            dx_w = alvo_wp[0] - self.x
            dy_w = alvo_wp[1] - self.y
            ang_alvo = math.atan2(dy_w, dx_w)

            # Bônus de alinhamento ao piso tátil
            if self.sobre_tatil:
                # Mistura o ângulo do tátil com o ângulo do waypoint
                diff_tatil = angulo_norm(self.ang_tatil - ang_alvo)
                ang_alvo   = ang_alvo + diff_tatil * 0.35
                self.cor_alerta = TATIL_GLOW
                self.status_assistivo = "Seguindo piso tátil direcional."
            else:
                self.cor_alerta = VERDE_LIVRE
                if self.estado not in ("AGUARDANDO_SINAL", "ATRAVESSANDO",
                                       "FREIO_EMERGENCIA"):
                    self.estado = "NAVEGANDO"
                    self.status_assistivo = "Caminho livre. Seguindo rota A*."

            diff = angulo_norm(ang_alvo - self.angulo)
            # Limita giro máximo por frame → evita rotação excessiva
            MAX_GIRO = 0.055
            self.angulo += max(-MAX_GIRO, min(MAX_GIRO, diff))
            vel = self.vel_base

        # Reduz velocidade se arnês tenso
        tensao = 0.0  # calculada externamente e aplicada em _run_once
        vel *= max(0.5, 1.0 - tensao / GUIA)

        self.vel_atual = vel
        dmx = math.cos(self.angulo) * vel
        dmy = math.sin(self.angulo) * vel
        self.distancia_total += math.hypot(dmx, dmy)
        self.x += dmx
        self.y += dmy
        self.passo_animacao += vel * 0.25

    # ── Desenho ──────────────────────────────────────────────
    def desenhar(self, tela):
        # Halo de tátil
        if self.sobre_tatil:
            pygame.draw.circle(tela, (*TATIL_GLOW, 0),
                               (int(self.x), int(self.y)), 28, 3)

        # LiDAR
        for r in self.leituras_lidar:
            fx = self.x + math.cos(r["angulo"]) * r["distancia"]
            fy = self.y + math.sin(r["angulo"]) * r["distancia"]
            cor = VERDE_LIVRE if r["distancia"] >= self.alcance_lidar - 5 else VERMELHO_PERIGO
            pygame.draw.line(tela, cor, (int(self.x), int(self.y)), (int(fx), int(fy)), 1)
            if r["tipo"]:
                pygame.draw.circle(tela, BRANCO, (int(fx), int(fy)), 3)

        # Patas
        b = math.sin(self.passo_animacao) * 5
        for ox, oy in [(10+b, -14), (10-b, 14), (-12-b, -14), (-12+b, 14)]:
            px, py = rotacionar_ponto(self.x, self.y, self.x + ox, self.y + oy, self.angulo)
            pygame.draw.circle(tela, DETALHE_ROBO, (px, py), 5)

        # Corpo
        pts = [rotacionar_ponto(self.x, self.y, self.x + dx, self.y + dy, self.angulo)
               for dx, dy in [(16,-11),(16,11),(-18,11),(-18,-11)]]
        pygame.draw.polygon(tela, CORPO_ROBO, pts)
        pygame.draw.polygon(tela, BRANCO, pts, 2)

        # Arnês dourado no dorso
        asp = [rotacionar_ponto(self.x, self.y, self.x + dx, self.y + dy, self.angulo)
               for dx, dy in [(-5,-11),(-5,11),(10,11),(10,-11)]]
        pygame.draw.polygon(tela, ARNES_GUIA, asp)

        # Cabeça
        cx, cy = rotacionar_ponto(self.x, self.y, self.x + 21, self.y, self.angulo)
        pygame.draw.circle(tela, CABECA_ROBO,  (cx, cy), 9)
        pygame.draw.circle(tela, DETALHE_ROBO, (cx, cy), 9, 2)
        # Câmera
        camx, camy = rotacionar_ponto(self.x, self.y, self.x + 27, self.y, self.angulo)
        pygame.draw.circle(tela, VERMELHO_PERIGO, (camx, camy), 3)
        # Orelhas
        for dy in (-10, 10):
            ex, ey = rotacionar_ponto(self.x, self.y, self.x + 19, self.y + dy, self.angulo)
            pygame.draw.circle(tela, DETALHE_ROBO, (ex, ey), 4)
        # Torre LiDAR
        lx, ly = rotacionar_ponto(self.x, self.y, self.x + 3, self.y, self.angulo)
        pygame.draw.circle(tela, LARANJA_ALERTA, (lx, ly), 5)
        pygame.draw.circle(tela, BRANCO,         (lx, ly), 5, 1)

# ──────────────────────────────────────────────────────────────
# CENÁRIO
# ──────────────────────────────────────────────────────────────
def criar_cenario():
    # Obstáculos reposicionados para não bloquear a rota direta do robô
    return [
        {"rect": pygame.Rect(160,  60, 45, 70), "tipo": "solo",     "nome": "Banca/Hidrante"},
        {"rect": pygame.Rect(420, 170, 30, 30), "tipo": "solo",     "nome": "Poste de Luz"},
        {"rect": pygame.Rect(300, 460, 38, 38), "tipo": "solo",     "nome": "Lixeira Urbana"},
        {"rect": pygame.Rect(130,  65, 80, 55), "tipo": "suspenso", "nome": "Copa de Árvore"},
        {"rect": pygame.Rect(480, 490, 90, 60), "tipo": "suspenso", "nome": "Toldo de Loja"},
    ]

# ──────────────────────────────────────────────────────────────
# DESENHO DO AMBIENTE
# ──────────────────────────────────────────────────────────────
def desenhar_ambiente_urbano(tela):
    tela.fill(CALCADA_CLARA)
    pygame.draw.rect(tela, GRAMA, (0, 0, AREA_MAPA_LARGURA, 30))
    pygame.draw.rect(tela, GRAMA, (0, ALTURA - 30, AREA_MAPA_LARGURA, 30))
    pygame.draw.rect(tela, ASFALTO, (0, RUA_Y, AREA_MAPA_LARGURA, RUA_ALTURA))
    pygame.draw.line(tela, MEIO_FIO, (0, RUA_Y),              (AREA_MAPA_LARGURA, RUA_Y), 4)
    pygame.draw.line(tela, MEIO_FIO, (0, RUA_Y + RUA_ALTURA), (AREA_MAPA_LARGURA, RUA_Y + RUA_ALTURA), 4)
    # Tracejado central
    for x in range(15, AREA_MAPA_LARGURA - 20, 45):
        if not (FAIXA_X - 5 <= x <= FAIXA_X + FAIXA_LARG + 5):
            pygame.draw.line(tela, BRANCO,
                             (x, RUA_Y + RUA_ALTURA // 2),
                             (x + 22, RUA_Y + RUA_ALTURA // 2), 3)
    # Faixa de pedestres
    for y in range(RUA_Y + 12, RUA_Y + RUA_ALTURA - 12, 22):
        pygame.draw.rect(tela, BRANCO, (FAIXA_X, y, FAIXA_LARG, 12))

    # Piso tátil — desenhado com textura pontilhada para melhor visualização
    for rect, _ in PISO_TATIL_FAIXAS:
        pygame.draw.rect(tela, PISO_TATIL_COR, rect)
        # Pontos guia sobre a faixa tátil (simula textura real)
        for px in range(rect.left + 4, rect.right - 2, 10):
            for py in range(rect.top + 3, rect.bottom - 1, 6):
                pygame.draw.circle(tela, (200, 155, 10), (px, py), 2)

    # Linha de retenção (onde veículos param)
    pygame.draw.line(tela, (220, 60, 60),
                     (FAIXA_X - 10, RUA_Y),
                     (FAIXA_X - 10, RUA_Y - 20), 3)
    pygame.draw.line(tela, (220, 60, 60),
                     (FAIXA_X + FAIXA_LARG + 10, RUA_Y + RUA_ALTURA),
                     (FAIXA_X + FAIXA_LARG + 10, RUA_Y + RUA_ALTURA + 20), 3)

# ──────────────────────────────────────────────────────────────
# PAINEL LATERAL
# ──────────────────────────────────────────────────────────────
def desenhar_painel(tela, fontes, robo, semaforo, destino_nome, modo_autonomo):
    ft, fx, fp = fontes
    px = AREA_MAPA_LARGURA
    pygame.draw.rect(tela, PRETO, (px, 0, LARGURA - px, ALTURA))
    pygame.draw.line(tela, BRANCO, (px, 0), (px, ALTURA), 2)

    y = 12
    tela.blit(ft.render("TELEMETRIA DOG-IA V2", True, CORPO_ROBO), (px + 10, y)); y += 30
    tela.blit(fx.render(f"Modo: {'AUTÔNOMO (IA)' if modo_autonomo else 'MANUAL'}", True, BRANCO), (px+10, y)); y += 20

    # Estado colorido
    cor_est = {
        "NAVEGANDO": VERDE_LIVRE, "AGUARDANDO_SINAL": AMARELO_SEM,
        "ATRAVESSANDO": (100,220,255), "FREIO_EMERGENCIA": VERMELHO_PERIGO,
        "DESTINO_ALCANCADO": VERDE_LIVRE, "REPLANEJANDO": LARANJA_ALERTA,
    }.get(robo.estado, BRANCO)
    tela.blit(fx.render(f"Estado: {robo.estado}", True, cor_est), (px+10, y)); y += 20
    tela.blit(fx.render(f"Destino: {destino_nome}", True, BRANCO), (px+10, y)); y += 20

    # Piso tátil
    tatil_cor = TATIL_GLOW if robo.sobre_tatil else (90, 90, 90)
    tatil_txt = "✔ Sobre piso tátil!" if robo.sobre_tatil else "✘ Fora do piso tátil"
    tela.blit(fx.render(tatil_txt, True, tatil_cor), (px+10, y)); y += 20

    # Semáforo
    cores_fase = {"VERDE": VERDE_LIVRE, "AMARELO": AMARELO_SEM,
                  "TODOS_VERM_1": VERMELHO_PERIGO, "TODOS_VERM_2": VERMELHO_PERIGO,
                  "PEDESTRE": (100, 220, 255)}
    cf = cores_fase.get(semaforo.fase, BRANCO)
    tela.blit(fx.render(f"Semáforo: {semaforo.fase}", True, cf), (px+10, y)); y += 20
    tela.blit(fx.render(f"  Tempo: {semaforo.tempo_restante:.1f}s",  True, BRANCO), (px+10, y)); y += 20

    if robo.leituras_lidar:
        md = min(r["distancia"] for r in robo.leituras_lidar)
        tela.blit(fx.render(f"LiDAR min: {md:.0f} px", True, BRANCO), (px+10, y))
    y += 20

    # Caixa de alerta
    y += 4
    pygame.draw.rect(tela, robo.cor_alerta, (px+10, y, 238, 78), border_radius=8)
    palavras, linha, ya = robo.status_assistivo.split(" "), "", y + 8
    for p in palavras:
        if len(linha + p) < 24:
            linha += p + " "
        else:
            tela.blit(fx.render(linha, True, PRETO), (px+18, ya)); ya += 20; linha = p + " "
    tela.blit(fx.render(linha, True, PRETO), (px+18, ya))
    y += 88

    # Métricas
    tela.blit(ft.render("MÉTRICAS", True, CORPO_ROBO), (px+10, y)); y += 24
    for m in [f"Distância: {robo.distancia_total:.0f} px",
              f"Tempo: {robo.tempo_viagem:.1f} s",
              f"Alertas ⚠: {robo.alertas_vermelhos}",
              f"Freios 🛑: {robo.freios_emergencia}",
              f"Viagens ✔: {robo.viagens_completas}"]:
        tela.blit(fx.render(m, True, BRANCO), (px+10, y)); y += 19

    # Legenda / controles
    y += 6
    tela.blit(ft.render("LEGENDA / CONTROLES", True, CORPO_ROBO), (px+10, y)); y += 22
    for txt, cor in [
        ("▬ Amarelo c/pts: Piso Tátil", PISO_TATIL_COR),
        ("■ Cinza: Obstáculo Solo",     (150,155,165)),
        ("■ Roxo: Suspenso (Alto)",     ROXO_SUSPENSO),
        ("▶ Verm/Azul: Carros",         COR_VEICULO_1),
        ("▶ Rosa !: Infrator",          COR_INFRATOR),
        ("1-4: Escolher destino",       BRANCO),
        ("Clique Esq: Novo destino",    BRANCO),
        ("Clique Dir: Obstáculo",       BRANCO),
        ("ESPAÇO: Manual / IA",         BRANCO),
        ("R: Reiniciar",                BRANCO),
    ]:
        tela.blit(fp.render(txt, True, cor), (px+10, y)); y += 17

# ──────────────────────────────────────────────────────────────
# LOOP PRINCIPAL
# ──────────────────────────────────────────────────────────────
def _run_once(seed_local=None, headless=False):
    if seed_local is not None:
        random.seed(seed_local)
        np.random.seed(seed_local)

    if not headless:
        pygame.init()
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
        tela   = pygame.display.set_mode((LARGURA, ALTURA))
        pygame.display.set_caption("DOG-IA V2.1 — Simulador Urbano Autônomo")
        clock  = pygame.time.Clock()
        fontes = (
            pygame.font.SysFont("arial", 17, bold=True),
            pygame.font.SysFont("arial", 13),
            pygame.font.SysFont("arial", 11),
        )
    else:
        tela = clock = None

    robo       = RoboCaoGuia(95, 127)
    tutor      = Tutor(95 - GUIA, 127)
    semaforo   = Semaforo(fase_inicial="PEDESTRE")  # começa em verde para pedestre
    obstaculos = criar_cenario()
    veiculos   = criar_veiculos()

    destino_info  = DESTINOS[pygame.K_1]
    destino_nome  = destino_info["nome"]
    alvo          = list(destino_info["pos"])

    grade, custos = construir_grade(obstaculos)
    _lock         = threading.Lock()

    def replanejar(force_grade=False):
        nonlocal grade, custos
        if force_grade:
            grade, custos = construir_grade(obstaculos)
        _g, _c = grade, custos
        _alvo  = list(alvo)
        def _job():
            sr = max(0, min(ROWS-1, int(robo.y) // CELL))
            sc = max(0, min(COLS-1, int(robo.x) // CELL))
            er = max(0, min(ROWS-1, _alvo[1]    // CELL))
            ec = max(0, min(COLS-1, _alvo[0]    // CELL))
            cam = astar(_g, _c, (sr, sc), (er, ec))
            with _lock:
                robo.caminho_global  = cam
                robo.waypoint_idx    = 0
                robo.estado          = "NAVEGANDO"
        threading.Thread(target=_job, daemon=True).start()

    replanejar()

    modo_autonomo = True
    rodando       = True
    ultimo_alerta = 0.0
    t_inicio      = time.time()

    while rodando:
        if headless:
            dt = 1.0 / FPS
        else:
            dt = min(clock.tick(FPS) / 1000.0, 0.05)

        # ── Eventos ──────────────────────────────────────────
        if not headless:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    rodando = False
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    if mx < AREA_MAPA_LARGURA:
                        if ev.button == 1:
                            alvo[:] = [mx, my]
                            replanejar(force_grade=False)
                        elif ev.button == 3:
                            obstaculos.append({
                                "rect": pygame.Rect(mx-22, my-22, 44, 44),
                                "tipo": random.choice(["solo","suspenso"]),
                                "nome": "Obstáculo",
                            })
                            replanejar(force_grade=True)
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_SPACE:
                        modo_autonomo = not modo_autonomo
                    elif ev.key == pygame.K_r:
                        robo       = RoboCaoGuia(95, 127)
                        tutor      = Tutor(95 - GUIA, 127)
                        obstaculos = criar_cenario()
                        veiculos   = criar_veiculos()
                        semaforo   = Semaforo(fase_inicial="PEDESTRE")
                        replanejar(force_grade=True)
                    elif ev.key in DESTINOS:
                        destino_info = DESTINOS[ev.key]
                        destino_nome = destino_info["nome"]
                        alvo[:]      = destino_info["pos"]
                        replanejar(force_grade=False)

            teclas = pygame.key.get_pressed()
            if not modo_autonomo:
                if teclas[pygame.K_LEFT]:  robo.angulo -= 0.05
                if teclas[pygame.K_RIGHT]: robo.angulo += 0.05
                if teclas[pygame.K_UP]:
                    robo.x += math.cos(robo.angulo) * 3
                    robo.y += math.sin(robo.angulo) * 3
                    robo.passo_animacao += 0.5

        # ── Atualização ───────────────────────────────────────
        semaforo.atualizar(dt)
        for v in veiculos:
            v.atualizar(dt, semaforo, veiculos)

        obs_lidar = obstaculos + [{"rect": v.rect, "tipo": "solo"} for v in veiculos]
        robo.ler_lidar(obs_lidar)

        if modo_autonomo:
            dist_alvo = math.hypot(alvo[0] - robo.x, alvo[1] - robo.y)
            if dist_alvo > 25:
                # Repassa tensão do arnês ao robô
                tensao = max(0.0, math.hypot(robo.x - tutor.x, robo.y - tutor.y) - GUIA)
                if tensao > 10:
                    robo.vel_base = max(1.0, robo.vel_base * 0.93)
                else:
                    robo.vel_base = min(2.5, robo.vel_base * 1.02)

                if robo.estado == "REPLANEJANDO":
                    replanejar(force_grade=False)
                else:
                    robo.decidir_navegacao(dt, semaforo, veiculos, alvo)
                    robo.aplicar_barreira(semaforo)
            else:
                robo.estado           = "DESTINO_ALCANCADO"
                robo.status_assistivo = f"DESTINO: {destino_nome} alcançado!"
                robo.cor_alerta       = VERDE_LIVRE
                robo.viagens_completas += 1
                if headless:
                    rodando = False

        tutor.atualizar(robo.x, robo.y, robo.angulo, dt)

        # Alertas sonoros estéreo
        agora = time.time()
        if not headless and agora - ultimo_alerta > 0.4:
            if robo.cor_alerta == VERMELHO_PERIGO:
                t3  = robo.num_raios // 3
                ml  = min(r["distancia"] for r in robo.leituras_lidar[:t3])
                mr  = min(r["distancia"] for r in robo.leituras_lidar[2*t3:])
                tocar_alerta(880, -0.7 if ml < mr else 0.7, 90)
                ultimo_alerta = agora
            elif robo.cor_alerta == LARANJA_ALERTA:
                tocar_alerta(540, 0.0, 70)
                ultimo_alerta = agora
            elif robo.cor_alerta == TATIL_GLOW:
                # Bip suave confirmando piso tátil
                tocar_alerta(330, 0.0, 50)
                ultimo_alerta = agora + 0.6  # menos frequente

        # ── Renderização ──────────────────────────────────────
        if not headless:
            desenhar_ambiente_urbano(tela)

            # Rota A*
            with _lock:
                cam_copy = list(robo.caminho_global)
                wp_idx   = robo.waypoint_idx
            if cam_copy and wp_idx < len(cam_copy):
                pts = cam_copy[wp_idx:]
                if len(pts) > 1:
                    pygame.draw.lines(tela, (80, 160, 255), False, pts, 1)

            # Ponto de destino
            pygame.draw.circle(tela, VERDE_LIVRE, alvo, 20, 3)
            pygame.draw.circle(tela, VERDE_LIVRE, alvo, 7)

            # Ponto de espera na borda
            pygame.draw.circle(tela, AMARELO_SEM,
                               (CENTRO_FAIXA_X, BORDA_ESPERA_Y), 6, 2)

            # Obstáculos
            for obs in obstaculos:
                r = obs["rect"]
                if obs["tipo"] == "suspenso":
                    pygame.draw.rect(tela, (90,95,102), r.move(5,5), border_radius=12)
                    pygame.draw.rect(tela, ROXO_SUSPENSO, r, border_radius=12)
                    pygame.draw.rect(tela, BRANCO, r, 2, border_radius=12)
                    lbl = fontes[2].render("ALTO", True, BRANCO)
                    tela.blit(lbl, (r.centerx - lbl.get_width()//2, r.centery - 7))
                else:
                    pygame.draw.rect(tela, (70,74,82), r, border_radius=5)
                    pygame.draw.rect(tela, BRANCO, r, 2, border_radius=5)
                    lbl = fontes[2].render("SOLO", True, BRANCO)
                    tela.blit(lbl, (r.centerx - lbl.get_width()//2, r.centery - 7))

            for v in veiculos:
                v.desenhar(tela, fontes[2])
            semaforo.desenhar(tela, fontes[2])
            tutor.desenhar(tela, robo.x, robo.y)
            robo.desenhar(tela)
            desenhar_painel(tela, fontes, robo, semaforo, destino_nome, modo_autonomo)
            pygame.display.flip()

        if headless and (time.time() - t_inicio) > 120:
            rodando = False

    return {
        "sucesso":   robo.estado == "DESTINO_ALCANCADO",
        "tempo":     robo.tempo_viagem,
        "distancia": robo.distancia_total,
        "alertas":   robo.alertas_vermelhos,
        "freios":    robo.freios_emergencia,
        "viagens":   robo.viagens_completas,
    }

# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────
def main():
    if BENCHMARK_MODE:
        print(f"\n{'='*55}")
        print(f"  DOG-IA V2.1 — BENCHMARK  {BENCHMARK_RUNS} corridas  seed={RNG_SEED}")
        print(f"{'='*55}")
        pygame.init()
        resultados = []
        for i in range(BENCHMARK_RUNS):
            s = RNG_SEED + i
            r = _run_once(seed_local=s, headless=True)
            resultados.append(r)
            print(f"  Run {i+1:>3}  seed={s}  {'✔' if r['sucesso'] else '✘'}  "
                  f"t={r['tempo']:>6.1f}s  dist={r['distancia']:>6.0f}px  "
                  f"alert={r['alertas']}  freios={r['freios']}")
        n   = BENCHMARK_RUNS
        ok  = sum(1 for r in resultados if r["sucesso"])
        print(f"\n  Taxa de sucesso : {ok/n*100:.1f}%  ({ok}/{n})")
        print(f"  Tempo médio     : {sum(r['tempo']    for r in resultados)/n:.2f} s")
        print(f"  Freios emergên. : {sum(r['freios']   for r in resultados)}")
        print(f"  Alertas vermel. : {sum(r['alertas']  for r in resultados)}")
        print(f"{'='*55}\n")
        pygame.quit()
    else:
        pygame.init()
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
        _run_once()
        pygame.quit()


if __name__ == "__main__":
    main()
