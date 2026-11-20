"""Planeamento local com A* numa grelha de ocupação (sem ROS, só numpy).

Passos:
  1. obstáculos = células ocupadas (e, opcionalmente, as desconhecidas);
  2. inflação: as células a menos de `raio_inflacao` de um obstáculo ficam
     proibidas, para que o carro possa ser tratado como um ponto;
  3. A* com 8 vizinhos e heurística euclidiana.
"""
import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .grid import DESCONHECIDO

VIZINHOS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def inflacionar(obstaculo, raio):
    """Marca como proibidas as células a menos de `raio` células de um obstáculo."""
    ny, nx = obstaculo.shape
    proibido = obstaculo.copy()
    r = int(raio)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= raio * raio:
                # desloca a máscara de obstáculos (dx, dy) células
                proibido[max(dy, 0):ny + min(dy, 0), max(dx, 0):nx + min(dx, 0)] |= \
                    obstaculo[max(-dy, 0):ny + min(-dy, 0), max(-dx, 0):nx + min(-dx, 0)]
    return proibido


def astar(proibido, inicio, objetivo, peso_heuristica=1.0):
    """A* numa grelha. inicio e objetivo são células (ix, iy).

    Devolve (caminho, expandidos): caminho é a lista de células, ou None.
    """
    ny, nx = proibido.shape
    g = {inicio: 0.0}                 # custo desde o início
    pai = {inicio: None}              # para reconstruir o caminho
    aberto = [(0.0, inicio)]          # lista aberta: (f, célula), ordenada por f
    fechado = set()                   # células já expandidas

    while aberto:
        _, atual = heapq.heappop(aberto)
        if atual in fechado:
            continue
        fechado.add(atual)

        if atual == objetivo:         # chegou: reconstruir o caminho para trás
            caminho = []
            while atual is not None:
                caminho.append(atual)
                atual = pai[atual]
            return caminho[::-1], len(fechado)

        for dx, dy in VIZINHOS:
            viz = (atual[0] + dx, atual[1] + dy)
            if not (0 <= viz[0] < nx and 0 <= viz[1] < ny):
                continue
            if proibido[viz[1], viz[0]] or viz in fechado:
                continue
            novo_g = g[atual] + math.hypot(dx, dy)
            if novo_g < g.get(viz, math.inf):
                g[viz] = novo_g
                pai[viz] = atual
                h = math.hypot(objetivo[0] - viz[0], objetivo[1] - viz[1])
                heapq.heappush(aberto, (novo_g + peso_heuristica * h, viz))

    return None, len(fechado)


@dataclass
class ParametrosPlaneador:
    raio_inflacao: float = 1.0       # m: meia largura do carro + margem
    peso_heuristica: float = 1.0     # 1 = A* ótimo; > 1 = weighted A* (mais rápido)
    desconhecido_livre: bool = True  # as células -1 contam como livres?
    corredor_y_min: float = 0.0      # m: limites laterais da estrada, se conhecidos;
    corredor_y_max: float = 0.0      #    fora de [y_min, y_max] é obstáculo (iguais = desligado)
    limiar_desvio: float = 0.3       # m: desvio lateral acima do qual o estado é DESVIO


@dataclass
class ResultadoPlaneamento:
    estado: str                      # LIVRE, DESVIO ou BLOQUEADO
    caminho_xy: list = field(default_factory=list)
    desvio_max: float = 0.0
    expandidos: int = 0
    tempo_ms: float = 0.0
    proibido: np.ndarray = None


def planear(grelha, inicio_xy, objetivo_xy, p=ParametrosPlaneador()):
    """Planeia de inicio_xy para objetivo_xy (m, no referencial da grelha)."""
    t0 = time.perf_counter()

    # 1. obstáculos
    obst = grelha.data >= 50
    if not p.desconhecido_livre:
        obst |= grelha.data == DESCONHECIDO
    if p.corredor_y_max > p.corredor_y_min:
        _, y = grelha.para_metros(0, np.arange(grelha.ny))
        obst |= ((y < p.corredor_y_min) | (y > p.corredor_y_max))[:, None]

    # 2. inflação
    proibido = inflacionar(obst, p.raio_inflacao / grelha.res)

    # 3. A* da célula do carro até à célula livre mais próxima do objetivo
    ix, iy, _ = grelha.para_celula(*inicio_xy)
    inicio = (int(ix), int(iy))
    proibido[inicio[1], inicio[0]] = False           # o carro está lá
    gx, gy, _ = grelha.para_celula(*objetivo_xy)
    livres = np.argwhere(~proibido)                  # linhas (iy, ix)
    k = np.argmin((livres[:, 1] - gx) ** 2 + (livres[:, 0] - gy) ** 2)
    objetivo = (int(livres[k, 1]), int(livres[k, 0]))

    caminho, expandidos = astar(proibido, inicio, objetivo, p.peso_heuristica)
    ms = (time.perf_counter() - t0) * 1000.0
    if caminho is None:
        return ResultadoPlaneamento('BLOQUEADO', [], 0.0, expandidos, ms, proibido)

    # 4. estado: desvio lateral máximo em relação à reta início -> objetivo
    xs, ys = grelha.para_metros(*np.array(caminho).T)
    (ax, ay), (bx, by) = inicio_xy, grelha.para_metros(*objetivo)
    comp = math.hypot(bx - ax, by - ay) or 1.0
    desvio = float(np.max(np.abs((bx - ax) * (ay - ys) - (ax - xs) * (by - ay)) / comp))
    estado = 'DESVIO' if desvio > p.limiar_desvio else 'LIVRE'
    return ResultadoPlaneamento(estado, list(zip(xs.tolist(), ys.tolist())),
                                desvio, expandidos, ms, proibido)
