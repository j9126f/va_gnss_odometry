"""Testes do A* e da grelha (correm sem ROS: python3 -m pytest src/va_planeamento)."""
import numpy as np

from va_planeamento.astar import ParametrosPlaneador, astar, planear
from va_planeamento.grid import LIVRE, OCUPADO, LocalGrid


def grelha_livre():
    return LocalGrid(0.2, -2.0, 30.0, -6.0, 6.0, valor_inicial=LIVRE)


def test_caminho_reto_sem_obstaculos():
    r = planear(grelha_livre(), (0.0, 0.0), (20.0, 0.0))
    assert r.estado == 'LIVRE'
    assert abs(r.caminho_xy[-1][0] - 20.0) < 0.3


def test_desvia_de_um_carro_estacionado():
    g = grelha_livre()
    g.marcar_poligono([(10, -1.9), (14.5, -1.9), (14.5, 0.2), (10, 0.2)], OCUPADO)
    r = planear(g, (0.0, 0.0), (20.0, 0.0))
    assert r.estado == 'DESVIO'
    ys = [y for x, y in r.caminho_xy if 10 <= x <= 14.5]
    assert min(ys) > 0.2 + 1.0 - 0.2      # passa à esquerda, com a margem da inflação


def test_bloqueado_quando_a_estrada_esta_fechada():
    g = grelha_livre()
    g.marcar_poligono([(10, -6), (11, -6), (11, 6), (10, 6)], OCUPADO)
    assert planear(g, (0.0, 0.0), (20.0, 0.0)).estado == 'BLOQUEADO'


def test_corredor_impede_contornar_por_fora():
    g = grelha_livre()
    g.marcar_poligono([(10, -3), (11, -3), (11, 3), (10, 3)], OCUPADO)   # há espaço por fora
    assert planear(g, (0.0, 0.0), (20.0, 0.0)).estado == 'DESVIO'
    p = ParametrosPlaneador(corredor_y_min=-3.5, corredor_y_max=3.5)
    assert planear(g, (0.0, 0.0), (20.0, 0.0), p).estado == 'BLOQUEADO'


def test_heuristica_inflacionada_expande_menos():
    g = grelha_livre()
    g.marcar_poligono([(10, -3), (12, -3), (12, 3), (10, 3)], OCUPADO)
    otimo = planear(g, (0.0, 0.0), (20.0, 0.0), ParametrosPlaneador(peso_heuristica=1.0))
    rapido = planear(g, (0.0, 0.0), (20.0, 0.0), ParametrosPlaneador(peso_heuristica=3.0))
    assert rapido.expandidos < otimo.expandidos


def test_astar_em_grelha_minima():
    proibido = np.zeros((3, 3), dtype=bool)
    proibido[1, 1] = True
    caminho, _ = astar(proibido, (0, 1), (2, 1))
    assert caminho[0] == (0, 1) and caminho[-1] == (2, 1)
    assert (1, 1) not in caminho
