import math

import numpy as np
import pytest

from va_g07.rota import SeguidorRota, en_para_base_link, para_en, reduzir

LAT0, LON0 = 40.63, -8.66          # Aveiro
M_LAT = 1 / 111319.49              # graus de latitude por metro


def rota_para_norte(comprimento=100.0, passo=2.0):
    n = np.arange(0.0, comprimento + passo, passo)
    return np.column_stack([LAT0 + n * M_LAT, np.full_like(n, LON0)])


def test_para_en_ida_e_volta():
    en = para_en([[LAT0 + 100 * M_LAT, LON0]], LAT0, LON0)[0]
    assert en[0] == pytest.approx(0.0, abs=1e-6)
    assert en[1] == pytest.approx(100.0, abs=0.01)


@pytest.mark.parametrize('azimute, ponto_en, esperado_xy', [
    (0.0, (0.0, 10.0), (10.0, 0.0)),      # virado a Norte, ponto a Norte: à frente
    (0.0, (10.0, 0.0), (0.0, -10.0)),     # virado a Norte, ponto a Este: à direita
    (90.0, (10.0, 0.0), (10.0, 0.0)),     # virado a Este, ponto a Este: à frente
    (90.0, (0.0, 10.0), (0.0, 10.0)),     # virado a Este, ponto a Norte: à esquerda
])
def test_en_para_base_link(azimute, ponto_en, esperado_xy):
    xy = en_para_base_link(np.array([ponto_en]), azimute)[0]
    assert xy == pytest.approx(esperado_xy, abs=1e-9)


def test_objetivo_a_frente_na_rota():
    s = SeguidorRota(rota_para_norte(), distancia_objetivo=25.0)
    (ox, oy), troco, dist, chegou = s.atualizar(LAT0 + 10 * M_LAT, LON0, 0.0)
    assert ox == pytest.approx(25.0, abs=2.1)      # resolução dos waypoints (2 m)
    assert oy == pytest.approx(0.0, abs=0.01)
    assert dist == pytest.approx(0.0, abs=0.01)
    assert not chegou


def test_carro_desviado_para_a_direita_ve_a_rota_a_esquerda():
    s = SeguidorRota(rota_para_norte())
    leste = 3.0 / (111319.49 * math.cos(math.radians(LAT0)))
    (ox, oy), _, dist, _ = s.atualizar(LAT0 + 10 * M_LAT, LON0 + leste, 0.0)
    assert oy == pytest.approx(3.0, abs=0.05)
    assert dist == pytest.approx(3.0, abs=0.05)


def test_fim_da_rota_e_fora_da_rota():
    s = SeguidorRota(rota_para_norte(), distancia_objetivo=25.0, distancia_max_rota=30.0)
    *_, chegou = s.atualizar(LAT0 + 90 * M_LAT, LON0, 0.0)
    assert chegou
    assert s.atualizar(LAT0 + 500 * M_LAT, LON0, 0.0) is None


def test_reduzir_espaca_e_tira_saltos():
    pts = [(LAT0 + i * 0.5 * M_LAT, LON0) for i in range(41)]   # 20 m a cada 0,5 m
    pts.insert(20, (LAT0 + 500 * M_LAT, LON0))                  # salto isolado de ~490 m
    pts.insert(0, (0.0, 0.0))                                   # INS sem solução
    r = reduzir(pts, espacamento=2.0)
    n = para_en(r, LAT0, LON0)[:, 1]
    assert n.max() < 21 and n.min() >= 0
    assert np.all(np.diff(n) == pytest.approx(2.0, abs=0.01))


def test_objetivo_fica_dentro_da_grelha_numa_curva():
    # 10 m para Norte e depois 40 m para Oeste: virado a Norte, a rota sai da grelha pela esquerda
    k = 1 / (111319.49 * math.cos(math.radians(LAT0)))
    pts = [(LAT0 + n * M_LAT, LON0) for n in range(0, 11, 2)]
    pts += [(LAT0 + 10 * M_LAT, LON0 - w * k) for w in range(2, 41, 2)]
    s = SeguidorRota(np.array(pts), distancia_objetivo=25.0)
    (ox, oy), _, _, _ = s.atualizar(LAT0, LON0, 0.0, limites=(-5.0, 35.0, -10.0, 10.0))
    assert ox == pytest.approx(10.0, abs=0.01)
    assert 8.0 <= oy < 10.0
