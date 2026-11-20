"""Rota global em (lat, lon) e objetivo local no base_link. Sem ROS (testável com pytest).

A rota é uma lista de waypoints (lat, lon) em graus, num CSV com colunas `lat,lon`
(gravada com `rota_de_bag`, ou exportada de OSRM / Google Directions / GPX).
"""
import csv
import math

import numpy as np

RAIO_TERRA = 6378137.0          # m (WGS84, semieixo maior)


def ler_csv(caminho):
    """Lê um CSV com colunas lat,lon. Devolve um array (N, 2) em graus."""
    with open(caminho, newline='') as f:
        linhas = [(float(r['lat']), float(r['lon'])) for r in csv.DictReader(f)]
    if len(linhas) < 2:
        raise ValueError(f'{caminho}: a rota precisa de pelo menos 2 waypoints')
    return np.array(linhas)


def gravar_csv(caminho, latlon):
    with open(caminho, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['lat', 'lon'])
        for lat, lon in latlon:
            w.writerow([f'{lat:.8f}', f'{lon:.8f}'])


def para_en(latlon, lat0, lon0):
    """(lat, lon) em graus -> (Este, Norte) em metros à volta de (lat0, lon0).

    Projeção equirretangular local: erro < 1 cm a 1 km da origem, chega para o planeamento local.
    """
    latlon = np.atleast_2d(latlon)
    n = np.radians(latlon[:, 0] - lat0) * RAIO_TERRA
    e = np.radians(latlon[:, 1] - lon0) * RAIO_TERRA * math.cos(math.radians(lat0))
    return np.column_stack([e, n])


def en_para_base_link(en, azimute_graus):
    """(Este, Norte) relativos ao carro -> (x frente, y esquerda) no base_link.

    azimute: rumo do carro em graus, de Norte para Este (convenção NovAtel INSPVA).
    """
    psi = math.radians(azimute_graus)
    e, n = en[:, 0], en[:, 1]
    x = e * math.sin(psi) + n * math.cos(psi)
    y = -e * math.cos(psi) + n * math.sin(psi)
    return np.column_stack([x, y])


def reduzir(latlon, espacamento=2.0, salto_max=15.0):
    """Waypoints espaçados de ~espacamento m; descarta saltos > salto_max m (outliers do GNSS)."""
    latlon = np.asarray(latlon)
    latlon = latlon[(latlon[:, 0] != 0) | (latlon[:, 1] != 0)]
    if len(latlon) == 0:
        return latlon
    lat0, lon0 = latlon[0]
    en = para_en(latlon, lat0, lon0)
    escolhidos = [0]
    anterior = en[0]
    for i in range(1, len(en)):
        d = math.hypot(*(en[i] - anterior))
        if d > salto_max and i - escolhidos[-1] < 10:
            continue                       # salto isolado: ignora
        if d >= espacamento:
            escolhidos.append(i)
            anterior = en[i]
    return latlon[escolhidos]


class SeguidorRota:
    """Mantém o índice do waypoint mais próximo e calcula o objetivo a uma distância ao longo da rota."""

    def __init__(self, latlon, distancia_objetivo=25.0, distancia_max_rota=30.0, janela=200):
        self.latlon = np.asarray(latlon)
        self.distancia_objetivo = distancia_objetivo
        self.distancia_max_rota = distancia_max_rota
        self.janela = janela               # waypoints procurados à frente (rota com voltas repetidas)
        self.indice = None
        lat0, lon0 = self.latlon[0]
        self._en = para_en(self.latlon, lat0, lon0)
        seg = np.hypot(*np.diff(self._en, axis=0).T)
        self._acum = np.concatenate([[0.0], np.cumsum(seg)])   # distância ao longo da rota
        self._origem = (lat0, lon0)

    @property
    def comprimento(self):
        return float(self._acum[-1])

    def atualizar(self, lat, lon, azimute, limites=None):
        """Devolve (objetivo_xy, troco_xy, distancia_a_rota, chegou) no base_link, ou None se fora da rota.

        limites = (x_min, x_max, y_min, y_max) da grelha: o troço é cortado no primeiro ponto que sai
        dela, para o objetivo ficar dentro da grelha (numa curva apertada fica mais perto do carro).
        """
        carro = para_en([[lat, lon]], *self._origem)[0]
        if self.indice is None:
            ini, fim = 0, len(self._en)                       # 1.ª vez: procura em toda a rota
        else:
            ini, fim = max(0, self.indice - 10), min(len(self._en), self.indice + self.janela)
        d = np.hypot(*(self._en[ini:fim] - carro).T)
        k = int(np.argmin(d))
        if d[k] > self.distancia_max_rota:
            if self.indice is not None:                       # perdeu-se: volta a procurar em toda a rota
                self.indice = None
            return None
        self.indice = ini + k

        alvo = self._acum[self.indice] + self.distancia_objetivo
        j = int(np.searchsorted(self._acum, alvo))
        chegou = j >= len(self._en)
        j = min(j, len(self._en) - 1)
        rel = self._en[self.indice:j + 1] - carro
        troco = en_para_base_link(rel, azimute)
        if limites is not None:
            x0, x1, y0, y1 = limites
            fora = ~((troco[:, 0] >= x0) & (troco[:, 0] < x1) & (troco[:, 1] >= y0) & (troco[:, 1] < y1))
            if fora[1:].any():
                troco = troco[:1 + int(np.argmax(fora[1:]))]
                chegou = False
        return troco[-1], troco, float(d[k]), chegou
