"""Grelha de ocupação local no referencial do carro (base_link).

Biblioteca sem ROS (só numpy), partilhada por todos os grupos:
- o nó de perceção de cada grupo constrói uma LocalGrid e marca nela
  o que deteta (pontos, polígonos, raios livres);
- o planeador local (A*) lê a mesma grelha a partir da mensagem
  nav_msgs/OccupancyGrid.

Convenção de valores (a mesma do nav_msgs/OccupancyGrid):
    -1  desconhecido      0  livre      100  ocupado
Índices: data[iy, ix], com ix ao longo de x (para a frente do carro)
e iy ao longo de y (para a esquerda). A célula (0, 0) tem o canto
inferior esquerdo em (x_min, y_min).
"""
import numpy as np

DESCONHECIDO = -1
LIVRE = 0
OCUPADO = 100


class LocalGrid:

    def __init__(self, resolucao=0.2, x_min=-5.0, x_max=35.0,
                 y_min=-10.0, y_max=10.0, valor_inicial=DESCONHECIDO):
        self.res = float(resolucao)
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.nx = int(round((x_max - x_min) / self.res))
        self.ny = int(round((y_max - y_min) / self.res))
        self.data = np.full((self.ny, self.nx), valor_inicial, dtype=np.int8)

    # ------------------------------------------------------------ conversões
    @property
    def x_max(self):
        return self.x_min + self.nx * self.res

    @property
    def y_max(self):
        return self.y_min + self.ny * self.res

    def para_celula(self, x, y):
        """Coordenadas (m) -> índices (ix, iy) e máscara dos que caem na grelha."""
        ix = np.floor((np.asarray(x, dtype=float) - self.x_min) / self.res).astype(int)
        iy = np.floor((np.asarray(y, dtype=float) - self.y_min) / self.res).astype(int)
        dentro = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, dentro

    def para_metros(self, ix, iy):
        """Índices -> coordenadas (m) do centro da célula."""
        x = self.x_min + (np.asarray(ix) + 0.5) * self.res
        y = self.y_min + (np.asarray(iy) + 0.5) * self.res
        return x, y

    # --------------------------------------------------------------- marcação
    def marcar_pontos(self, x, y, valor=OCUPADO):
        """Marca as células que contêm os pontos (x, y), p. ex. impactos de um LiDAR."""
        ix, iy, ok = self.para_celula(x, y)
        self.data[iy[ok], ix[ok]] = valor

    def marcar_raios_livres(self, x0, y0, x, y):
        """Marca como livres as células atravessadas pelos raios (x0,y0)->(x,y).

        Útil para sensores de distância: entre o sensor e o impacto não há nada.
        Os pontos de impacto não são marcados (usar marcar_pontos a seguir).
        """
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        comp = np.hypot(x - x0, y - y0)
        n = int(np.ceil(comp.max() / (0.5 * self.res))) if comp.size else 0
        if n == 0:
            return
        s = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]  # (n, 1)
        xs = x0 + s * (x - x0)                                  # (n, raios)
        ys = y0 + s * (y - y0)
        ix, iy, ok = self.para_celula(xs.ravel(), ys.ravel())
        livre = self.data[iy[ok], ix[ok]] != OCUPADO
        self.data[iy[ok][livre], ix[ok][livre]] = LIVRE

    def marcar_poligono(self, vertices, valor=OCUPADO):
        """Marca o interior de um polígono [(x, y), ...] em base_link.

        Útil para câmaras (zona fora da faixa, caixa de um veículo projetada no
        chão) e para LiDAR 3D (contorno de um cluster).
        """
        v = np.asarray(vertices, dtype=float)
        ix0, iy0, _ = self.para_celula(v[:, 0].min(), v[:, 1].min())
        ix1, iy1, _ = self.para_celula(v[:, 0].max(), v[:, 1].max())
        ix0, iy0 = max(int(ix0), 0), max(int(iy0), 0)
        ix1, iy1 = min(int(ix1), self.nx - 1), min(int(iy1), self.ny - 1)
        if ix1 < ix0 or iy1 < iy0:
            return
        gx, gy = np.meshgrid(np.arange(ix0, ix1 + 1), np.arange(iy0, iy1 + 1))
        px, py = self.para_metros(gx, gy)
        dentro = np.zeros(px.shape, dtype=bool)
        j = len(v) - 1
        for i in range(len(v)):            # teste do número de cruzamentos
            xi, yi = v[i]
            xj, yj = v[j]
            cruza = ((yi > py) != (yj > py)) & \
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
            dentro ^= cruza
            j = i
        self.data[gy[dentro], gx[dentro]] = valor

    # ------------------------------------------------------------- mensagens
    def para_msg(self, header):
        """Converte para nav_msgs/OccupancyGrid (o import do ROS fica aqui dentro)."""
        from nav_msgs.msg import OccupancyGrid
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = self.res
        msg.info.width = self.nx
        msg.info.height = self.ny
        msg.info.origin.position.x = self.x_min
        msg.info.origin.position.y = self.y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = self.data.ravel().tolist()
        return msg

    @classmethod
    def de_msg(cls, msg):
        """Cria uma LocalGrid a partir de um nav_msgs/OccupancyGrid (sem rotação)."""
        g = cls.__new__(cls)
        g.res = float(msg.info.resolution)
        g.x_min = float(msg.info.origin.position.x)
        g.y_min = float(msg.info.origin.position.y)
        g.nx = int(msg.info.width)
        g.ny = int(msg.info.height)
        g.data = np.asarray(msg.data, dtype=np.int8).reshape(g.ny, g.nx)
        return g
