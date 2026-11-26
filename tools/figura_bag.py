"""Figuras para o relatório e para a apresentação, a partir de um bag (sem ROS).

    pip install mcap mcap-ros2-support matplotlib numpy pillow
    python tools/figura_bag.py data/<pasta_do_bag> --grupo g00
    python tools/figura_bag.py data/<pasta_do_bag> --grupo g00 --gif docs/img/demo.gif

Gera em docs/img/:
  <bag>_instantes.png   grelha + zona proibida + caminho + objetivo em 3 instantes
  <bag>_estado.png      estado do planeador e desvio lateral ao longo do tempo
  (opcional) GIF animado com a sequência completa
"""
import argparse
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import PillowWriter
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from mcap_ros2.reader import read_ros2_messages

matplotlib.use('Agg')     # só grava ficheiros, não abre janelas


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def ler(bag, grupo):
    mcap = next(bag.glob('*.mcap')) if bag.is_dir() else bag
    t = {k: f'/{grupo}/{k}' for k in ('grelha_local', 'zona_proibida', 'caminho_local',
                                      'estado', 'objetivo', 'rota_local')}
    grelhas, zonas, caminhos, estados, objetivos, rotas = {}, {}, {}, [], [], []
    for m in read_ros2_messages(str(mcap), topics=list(t.values())):
        msg = m.ros_msg
        if m.channel.topic == t['grelha_local']:
            grelhas[stamp(msg)] = msg
        elif m.channel.topic == t['zona_proibida']:
            zonas[stamp(msg)] = msg
        elif m.channel.topic == t['caminho_local']:
            caminhos[stamp(msg)] = (m.log_time_ns * 1e-9, msg)
        elif m.channel.topic == t['rota_local']:
            rotas.append((m.log_time_ns * 1e-9, [(q.pose.position.x, q.pose.position.y) for q in msg.poses]))
        elif m.channel.topic == t['objetivo']:
            objetivos.append((m.log_time_ns * 1e-9, (msg.pose.position.x, msg.pose.position.y)))
        else:
            estados.append((m.log_time_ns * 1e-9, msg.data))
    # um "instante" = caminho + grelha + zona proibida com o mesmo header.stamp. O tempo do
    # instante é o da gravação do caminho (log_time), o mesmo relógio do estado, do objetivo e
    # da rota: ao reproduzir um bag antigo, o header.stamp é o do sensor (outro dia).
    instantes = [(caminhos[s][0], grelhas[s], zonas.get(s), caminhos[s][1])
                 for s in sorted(caminhos) if s in grelhas]
    instantes.sort(key=lambda x: x[0])
    return instantes, estados, objetivos, rotas


def desenhar(ax, grelha, zona, caminho, titulo, objetivo=None, rota=None):
    info = grelha.info
    w, h, r = info.width, info.height, info.resolution
    x0, y0 = info.origin.position.x, info.origin.position.y
    ext = [x0, x0 + w * r, y0, y0 + h * r]
    d = np.asarray(grelha.data, dtype=np.int16).reshape(h, w)
    img = np.where(d < 0, 1, np.where(d >= 50, 2, 0))       # 0 livre, 1 desconhecido, 2 ocupado
    ax.imshow(img, origin='lower', extent=ext, interpolation='nearest',
              cmap=ListedColormap(['#ffffff', '#d9dde3', '#1f2328']), vmin=0, vmax=2)
    if zona is not None:
        c = np.asarray(zona.data, dtype=np.int16).reshape(h, w)
        proib = np.ma.masked_where((c < 100) | (d >= 50), c)
        ax.imshow(proib, origin='lower', extent=ext, interpolation='nearest',
                  cmap=ListedColormap(['#f4a3a3']), alpha=0.8)
    if rota:
        ax.plot(*zip(*rota), color='#2563eb', lw=1.2, ls='--', label='rota global')
    xs = [p.pose.position.x for p in caminho.poses]
    ys = [p.pose.position.y for p in caminho.poses]
    ax.plot(xs, ys, color='#4a7c00', lw=2.5)
    ax.add_patch(Rectangle((-0.6, -0.75), 3.5 + 0.6, 1.5, fc='#91d400', ec='#4a7c00'))
    if objetivo is not None:
        ax.plot(*objetivo, marker='X', ms=11, color='#b91c1c', mec='white', mew=1.2, zorder=5)
        ax.annotate('objetivo', objetivo, xytext=(0, 9), textcoords='offset points',
                    ha='center', fontsize=8.5, color='#b91c1c')
    ax.set_title(titulo, fontsize=10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')


def estado_em(estados, t):
    anteriores = [e for (te, e) in estados if te <= t + 0.05]
    return anteriores[-1] if anteriores else ''


def rota_em(rotas, t):
    anteriores = [r for (tr, r) in rotas if tr <= t + 0.05]
    return anteriores[-1] if anteriores else None


def objetivo_em(objetivos, t, omissao):
    """Último objetivo publicado em /gNN/objetivo até t; senão o dos parâmetros."""
    anteriores = [o for (to, o) in objetivos if to <= t + 0.05]
    return anteriores[-1] if anteriores else omissao


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('--grupo', default='g00')
    ap.add_argument('--saida', type=Path, default=Path('docs/img'))
    ap.add_argument('--gif', type=Path, help='ficheiro GIF a criar (opcional)')
    ap.add_argument('--objetivo', type=float, nargs=2, default=[25.0, 0.0], metavar=('X', 'Y'),
                    help='objetivo local (m) se o bag não tiver /gNN/objetivo; '
                         'o mesmo de objetivo_x/objetivo_y no params.yaml')
    a = ap.parse_args()
    a.saida.mkdir(parents=True, exist_ok=True)

    instantes, estados, objetivos, rotas = ler(a.bag, a.grupo)
    if not instantes:
        raise SystemExit(f'sem /{a.grupo}/caminho_local + /{a.grupo}/grelha_local no bag')
    t0 = instantes[0][0]
    nome = a.bag.name

    # 1. três instantes: o primeiro LIVRE, o maior DESVIO e um BLOQUEADO (se houver)
    def desvio(e):
        return float(e.split()[1]) if e.startswith('DESVIO') else -1.0
    rot = [(s, g, c, p, estado_em(estados, s)) for s, g, c, p in instantes]
    escolha = [next((x for x in rot if x[4] == 'LIVRE'), rot[0]),
               max(rot, key=lambda x: desvio(x[4])),
               next((x for x in rot if x[4] == 'BLOQUEADO'), rot[-1])]
    fig, axs = plt.subplots(len(escolha), 1, figsize=(9, 3.2 * len(escolha)))
    for ax, (s, g, c, p, e) in zip(np.atleast_1d(axs), escolha):
        desenhar(ax, g, c, p, f't = {s - t0:5.1f} s   {e}',
                 objetivo_em(objetivos, s, tuple(a.objetivo)), rota_em(rotas, s))
    fig.tight_layout()
    fig.savefig(a.saida / f'{nome}_instantes.png', dpi=150)

    # 2. estado e desvio ao longo do tempo
    te = np.array([t for t, _ in estados]) - t0
    cod = {'LIVRE': 0, 'DESVIO': 1, 'BLOQUEADO': 2}
    ce = np.array([cod[e.split()[0]] for _, e in estados])
    dv = np.array([max(desvio(e), 0.0) for _, e in estados])
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 4.5), sharex=True)
    a1.step(te, ce, where='post', color='#1f2328')
    a1.set_yticks([0, 1, 2], ['LIVRE', 'DESVIO', 'BLOQUEADO'])
    a1.grid(True, alpha=0.3)
    a2.plot(te, dv, color='#4a7c00')
    a2.set_ylabel('desvio lateral (m)')
    a2.set_xlabel('tempo (s)')
    a2.grid(True, alpha=0.3)
    pct = {k: 100.0 * np.mean(ce == v) for k, v in cod.items()}
    a1.set_title('  '.join(f'{k}: {v:.0f}%' for k, v in pct.items()), fontsize=10)
    fig.tight_layout()
    fig.savefig(a.saida / f'{nome}_estado.png', dpi=150)
    print(f'{len(instantes)} instantes; estados: ' +
          ', '.join(f'{k} {v:.0f}%' for k, v in pct.items()))

    # 3. GIF (um em cada 2 instantes, para não ficar pesado)
    if a.gif:
        a.gif.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 4))
        escritor = PillowWriter(fps=5)
        with escritor.saving(fig, str(a.gif), dpi=80):
            for s, g, c, p, e in rot[::2]:
                ax.clear()
                desenhar(ax, g, c, p, f't = {s - t0:5.1f} s   {e}',
                         objetivo_em(objetivos, s, tuple(a.objetivo)), rota_em(rotas, s))
                escritor.grab_frame()
        print(f'GIF: {a.gif}')


if __name__ == '__main__':
    main()
