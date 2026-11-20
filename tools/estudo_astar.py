"""Ex. 5 da aula 10: efeito dos parâmetros do A* nas mesmas grelhas (valores reprodutíveis).

O log do planeador sai no máximo 1 vez/s e a carrinha anda 4 m nesse tempo, por isso ler
"com a carrinha a ~15 m" à mão dá valores diferentes de cada vez. Aqui corre-se o mesmo
planear() do va_planeamento sobre as grelhas gravadas, com a carrinha entre --de e --ate m,
e mostra-se a mediana de nós expandidos, tempo e desvio, e a % de cada estado.

    docker compose --profile simulado up -d                  # (e, para o 2.º cenário:
    docker compose exec planeador bash                       #  ros2 param set /fake_scan cenario bloqueio)
    ros2 bag record -s mcap -o /data/g07_ex5 /g07/grelha_local      # ~30 s, Ctrl+C
    exit
    docker compose run --rm -v ./tools:/tools planeador python3 /tools/estudo_astar.py /data/g07_ex5
"""
import argparse
import statistics as st

import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from va_planeamento.astar import ParametrosPlaneador, planear
from va_planeamento.grid import LocalGrid

OBJETIVO = (25.0, 0.0)


def ler_grelhas(bag, topico):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topico]))
    while r.has_next():
        _, dados, _ = r.read_next()
        yield LocalGrid.de_msg(deserialize_message(dados, OccupancyGrid))


def classificar(g):
    """Distância à carrinha (traseira, m) e se o camião do cenário "bloqueio" está na grelha."""
    iy, ix = np.nonzero(g.data >= 50)
    x, y = g.para_metros(ix, iy)
    carrinha = (y > -3.2) & (y < -0.9) & (x > 0)
    camiao = ((y > 1.1) & (y < 4.5) & (x > 0)).sum() > 20
    return (x[carrinha].min() if carrinha.any() else None), camiao


def correr(grelhas, repeticoes=3, **kw):
    p = ParametrosPlaneador(**kw)
    exp, ms, desv, est = [], [], [], []
    for g in grelhas:
        for _ in range(repeticoes):
            r = planear(g, (0.0, 0.0), OBJETIVO, p)
            ms.append(r.tempo_ms)
        exp.append(r.expandidos)
        desv.append(r.desvio_max if r.estado == 'DESVIO' else 0.0)
        est.append(r.estado)
    estados = ', '.join(f'{e} {100 * est.count(e) // len(est)}%'
                        for e in ('LIVRE', 'DESVIO', 'BLOQUEADO') if e in est)
    return f'{int(st.median(exp)):>10d} | {st.median(ms):>9.1f} | {st.median(desv):>9.1f} | {estados}'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--topico', default='/g07/grelha_local')
    ap.add_argument('--de', type=float, default=13.0, help='distância mínima da carrinha (m)')
    ap.add_argument('--ate', type=float, default=17.0, help='distância máxima da carrinha (m)')
    a = ap.parse_args()

    cenarios = {'desvio': [], 'bloqueio': []}
    for g in ler_grelhas(a.bag, a.topico):
        xo, camiao = classificar(g)
        if xo is not None and a.de <= xo <= a.ate:
            cenarios['bloqueio' if camiao else 'desvio'].append(g)

    cab = f'{"parâmetro":<20} | {"valor":>6} | expandidos | tempo (ms) | desvio (m) | estados'
    for nome, gs in cenarios.items():
        if not gs:
            continue
        print(f'\nCenário "{nome}": {len(gs)} grelhas com a carrinha a {a.de:g}-{a.ate:g} m\n{cab}')
        if nome == 'desvio':
            for w in (1.0, 3.0, 10.0):
                print(f'{"peso_heuristica":<20} | {w:>6g} | {correr(gs, peso_heuristica=w)}')
            for ri in (0.5, 1.0, 1.5):
                print(f'{"raio_inflacao":<20} | {ri:>6g} | {correr(gs, raio_inflacao=ri)}')
            for dl in (True, False):
                print(f'{"desconhecido_livre":<20} | {str(dl).lower():>6} | {correr(gs, desconhecido_livre=dl)}')
        else:
            print(f'{"(omissão)":<20} | {"":>6} | {correr(gs)}')
            print(f'{"desconhecido_livre":<20} | {"false":>6} | {correr(gs, desconhecido_livre=False)}')
            print(f'{"corredor_y":<20} | {"-3.5/5":>6} | {correr(gs, corredor_y_min=-3.5, corredor_y_max=5.0)}')


if __name__ == '__main__':
    main()
