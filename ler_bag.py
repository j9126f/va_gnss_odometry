"""Leitura off-line de um bag MCAP do GNSS/INS, sem ROS (Linux, Windows ou macOS).

    ./ler_bag.sh data/<pasta_do_bag>              (Linux: instala tudo sozinho)
ou, à mão:
    pip install mcap mcap-ros2-support matplotlib
    python ler_bag.py data/<pasta_do_bag>

Mostra o resumo do bag, desenha trajetória e atitude e grava, na pasta do bag,
um CSV (para Matlab/Excel) e um PNG.
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

NAN = float('nan')
TOPICO_INS = '/novatel/oem7/inspva'
TOPICO_FIX = '/novatel/oem7/fix'


def resumo(mcap_file):
    with open(mcap_file, 'rb') as f:
        s = make_reader(f).get_summary()
    est = s.statistics
    dur = (est.message_end_time - est.message_start_time) * 1e-9
    print(f'Ficheiro: {mcap_file.name}  |  duração: {dur:.1f} s')
    for cid, n in sorted(est.channel_message_counts.items(), key=lambda x: s.channels[x[0]].topic):
        print(f'  {s.channels[cid].topic:<40} {n:>8} msgs')
    return {s.channels[c].topic for c in est.channel_message_counts}


def ler(mcap_file, topico, extrair):
    return [extrair(m.ros_msg, m.log_time_ns * 1e-9)
            for m in read_ros2_messages(str(mcap_file), topics=[topico])]


def g(msg, nome):
    return getattr(msg, nome, NAN)


def extrair_ins(msg, t):
    return dict(t=t, lat=g(msg, 'latitude'), lon=g(msg, 'longitude'), alt=g(msg, 'height'),
                roll=g(msg, 'roll'), pitch=g(msg, 'pitch'), azimuth=g(msg, 'azimuth'),
                status=str(getattr(msg, 'status', '')))


def extrair_fix(msg, t):
    return dict(t=t, lat=msg.latitude, lon=msg.longitude, alt=msg.altitude,
                roll=NAN, pitch=NAN, azimuth=NAN, status=str(msg.status.status))


def main():
    if len(sys.argv) < 2:
        sys.exit('uso: python ler_bag.py data/<pasta_do_bag>')
    bag = Path(sys.argv[1])
    mcap_file = next(bag.glob('*.mcap')) if bag.is_dir() else bag
    pasta = mcap_file.parent

    topicos = resumo(mcap_file)
    if TOPICO_INS in topicos:
        fonte, linhas = 'INSPVA (GNSS+INS)', ler(mcap_file, TOPICO_INS, extrair_ins)
    elif TOPICO_FIX in topicos:
        fonte, linhas = 'NavSatFix (só GNSS)', ler(mcap_file, TOPICO_FIX, extrair_fix)
    else:
        sys.exit('O bag não tem /novatel/oem7/inspva nem /novatel/oem7/fix.')

    linhas = [r for r in linhas if r['lat'] == r['lat'] and abs(r['lat']) > 1e-9]  # sem NaN nem 0,0
    if not linhas:
        sys.exit('Sem posições válidas no bag (o recetor ainda não tinha solução?).')
    t0 = linhas[0]['t']
    for r in linhas:
        r['t'] -= t0

    csv_path = pasta / 'gnss_ins.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
        w.writeheader()
        w.writerows(linhas)
    print(f'\n{len(linhas)} posições ({fonte}). CSV: {csv_path}')

    # Trajetória em metros (aproximação plana local, suficiente para percursos curtos)
    lat0, lon0 = linhas[0]['lat'], linhas[0]['lon']
    x = [(r['lon'] - lon0) * 111320 * math.cos(math.radians(lat0)) for r in linhas]
    y = [(r['lat'] - lat0) * 110540 for r in linhas]

    tem_atitude = any(r['azimuth'] == r['azimuth'] for r in linhas)
    fig, eixos = plt.subplots(1, 2 if tem_atitude else 1, figsize=(12 if tem_atitude else 6, 5))
    ax = eixos[0] if tem_atitude else eixos
    ax.plot(x, y)
    ax.plot(x[0], y[0], 'go', label='início')
    ax.plot(x[-1], y[-1], 'rs', label='fim')
    ax.set_aspect('equal')
    ax.set_xlabel('Este (m)')
    ax.set_ylabel('Norte (m)')
    ax.set_title(f'Trajetória - {fonte}')
    ax.grid(True)
    ax.legend()
    if tem_atitude:
        t = [r['t'] for r in linhas]
        for campo in ('roll', 'pitch', 'azimuth'):
            eixos[1].plot(t, [r[campo] for r in linhas], label=campo)
        eixos[1].set_xlabel('tempo (s)')
        eixos[1].set_ylabel('graus')
        eixos[1].set_title('Atitude')
        eixos[1].grid(True)
        eixos[1].legend()
    fig.tight_layout()
    png = pasta / 'gnss_ins.png'
    fig.savefig(png, dpi=150)
    print(f'Figura: {png}')
    plt.show()


if __name__ == '__main__':
    main()
