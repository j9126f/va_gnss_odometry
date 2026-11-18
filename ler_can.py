"""Leitura off-line do CAN bus gravado num bag MCAP, sem ROS (Linux, Windows ou macOS).

    ./ler_can.sh data/<pasta_do_bag>
ou, à mão:
    pip install mcap mcap-ros2-support matplotlib
    python ler_can.py data/<pasta_do_bag>

Mostra os IDs presentes em /can/raw (taxa e n.º de tramas) e, se CAN_DIRECAO_ID estiver no .env,
descodifica o ângulo de direção a partir das tramas brutas (serve também para bags gravados antes
de se saber o ID, ou para corrigir a escala/sentido depois). Grava na pasta do bag can_direcao.csv e
can_direcao.png. Se o bag tiver /can/steering_angle (descodificado no carro), compara os dois.
"""
import collections
import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages

sys.path.insert(0, str(Path(__file__).resolve().parent / 'docker'))
from can_sinal import SinalDirecao  # noqa: E402

TOPICO_RAW = '/can/raw'
TOPICO_DIRECAO = '/can/steering_angle'


def ler_env(caminho):
    env = {}
    if caminho.exists():
        for linha in caminho.read_text(encoding='utf-8').splitlines():
            linha = linha.split('#', 1)[0].strip()
            if '=' in linha:
                k, v = linha.split('=', 1)
                env[k.strip()] = v.strip()
    return env


def main():
    if len(sys.argv) < 2:
        sys.exit('uso: python ler_can.py data/<pasta_do_bag>')
    bag = Path(sys.argv[1])
    mcap_file = next(bag.glob('*.mcap')) if bag.is_dir() else bag
    pasta = mcap_file.parent
    sinal = SinalDirecao.do_ambiente(ler_env(Path(__file__).resolve().parent / '.env'))

    ids = collections.Counter()
    t_raw, ang_raw, t_dir, ang_dir = [], [], [], []
    for m in read_ros2_messages(str(mcap_file), topics=[TOPICO_RAW, TOPICO_DIRECAO]):
        t = m.log_time_ns * 1e-9
        msg = m.ros_msg
        if m.channel.topic == TOPICO_DIRECAO:
            t_dir.append(t)
            ang_dir.append(math.degrees(msg.drive.steering_angle))
            continue
        if msg.is_error:
            continue
        ids[(msg.id, msg.is_extended)] += 1
        if sinal and msg.id == sinal.can_id and msg.is_extended == sinal.extendido:
            t_raw.append(t)
            ang_raw.append(math.degrees(sinal.rodas_rad(bytes(msg.data[:msg.dlc]))))

    if not ids and not t_dir:
        sys.exit(f'O bag não tem {TOPICO_RAW} nem {TOPICO_DIRECAO}.')
    if ids:
        print(f'{TOPICO_RAW}: {sum(ids.values())} tramas, {len(ids)} IDs')
        for (cid, ext), n in sorted(ids.items()):
            print(f"  {'0x%08X' % cid if ext else '0x%03X' % cid:>11}  {n:>8}")
    if not sinal:
        print('\nCAN_DIRECAO_ID vazio no .env: só a lista de IDs. Para o descobrir: '
              'docker compose run --rm can can_identificar.py')
        if not t_dir:
            return
    else:
        print(f'\nSinal: {sinal.descricao()}\n{len(ang_raw)} ângulos descodificados de {TOPICO_RAW}')

    t0 = min((t_raw or [math.inf])[0], (t_dir or [math.inf])[0])
    csv_path = pasta / 'can_direcao.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'angulo_rodas_graus', 'fonte'])
        w.writerows([t - t0, a, 'raw'] for t, a in zip(t_raw, ang_raw))
        w.writerows([t - t0, a, 'steering_angle'] for t, a in zip(t_dir, ang_dir))
    print(f'CSV: {csv_path}')

    fig, ax = plt.subplots(figsize=(10, 4))
    if t_raw:
        ax.plot([t - t0 for t in t_raw], ang_raw, label=f'descodificado de {TOPICO_RAW} (.env atual)')
    if t_dir:
        ax.plot([t - t0 for t in t_dir], ang_dir, '--', label=f'{TOPICO_DIRECAO} (gravado no carro)')
    ax.set_xlabel('tempo (s)')
    ax.set_ylabel('ângulo das rodas (graus, + = esquerda)')
    ax.set_title('Direção (CAN bus)')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    png = pasta / 'can_direcao.png'
    fig.savefig(png, dpi=150)
    print(f'Figura: {png}')
    plt.show()


if __name__ == '__main__':
    main()
