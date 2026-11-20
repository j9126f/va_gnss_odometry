"""Cria uma rota (CSV lat,lon) a partir da trajetória do INSPVA gravada num bag ("teach and repeat").

    docker compose run --rm planeador ros2 run va_g07 rota_de_bag /data/<bag> /data/rota.csv [espacamento_m]

Usa só as mensagens com solução INS (status 3, 6 ou 7) e lat/lon diferentes de 0,
descarta saltos isolados e deixa um waypoint a cada ~espacamento m (2 m por omissão).
"""
import sys

import rosbag2_py
from novatel_oem7_msgs.msg import INSPVA
from rclpy.serialization import deserialize_message

from .rota import gravar_csv, reduzir
from .rota_objetivo import ESTADOS_VALIDOS


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    bag, saida = sys.argv[1], sys.argv[2]
    espacamento = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    leitor = rosbag2_py.SequentialReader()
    leitor.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    leitor.set_filter(rosbag2_py.StorageFilter(topics=['/novatel/oem7/inspva']))
    pontos, total = [], 0
    while leitor.has_next():
        _, dados, _ = leitor.read_next()
        m = deserialize_message(dados, INSPVA)
        total += 1
        if m.status.status in ESTADOS_VALIDOS:
            pontos.append((m.latitude, m.longitude))

    rota = reduzir(pontos, espacamento)
    if len(rota) < 2:
        sys.exit(f'{bag}: sem solução INS suficiente para fazer uma rota ({len(pontos)}/{total} válidas)')
    gravar_csv(saida, rota)
    print(f'{total} INSPVA, {len(pontos)} com solução INS -> {len(rota)} waypoints em {saida}')
