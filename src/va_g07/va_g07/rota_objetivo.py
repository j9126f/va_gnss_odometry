"""Nó do grupo 07: rota global (GNSS) -> objetivo local do planeador.

Subscreve
  /novatel/oem7/inspva  novatel_oem7_msgs/INSPVA   posição (lat, lon) e azimute do carro
Publica (nomes relativos: com __ns:=/g07 ficam /g07/...)
  objetivo        geometry_msgs/PoseStamped  ponto da rota a `distancia_objetivo` m à frente, no base_link
                                             (ou o último dentro da grelha, numa curva apertada)
  rota_local      nav_msgs/Path              troço da rota entre o carro e o objetivo, no base_link
  grelha_local    nav_msgs/OccupancyGrid     toda -1 (o GNSS não vê obstáculos); só com publicar_grelha,
                                             para o planeador correr sem outro nó de perceção
O header.stamp de tudo é o do INSPVA que lhe deu origem.
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from novatel_oem7_msgs.msg import INSPVA
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from va_planeamento.grid import LocalGrid

from .rota import SeguidorRota, ler_csv

# INSPVA.status: com estes há posição e azimute utilizáveis (ver ler_bag / Ex. 3)
ESTADOS_VALIDOS = {3, 6, 7}     # SOLUTION_GOOD, SOLUTION_FREE, ALIGNMENT_COMPLETE


class RotaObjetivo(Node):

    def __init__(self):
        super().__init__('rota_objetivo')
        rota_omissao = os.path.join(get_package_share_directory('va_g07'), 'rotas', 'teste1.csv')
        self.declare_parameter('ficheiro_rota', rota_omissao)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('distancia_objetivo', 25.0)
        self.declare_parameter('distancia_max_rota', 30.0)
        self.declare_parameter('frequencia', 5.0)
        self.declare_parameter('publicar_grelha', True)
        self.declare_parameter('resolucao', 0.2)
        self.declare_parameter('x_min', -5.0)
        self.declare_parameter('x_max', 35.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        ficheiro = gp('ficheiro_rota')
        latlon = ler_csv(ficheiro)
        self.seguidor = SeguidorRota(latlon, gp('distancia_objetivo'), gp('distancia_max_rota'))
        self.get_logger().info(
            f'rota {ficheiro}: {len(latlon)} waypoints, {self.seguidor.comprimento:.0f} m')

        self.periodo = 1.0 / gp('frequencia')
        self.ultimo = None
        self.pub_objetivo = self.create_publisher(PoseStamped, 'objetivo', 10)
        self.pub_rota = self.create_publisher(Path, 'rota_local', 10)
        self.pub_grelha = self.create_publisher(OccupancyGrid, 'grelha_local', 10)
        self.create_subscription(INSPVA, '/novatel/oem7/inspva', self.on_inspva, 10)

    def on_inspva(self, msg):
        # limitar à frequência do planeador (o INSPVA chega a 50 Hz)
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.ultimo is not None and 0 <= t - self.ultimo < self.periodo:
            return
        self.ultimo = t

        if msg.status.status not in ESTADOS_VALIDOS or (msg.latitude == 0 and msg.longitude == 0):
            self.get_logger().info(f'INS ainda sem solução (status {msg.status.status})',
                                   throttle_duration_sec=5.0)
            return

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        limites = (gp('x_min'), gp('x_max'), gp('y_min'), gp('y_max'))
        r = self.seguidor.atualizar(msg.latitude, msg.longitude, msg.azimuth, limites)
        if r is None:
            self.get_logger().warn('carro fora da rota (> distancia_max_rota): sem objetivo',
                                   throttle_duration_sec=2.0)
            return
        (ox, oy), troco, dist_rota, chegou = r

        header = msg.header
        header.frame_id = self.get_parameter('frame_id').value

        objetivo = PoseStamped(header=header)
        objetivo.pose.position.x = float(ox)
        objetivo.pose.position.y = float(oy)
        # orientação: direção do último segmento da rota até ao objetivo
        if len(troco) >= 2:
            dx, dy = troco[-1] - troco[-2]
            yaw = math.atan2(dy, dx)
            objetivo.pose.orientation.z = math.sin(yaw / 2)
            objetivo.pose.orientation.w = math.cos(yaw / 2)
        else:
            objetivo.pose.orientation.w = 1.0
        self.pub_objetivo.publish(objetivo)

        caminho = Path(header=header)
        for x, y in troco:
            p = PoseStamped(header=header)
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            caminho.poses.append(p)
        self.pub_rota.publish(caminho)

        if gp('publicar_grelha'):
            grelha = LocalGrid(gp('resolucao'), gp('x_min'), gp('x_max'), gp('y_min'), gp('y_max'))
            self.pub_grelha.publish(grelha.para_msg(header))   # toda -1: o GNSS não vê obstáculos

        self.get_logger().info(
            f'objetivo x={ox:5.1f} y={oy:5.1f} m  a {dist_rota:4.1f} m da rota  '
            f'waypoint {self.seguidor.indice}/{len(self.seguidor.latlon)}'
            + ('  FIM DA ROTA' if chegou else ''),
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = RotaObjetivo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
