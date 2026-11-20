"""Nó de planeamento local: A* sobre a grelha de ocupação local.

Subscreve
  grelha_local   nav_msgs/OccupancyGrid     grelha no referencial do carro
  objetivo       geometry_msgs/PoseStamped  (opcional) objetivo local
Publica
  caminho_local  nav_msgs/Path              caminho planeado
  estado         std_msgs/String            LIVRE | DESVIO <m> | BLOQUEADO
  zona_proibida  nav_msgs/OccupancyGrid     obstáculos inflacionados (100)

Os nomes são relativos: com --ros-args -r __ns:=/g05 ficam /g05/grelha_local, ...
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .astar import ParametrosPlaneador, planear
from .grid import LocalGrid


class LocalAStar(Node):

    def __init__(self):
        super().__init__('local_astar')
        d = ParametrosPlaneador()
        self.declare_parameter('frequencia', 5.0)
        self.declare_parameter('objetivo_x', 25.0)
        self.declare_parameter('objetivo_y', 0.0)
        self.declare_parameter('raio_inflacao', d.raio_inflacao)
        self.declare_parameter('peso_heuristica', d.peso_heuristica)
        self.declare_parameter('desconhecido_livre', d.desconhecido_livre)
        self.declare_parameter('corredor_y_min', d.corredor_y_min)
        self.declare_parameter('corredor_y_max', d.corredor_y_max)
        self.declare_parameter('limiar_desvio', d.limiar_desvio)

        self.grelha = None
        self.objetivo = None
        self.create_subscription(OccupancyGrid, 'grelha_local', self.on_grelha, 10)
        self.create_subscription(PoseStamped, 'objetivo', self.on_objetivo, 10)
        self.pub_caminho = self.create_publisher(Path, 'caminho_local', 10)
        self.pub_estado = self.create_publisher(String, 'estado', 10)
        self.pub_proibido = self.create_publisher(OccupancyGrid, 'zona_proibida', 10)
        self.create_timer(1.0 / self.get_parameter('frequencia').value, self.ciclo)

    def on_grelha(self, msg):
        self.grelha = msg

    def on_objetivo(self, msg):
        self.objetivo = msg

    def ciclo(self):
        msg = self.grelha
        if msg is None:
            self.get_logger().info('à espera de grelha_local ...', throttle_duration_sec=5.0)
            return
        gp = self.get_parameter
        p = ParametrosPlaneador(
            raio_inflacao=gp('raio_inflacao').value,
            peso_heuristica=gp('peso_heuristica').value,
            desconhecido_livre=gp('desconhecido_livre').value,
            corredor_y_min=gp('corredor_y_min').value,
            corredor_y_max=gp('corredor_y_max').value,
            limiar_desvio=gp('limiar_desvio').value,
        )
        # objetivo: o do tópico `objetivo` (se houver) ou o dos parâmetros
        if self.objetivo is not None:
            objetivo = (self.objetivo.pose.position.x, self.objetivo.pose.position.y)
        else:
            objetivo = (gp('objetivo_x').value, gp('objetivo_y').value)

        r = planear(LocalGrid.de_msg(msg), (0.0, 0.0), objetivo, p)

        # caminho
        caminho = Path(header=msg.header)
        for x, y in r.caminho_xy:
            pose = PoseStamped(header=msg.header)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            caminho.poses.append(pose)
        self.pub_caminho.publish(caminho)

        # estado
        texto = f'DESVIO {r.desvio_max:.1f} m' if r.estado == 'DESVIO' else r.estado
        self.pub_estado.publish(String(data=texto))

        # zona proibida (para ver no Foxglove / rviz2)
        zona = OccupancyGrid(header=msg.header, info=msg.info)
        zona.data = np.where(r.proibido, 100, 0).astype(np.int8).ravel().tolist()
        self.pub_proibido.publish(zona)

        idade = self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)
        self.get_logger().info(
            f'{texto:14s} expandidos={r.expandidos:5d}  A*={r.tempo_ms:5.1f} ms  '
            f'idade da grelha={idade.nanoseconds * 1e-6:5.0f} ms',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = LocalAStar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
