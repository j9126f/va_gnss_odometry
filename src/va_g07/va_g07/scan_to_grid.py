"""Perceção (exemplo): LaserScan -> grelha de ocupação local no base_link.

É ESTE o nó que cada grupo substitui pelo seu: subscreve o tópico standard
do sensor, extrai o que interessa (impactos, clusters, linhas, ...) e publica
o resultado em `grelha_local` (nav_msgs/OccupancyGrid, frame base_link).
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from va_planeamento.grid import LocalGrid


class ScanToGrid(Node):

    def __init__(self):
        super().__init__('scan_to_grid')
        self.declare_parameter('frame_grelha', 'base_link')
        self.declare_parameter('resolucao', 0.2)
        self.declare_parameter('x_min', -5.0)
        self.declare_parameter('x_max', 35.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        self.declare_parameter('marcar_livre', True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(OccupancyGrid, 'grelha_local', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, scan):
        gp = self.get_parameter
        frame = gp('frame_grelha').value
        # 1. onde está o sensor no carro? (vem do /tf_static, ou seja, do URDF)
        try:
            tf = self.tf_buffer.lookup_transform(frame, scan.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'sem TF {frame} <- {scan.header.frame_id}: {e}',
                                   throttle_duration_sec=5.0)
            return
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)

        # 2. impactos válidos, do referencial do sensor para o base_link
        r = np.asarray(scan.ranges)
        a = scan.angle_min + scan.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        xs, ys = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        xb, yb = tx + xs * c - ys * s, ty + xs * s + ys * c

        # 3. grelha: livre entre o sensor e os impactos, ocupado nos impactos
        g = LocalGrid(gp('resolucao').value, gp('x_min').value, gp('x_max').value,
                      gp('y_min').value, gp('y_max').value)
        if gp('marcar_livre').value:
            # um raio sem eco também mostra espaço livre até ao alcance máximo
            rl = np.where(ok, r, np.where(np.isinf(r), scan.range_max, 0.0))
            xl, yl = rl * np.cos(a), rl * np.sin(a)
            g.marcar_raios_livres(tx, ty, tx + xl * c - yl * s, ty + xl * s + yl * c)
        g.marcar_pontos(xb, yb)

        msg = g.para_msg(scan.header)
        msg.header.frame_id = frame          # mesmo instante do scan, outro referencial
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanToGrid()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
