"""Driver simulado: LiDAR 2D a 3,5 m do base_link, numa rua com uma carrinha estacionada.

O ATLASCAR2 avança a `velocidade` m/s; a carrinha estacionada (e, no cenário
"bloqueio", um camião parado na outra faixa) aproxima-se, passa e o cenário
repete. Serve para testar a cadeia perceção -> grelha -> A* sem o carro real.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

SENSOR_X = 3.5    # posição do sensor no base_link (no projeto real vem do URDF)


def retangulo(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


class FakeScan(Node):

    def __init__(self):
        super().__init__('fake_scan')
        self.declare_parameter('frame_id', 'lidar2d_frente')
        self.declare_parameter('frequencia', 10.0)
        self.declare_parameter('velocidade', 4.0)
        self.declare_parameter('cenario', 'desvio')        # desvio | bloqueio
        self.frame_id = self.get_parameter('frame_id').value
        f = self.get_parameter('frequencia').value

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.t0 = self.get_clock().now()
        self.rng = np.random.default_rng(0)
        self.create_timer(1.0 / f, self.publicar)

        self.tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x = SENSOR_X
        tf.transform.translation.z = 0.4
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform(tf)

    def cena(self, t):
        """Segmentos (no base_link) visíveis no instante t.

        Estrada entre um passeio à direita (y = -3,5 m) e um muro à esquerda
        (y = +5 m); o carro segue em y = 0.
        """
        v = self.get_parameter('velocidade').value
        xo = 40.0 - (v * t) % 50.0               # traseira da carrinha estacionada
        seg = [((-10, 5.0), (60, 5.0)),          # muro à esquerda
               ((-10, -3.5), (60, -3.5))]        # passeio à direita
        seg += retangulo(xo, -3.3, xo + 5.0, -0.8)          # carrinha estacionada, invade a faixa
        if self.get_parameter('cenario').value == 'bloqueio':
            seg += retangulo(xo + 2.0, 1.0, xo + 9.0, 4.6)  # camião parado na outra faixa
        return seg

    def publicar(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        n = 361
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (n - 1)
        scan.range_min = 0.1
        scan.range_max = 40.0

        ang = scan.angle_min + scan.angle_increment * np.arange(n)
        dx, dy = np.cos(ang), np.sin(ang)
        r = np.full(n, np.inf)
        for (ax, ay), (bx, by) in self.cena(t):
            ax -= SENSOR_X
            bx -= SENSOR_X
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                s = (ax * ey - ay * ex) / den          # distância ao longo do raio
                u = (ax * dy - ay * dx) / den          # posição ao longo do segmento
            ok = (np.abs(den) > 1e-9) & (s > 0) & (u >= 0) & (u <= 1)
            r = np.where(ok & (s < r), s, r)
        r = r + self.rng.normal(0.0, 0.02, n)
        r[~np.isfinite(r) | (r > scan.range_max)] = np.inf     # sem eco
        scan.ranges = r.astype(float).tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = FakeScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
