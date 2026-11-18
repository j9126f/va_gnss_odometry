#!/usr/bin/env python3
"""Nó ROS 2: lê a mensagem do ângulo de direção diretamente do SocketCAN (filtro no kernel, só esse ID)
e publica o ângulo de viragem das rodas da frente em /can/steering_angle
(ackermann_msgs/AckermannDriveStamped, drive.steering_angle em rad, positivo = esquerda).
O carimbo de tempo é o da receção da trama no kernel. Configuração: CAN_DIRECAO_* no .env (ver can_sinal.py).
"""
import math
import os
import socket
import struct

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from can_sinal import SinalDirecao

CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG = 0x80000000, 0x40000000, 0x20000000
CAN_SFF_MASK, CAN_EFF_MASK = 0x7FF, 0x1FFFFFFF
TOPICO = '/can/steering_angle'
SO_TIMESTAMP = getattr(socket, 'SO_TIMESTAMP', 29)   # = SCM_TIMESTAMP (Linux); o Python 3.12 não o define


def abrir_socket(interface, sinal):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    if sinal.extendido:
        fid, mask = sinal.can_id | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
    else:
        fid, mask = sinal.can_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
    s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', fid, mask))
    s.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMP, 1)
    s.bind((interface,))
    s.settimeout(0.5)
    return s


def main():
    sinal = SinalDirecao.do_ambiente()
    if sinal is None:
        raise SystemExit('[can_direcao] CAN_DIRECAO_ID não definido no .env')
    interface = os.environ.get('CAN_IF', 'can0')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C -> KeyboardInterrupt, saída limpa
    node = rclpy.create_node('can_direcao')
    pub = node.create_publisher(AckermannDriveStamped, TOPICO, qos_profile_sensor_data)
    log = node.get_logger()
    log.info(f'{interface}: {sinal.descricao()} -> {TOPICO}')

    s = abrir_socket(interface, sinal)
    tam_anc = socket.CMSG_SPACE(struct.calcsize('@ll'))
    n, ultimo_log = 0, None
    try:
        while rclpy.ok():
            try:
                frame, anc, _, _ = s.recvmsg(16, tam_anc)
            except socket.timeout:
                if n and ultimo_log != -1:
                    log.warn('Sem mensagens de direção há 0,5 s')
                    ultimo_log = -1
                continue
            _, dlc = struct.unpack('=IB3x', frame[:8])
            msg = AckermannDriveStamped()
            stamp = None
            for nivel, tipo, dados in anc:
                if nivel == socket.SOL_SOCKET and tipo == SO_TIMESTAMP:
                    sec, usec = struct.unpack('@ll', dados[:struct.calcsize('@ll')])
                    stamp = Time(sec=sec, nanosec=usec * 1000)
            msg.header.stamp = stamp or node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.drive.steering_angle = sinal.rodas_rad(frame[8:8 + dlc])
            pub.publish(msg)
            n += 1
            if n == 1 or n % 5000 == 0 or ultimo_log == -1:
                log.info(f'{n} mensagens; ângulo das rodas = {math.degrees(msg.drive.steering_angle):+.2f}º')
                ultimo_log = n
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
