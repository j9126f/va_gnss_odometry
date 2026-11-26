#!/usr/bin/env python3
"""Encoder da roda (Arduino ATmega328P Xplained Mini, USB série 115200 baud) -> ROS 2.

O Arduino escreve linhas de texto (~90/s), p. ex.:
    position = 0
    0
    0 0 0 0
    CAN BUS sendMsgBuf ok!
Publica:
    /encoder/raw      std_msgs/String  todas as linhas, tal como vêm (para descodificar depois)
    /encoder/posicao  std_msgs/Int64   o N de "position = N"
O instante de cada mensagem fica no bag (log_time), com o mesmo relógio do GNSS e do CAN.
Porta: ENCODER_DEVICE no .env; vazio = /dev/serial/by-id/*mEDBG* (ou *Arduino*). ENCODER_BAUD (115200).
"""
import glob
import os
import re
import termios
import time

import rclpy
from std_msgs.msg import Int64, String

BAUDS = {9600: termios.B9600, 57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400}
POSICAO = re.compile(r'position\s*=\s*(-?\d+)')


def porta():
    dev = os.environ.get('ENCODER_DEVICE', '').strip()
    if dev:
        return dev
    for padrao in ('*mEDBG*', '*Arduino*', '*arduino*'):
        achados = sorted(glob.glob(f'/dev/serial/by-id/{padrao}'))
        if achados:
            return achados[0]
    return None


def abrir(dev, baud):
    fd = os.open(dev, os.O_RDONLY | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = 0                                             # iflag: em bruto
    a[1] = 0                                             # oflag
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1
    a[3] = 0                                             # lflag: sem eco nem modo canónico
    a[4] = a[5] = BAUDS[baud]
    a[6][termios.VMIN], a[6][termios.VTIME] = 1, 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def main():
    rclpy.init()
    no = rclpy.create_node('encoder')
    log = no.get_logger()
    pub_raw = no.create_publisher(String, '/encoder/raw', 100)
    pub_pos = no.create_publisher(Int64, '/encoder/posicao', 100)
    baud = int(os.environ.get('ENCODER_BAUD', '115200') or 115200)

    dev = None
    while rclpy.ok() and dev is None:
        dev = porta()
        if dev is None or not os.path.exists(dev):
            log.warn('À espera do encoder (USB ligado? ENCODER_DEVICE no .env)...', throttle_duration_sec=10)
            dev = None
            time.sleep(1)
    fd = abrir(dev, baud)
    log.info(f'Encoder em {dev} a {baud} baud')

    buf, n, ultimo, t_info = b'', 0, None, time.time()
    try:
        while rclpy.ok():
            dados = os.read(fd, 512)
            if not dados:
                raise OSError('porta fechada')
            buf += dados
            *linhas, buf = buf.split(b'\n')
            for linha in linhas:
                texto = linha.decode('ascii', 'replace').strip()
                if not texto:
                    continue
                pub_raw.publish(String(data=texto))
                m = POSICAO.search(texto)
                if m:
                    ultimo = int(m.group(1))
                    pub_pos.publish(Int64(data=ultimo))
                    n += 1
            if time.time() - t_info > 10:
                log.info(f'{n} posições em 10 s; última = {ultimo}')
                n, t_info = 0, time.time()
    except OSError as e:
        log.error(f'Encoder: {e} (USB desligado?). O serviço volta a arrancar.')
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
