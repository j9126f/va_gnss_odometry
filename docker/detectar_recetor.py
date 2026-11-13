#!/usr/bin/env python3
"""Procura o recetor NovAtel nas portas USB/série e descobre o baud rate.

Correr com o driver PARADO (a porta não pode estar em uso):
    docker compose down
    docker compose run --rm driver detectar_recetor.py
"""
import glob
import sys
import time

import serial

BAUDS = [115200, 9600, 230400, 460800, 57600, 38400, 19200]
SYNC_BINARIO = b"\xaa\x44\x12"  # início de um log binário NovAtel


def testar(dev, baud):
    """True = recetor respondeu; False = sem resposta; None = não foi possível abrir."""
    try:
        with serial.Serial(dev, baud, timeout=0.3) as s:
            s.reset_input_buffer()
            s.write(b"\r\nlog versiona once\r\n")
            fim = time.time() + 2.0
            buf = b""
            while time.time() < fim:
                buf += s.read(512)
                if b"VERSIONA" in buf or b"<OK" in buf or SYNC_BINARIO in buf:
                    return True
    except (serial.SerialException, OSError) as e:
        print(f"   não consegui abrir {dev}: {e}")
        return None
    return False


def main():
    dispositivos = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not dispositivos:
        print("Nenhuma porta /dev/ttyACM* ou /dev/ttyUSB* encontrada.")
        print("Verifica o cabo, a alimentação do recetor e `ls /dev/tty*`.")
        print("Se o recetor liga por Ethernet, usa OEM7_IP no .env (e `ping <IP>` para testar).")
        sys.exit(1)

    encontrados = []
    for dev in dispositivos:
        # Em USB (ttyACM) o baud rate é irrelevante: basta um teste
        bauds = BAUDS[:1] if "ttyACM" in dev else BAUDS
        print(f"A testar {dev} ...")
        for baud in bauds:
            r = testar(dev, baud)
            if r is None:
                break
            if r:
                print(f"   RESPONDEU a {baud} baud")
                encontrados.append((dev, baud))
                break
        else:
            print("   sem resposta")

    print()
    if not encontrados:
        print("Nenhuma porta respondeu. Confirma que o recetor está ligado e com GNSS/INS a arrancar,")
        print("que o driver está parado (docker compose down) e o cabo/porta COM usados.")
        sys.exit(2)

    print("Recetor encontrado. Coloca no ficheiro .env:")
    dev, baud = encontrados[0]
    print(f"   OEM7_DEVICE={dev}")
    if "ttyACM" not in dev:
        print(f"   OEM7_BAUD={baud}")
    if len(encontrados) > 1:
        print("Outras portas que também responderam:", ", ".join(d for d, _ in encontrados[1:]))


if __name__ == "__main__":
    main()
