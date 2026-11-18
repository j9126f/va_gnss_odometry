#!/bin/bash
# Lê o CAN bus de um bag MCAP no portátil (sem ROS). Cria sozinho um ambiente Python na primeira vez.
#   ./ler_can.sh data/<pasta_do_bag>
if [ -z "$1" ]; then echo "uso: ./ler_can.sh data/<pasta_do_bag>"; exit 1; fi
BAG="$(realpath "$1")"
cd "$(dirname "$0")" || exit 1
if [ ! -d .venv ]; then
    python3 -m venv .venv || { echo "Falta o venv. Corre: sudo apt install -y python3-venv"; exit 1; }
fi
.venv/bin/pip install -q mcap mcap-ros2-support matplotlib || exit 1
.venv/bin/python ler_can.py "$BAG"
