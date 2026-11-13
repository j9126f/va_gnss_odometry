#!/bin/bash
# Verificações rápidas com o driver a correr:
#   docker compose exec driver verificar.sh
source /opt/ros/jazzy/setup.bash

echo "=== Tópicos do recetor"
ros2 topic list 2>/dev/null | grep -E '^/novatel/oem7/(inspva|bestpos|fix|odom|oem7raw|imu/data)$' \
    || echo "NENHUM: o driver não está a publicar (ver: docker compose logs driver)"

echo
echo "=== Taxa de /novatel/oem7/inspva (tipicamente ~50 Hz com SPAN)"
timeout 6 ros2 topic hz /novatel/oem7/inspva 2>&1 | tail -n 3

echo
echo "=== Última mensagem INSPVA (posição, atitude e estado do INS)"
timeout 8 ros2 topic echo --once /novatel/oem7/inspva 2>&1 | head -n 40

echo
echo "=== Posição GNSS (BESTPOS)"
timeout 8 ros2 topic echo --once /novatel/oem7/bestpos 2>&1 | head -n 40

echo
echo "=== Frame do sensor (odometria)"
timeout 8 ros2 topic echo --once /novatel/oem7/odom --field header.frame_id 2>&1 | head -n 3
