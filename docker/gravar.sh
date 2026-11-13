#!/bin/bash
# Grava em MCAP os tópicos do recetor (/novatel/oem7/*) e as TF.
# O nome segue o padrão grupo_sensor_data_percurso (hora em UTC).
source /opt/ros/jazzy/setup.bash

NOME="${GRUPO:-g00}_gnss-ins_$(date +%Y%m%d_%H%M%S)_${PERCURSO:-teste}"

echo "[gravar] À espera do driver (tópicos /novatel/oem7/*)..."
ENCONTRADO=0
for _ in $(seq 1 120); do
    if ros2 topic list 2>/dev/null | grep -q '^/novatel/oem7/'; then ENCONTRADO=1; break; fi
    sleep 1
done
if [ "$ENCONTRADO" -eq 0 ]; then
    echo "[gravar] ERRO: o driver não está a publicar. Ver: docker compose logs driver"
    exit 1
fi

echo "[gravar] A gravar em /data/$NOME (Ctrl+C para parar)"
# exec: o ros2 bag recebe diretamente o sinal de paragem e fecha o MCAP corretamente
exec ros2 bag record -s mcap -o "/data/$NOME" -e '^/(novatel/.*|tf|tf_static)$'
