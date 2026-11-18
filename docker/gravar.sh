#!/bin/bash
# Grava em MCAP os tópicos do recetor (/novatel/oem7/*), do CAN bus (/can/*) e as TF.
# Grava o que estiver disponível (GNSS/INS, CAN ou ambos); o nome do bag diz o que tem:
#   grupo_<gnss-ins-can | gnss-ins | can>_data_percurso (hora em UTC)
source /opt/ros/jazzy/setup.bash

tem() { ros2 topic list 2>/dev/null | grep -q "$1"; }

echo "[gravar] À espera do driver NovAtel (/novatel/oem7/*) e/ou do CAN (/can/raw)..."
GNSS=0; CAN=0
for i in $(seq 1 120); do
    tem '^/novatel/oem7/' && GNSS=1
    tem '^/can/raw$' && CAN=1
    [ "$GNSS" = 1 ] && [ "$CAN" = 1 ] && break
    # se só um apareceu, dá mais 20 s ao outro antes de começar
    if [ "$((GNSS + CAN))" = 1 ] && [ -z "$PRIMEIRO" ]; then PRIMEIRO=$i; fi
    [ -n "$PRIMEIRO" ] && [ "$i" -ge "$((PRIMEIRO + 20))" ] && break
    sleep 1
done

if [ "$GNSS" = 0 ] && [ "$CAN" = 0 ]; then
    echo "[gravar] ERRO: nada a publicar. Ver: docker compose logs driver can"
    exit 1
fi
[ "$GNSS" = 0 ] && echo "[gravar] AVISO: sem GNSS/INS (recetor ligado? ver: docker compose logs driver). Grava-se só o CAN."
[ "$CAN" = 0 ] && echo "[gravar] AVISO: sem /can/raw (adaptador CAN ligado? ver: docker compose logs can). Grava-se só o GNSS/INS."
if [ "$CAN" = 1 ] && ! tem '^/can/steering_angle$'; then
    echo "[gravar] AVISO: sem /can/steering_angle (CAN_DIRECAO_ID vazio?). Grava-se o CAN em bruto."
fi

SENSOR=$([ "$GNSS" = 1 ] && echo gnss-ins)$([ "$GNSS" = 1 ] && [ "$CAN" = 1 ] && echo -)$([ "$CAN" = 1 ] && echo can)
NOME="${GRUPO:-g00}_${SENSOR}_$(date +%Y%m%d_%H%M%S)_${PERCURSO:-teste}"

echo "[gravar] A gravar em /data/$NOME (Ctrl+C para parar)"
# exec: o ros2 bag recebe diretamente o sinal de paragem e fecha o MCAP corretamente
exec ros2 bag record -s mcap -o "/data/$NOME" -e '^/(novatel/.*|can/.*|tf|tf_static)$'
