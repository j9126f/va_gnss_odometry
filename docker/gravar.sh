#!/bin/bash
# Grava em MCAP o que estiver a ser publicado: recetor (/novatel/oem7/*), CAN bus (/can/*), encoder (/encoder/*),
# LiDAR simulado ou reproduzido (/scan), resultados do grupo (/<GRUPO>/*) e as TF.
# O nome do bag diz o que tem (hora em UTC):
#   <grupo>_<gnss-ins-can-encoder | gnss-ins | can | encoder | resultados>_<data>_<percurso>
#   (com sensores e resultados: <grupo>_gnss-ins-can-resultados_...)
source /opt/ros/jazzy/setup.bash
G="${GRUPO:-g07}"

# só tópicos com publicador (o "ros2 topic list" também mostra os que só têm subscritores,
# p. ex. /novatel/oem7/inspva subscrito pelo nó rota_objetivo sem o recetor ligado)
publicados() { ros2 topic list -v 2>/dev/null | sed -n '/^Published topics:/,/^Subscribed topics:/p' | awk '/^ \* /{print $2}'; }
tem() { publicados | grep -q "$1"; }

echo "[gravar] À espera do driver NovAtel, do CAN, do /scan ou dos resultados /$G/* ..."
GNSS=0; CAN=0; ENC=0; RES=0
for i in $(seq 1 120); do
    tem '^/novatel/oem7/' && GNSS=1
    tem '^/can/raw$' && CAN=1
    tem '^/encoder/' && ENC=1
    tem "^/$G/" && RES=1
    [ "$GNSS" = 1 ] && [ "$CAN" = 1 ] && [ "$ENC" = 1 ] && break
    # se só um apareceu, dá mais 20 s aos outros antes de começar
    if [ "$((GNSS + CAN + ENC + RES))" -ge 1 ] && [ -z "$PRIMEIRO" ]; then PRIMEIRO=$i; fi
    [ -n "$PRIMEIRO" ] && [ "$i" -ge "$((PRIMEIRO + 20))" ] && break
    sleep 1
done

if [ "$((GNSS + CAN + ENC + RES))" = 0 ]; then
    echo "[gravar] ERRO: nada a publicar. Ver: docker compose logs driver can percecao planeador"
    exit 1
fi
if [ "$((GNSS + CAN + ENC))" -ge 1 ]; then
    [ "$GNSS" = 0 ] && echo "[gravar] AVISO: sem GNSS/INS (recetor ligado? ver: docker compose logs driver)."
    [ "$CAN" = 0 ] && echo "[gravar] AVISO: sem /can/raw (adaptador CAN ligado? ver: docker compose logs can)."
    [ "$ENC" = 0 ] && echo "[gravar] AVISO: sem /encoder/* (Arduino ligado? ver: docker compose logs encoder)."
fi
if [ "$CAN" = 1 ] && ! tem '^/can/steering_angle$'; then
    echo "[gravar] AVISO: sem /can/steering_angle (CAN_DIRECAO_ID vazio?). Grava-se o CAN em bruto."
fi

PARTES=()
[ "$GNSS" = 1 ] && PARTES+=(gnss-ins)
[ "$CAN" = 1 ] && PARTES+=(can)
[ "$ENC" = 1 ] && PARTES+=(encoder)
[ "$RES" = 1 ] && PARTES+=(resultados)
SENSOR=$(IFS=-; echo "${PARTES[*]}")
NOME="${G}_${SENSOR}_$(date +%Y%m%d_%H%M%S)_${PERCURSO:-teste}"

echo "[gravar] A gravar em /data/$NOME (Ctrl+C para parar)"
# exec: o ros2 bag recebe diretamente o sinal de paragem e fecha o MCAP corretamente
exec ros2 bag record -s mcap -o "/data/$NOME" -e "^/(novatel/.*|can/.*|encoder/.*|scan|$G/.*|tf|tf_static)$"
