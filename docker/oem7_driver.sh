#!/bin/bash
# Arranca o driver NovAtel OEM7 conforme as variáveis do .env:
#   OEM7_IP preenchido  -> ligação por rede (oem7_net.launch.py)
#   OEM7_IP vazio       -> USB/série (oem7_port.launch.py), com deteção automática da porta

LAUNCH_DIR="$(ros2 pkg prefix novatel_oem7_driver)/share/novatel_oem7_driver/launch"
IP="${OEM7_IP:-}"
DEV="${OEM7_DEVICE:-}"
BAUD="${OEM7_BAUD:-115200}"

if [ -n "$IP" ]; then
    LAUNCH=oem7_net.launch.py
    ARGS=("oem7_ip_addr:=$IP")
    [ -n "${OEM7_PORT:-}" ] && ARGS+=("oem7_port:=$OEM7_PORT")
    [ -n "${OEM7_IF:-}" ] && ARGS+=("oem7_if:=$OEM7_IF")
    echo "[oem7] Ligação por rede ao recetor em $IP"
else
    if [ -z "$DEV" ]; then
        echo "[oem7] A procurar o recetor em USB/série (até 30 s)..."
        for _ in $(seq 1 30); do
            DEV="$(ls -d /dev/serial/by-id/*[Nn]ov[Aa][Tt]el* 2>/dev/null | head -n1)"
            [ -z "$DEV" ] && DEV="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1)"
            [ -n "$DEV" ] && break
            sleep 1
        done
    fi
    if [ -z "$DEV" ] || [ ! -e "$DEV" ]; then
        echo "[oem7] ERRO: não encontrei nenhuma porta USB/série."
        echo "       Verifica o cabo e a alimentação do recetor, e corre no portátil: ls /dev/ttyUSB* /dev/ttyACM*"
        echo "       Se o recetor liga por Ethernet, preenche OEM7_IP no ficheiro .env."
        exit 1
    fi
    LAUNCH=oem7_port.launch.py
    ARGS=("oem7_port_name:=$DEV" "oem7_port_baud:=$BAUD")
    echo "[oem7] Porta $DEV a $BAUD baud"
fi

if [ ! -f "$LAUNCH_DIR/$LAUNCH" ]; then
    echo "[oem7] ERRO: o launch file $LAUNCH não existe nesta versão do driver. Disponíveis:"
    ls "$LAUNCH_DIR"
    exit 1
fi

exec ros2 launch novatel_oem7_driver "$LAUNCH" "${ARGS[@]}"
