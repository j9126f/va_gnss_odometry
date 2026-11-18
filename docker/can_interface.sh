#!/bin/bash
# Configura a interface SocketCAN do adaptador USB2CAN (8devices, driver usb_8dev).
# Só mexe na interface se ela estiver em baixo (não interrompe outro contentor que já a use).
#   CAN_IF (can0)  CAN_BITRATE (500000)  CAN_LISTEN_ONLY (1 = nunca transmite nem dá ACK no barramento do carro)
IF="${CAN_IF:-can0}"
BITRATE="${1:-${CAN_BITRATE:-500000}}"
LISTEN="${CAN_LISTEN_ONLY:-1}"

for i in $(seq 1 600); do
    [ -e "/sys/class/net/$IF" ] && break
    [ $((i % 10)) -eq 1 ] && echo "[can] À espera da interface $IF (adaptador USB2CAN ligado? ver: ip link show)..."
    sleep 1
done
if [ ! -e "/sys/class/net/$IF" ]; then
    echo "[can] ERRO: a interface $IF não existe."
    exit 1
fi

if [ "${2:-}" != "forcar" ] && ip link show "$IF" | grep -q '[<,]UP[,>]'; then
    echo "[can] $IF já está ativa: $(ip -details link show "$IF" | grep -oE 'bitrate [0-9]+' | head -n1)"
    exit 0
fi

ip link set "$IF" down 2>/dev/null
MODO=off; [ "$LISTEN" = "1" ] && MODO=on
if ! ip link set "$IF" type can bitrate "$BITRATE" listen-only "$MODO" restart-ms 100 \
     || ! ip link set "$IF" up; then
    echo "[can] ERRO ao configurar $IF (o contentor precisa de cap_add: NET_ADMIN e network_mode: host)."
    exit 1
fi
echo "[can] $IF ativa a $BITRATE bit/s (listen-only: $MODO)"
