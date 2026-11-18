#!/bin/bash
# Serviço "can": configura a interface, grava todas as tramas em /can/raw (can_msgs/Frame)
# e, se CAN_DIRECAO_ID estiver definido no .env, publica o ângulo das rodas em /can/steering_angle.
source /opt/ros/jazzy/setup.bash
set -m   # processos em segundo plano recebem o SIGINT (sem isto o bash fá-los ignorá-lo)
can_interface.sh || exit 1
IF="${CAN_IF:-can0}"

ros2 launch ros2_socketcan socket_can_receiver.launch.py \
    interface:="$IF" from_can_bus_topic:=/can/raw \
    use_bus_time:=true warn_on_receive_timeout:=false &
PIDS=($!)

if [ -n "${CAN_DIRECAO_ID:-}" ]; then
    can_direcao.py &
    PIDS+=($!)
else
    echo "[can] CAN_DIRECAO_ID vazio no .env: só se gravam as tramas brutas (/can/raw)."
    echo "      Para descobrir a mensagem do ângulo: docker compose run --rm can can_identificar.py"
fi

# Ctrl+C / docker stop -> para todos; se um processo morrer, sai (o compose volta a arrancar o serviço)
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait; exit 0' INT TERM
todos_vivos() { for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null || return 1; done; }
while todos_vivos; do sleep 1; done
echo "[can] Um dos processos terminou; a sair."
kill -INT "${PIDS[@]}" 2>/dev/null
wait
exit 1
