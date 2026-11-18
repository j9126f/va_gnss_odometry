# Aquisição GNSS/INS (NovAtel SPAN-IGM-S1) + CAN bus do carro (ROS 2 Jazzy)

Pensado para Ubuntu 24.04 nativo com Docker Engine. No Windows só serve para ver e analisar bags.

| Ficheiro | Função |
|---|---|
| `Dockerfile` | Imagem: ROS 2 Jazzy + driver NovAtel + MCAP + Foxglove bridge |
| `compose.yaml` | Serviços: `driver`, `can`, `gravador` (profile `gravar`), `foxglove` (profile `foxglove`) |
| `.env` | Configuração: grupo, percurso, IP/porta/baud, pasta dos bags, CAN (bitrate, sinal da direção) |
| `docker/` | Scripts que correm dentro do contentor (driver, gravação, verificação, deteção da porta) |
| `src/` | Pacotes ROS 2 do grupo (nós de processamento), compilados no build |
| `data/` | Bags gravados (na aquisição real, apontar `DATA_DIR` para o SSD externo) |
| `ler_bag.sh` / `ler_bag.py` | Análise off-line sem ROS: resumo, trajetória, atitude, CSV |
| `ler_can.sh` / `ler_can.py` | CAN off-line sem ROS: IDs gravados e ângulo de direção descodificado de `/can/raw` (CSV + PNG) |

## Comandos

```bash
docker compose build                                  # uma vez (demora alguns minutos)
docker compose run --rm driver detectar_recetor.py    # só se a deteção automática falhar
docker compose up driver                              # arranca o driver
docker compose exec driver verificar.sh               # (outro terminal) confirma que há dados
docker compose --profile gravar up                    # driver + CAN + gravação (Ctrl+C para parar)
./ler_bag.sh data/<pasta_do_bag>                      # análise
sudo chown -R $USER data                              # se precisares de mover/apagar bags
```

## CAN bus (ângulo de direção)

Adaptador USB2CAN (8devices) -> interface SocketCAN `can0`. O serviço `can` configura-a em **listen-only**
(não transmite nem confirma tramas: não interfere com o carro) e publica:

| Tópico | Tipo | Conteúdo |
|---|---|---|
| `/can/raw` | `can_msgs/Frame` | todas as tramas do barramento (sempre) |
| `/can/steering_angle` | `ackermann_msgs/AckermannDriveStamped` | `drive.steering_angle` = ângulo das rodas da frente (rad, + = esquerda), só com `CAN_DIRECAO_ID` definido |

A mensagem do ângulo é diferente em cada carro. Para a descobrir (carro ligado e parado):

```bash
docker compose run --rm can can_identificar.py --bitrate auto   # 1.ª vez: descobre o bitrate -> CAN_BITRATE
docker compose run --rm can can_identificar.py                  # centro -> batente esq. -> batente dir. -> 1 volta
```

Copiar as linhas `CAN_DIRECAO_*` sugeridas para o `.env`. Depois:
- **Sentido**: virar à esquerda tem de dar ângulo positivo; senão `CAN_DIRECAO_SENTIDO=-1`.
- **Escala**: medida na última fase (volante rodado exatamente uma volta, 360º, para a esquerda: o
  símbolo do volante fica outra vez direito). Não é preciso saber o ângulo do batente.
- **Volante vs rodas**: a escala medida dá graus de volante. `CAN_RELACAO_DIRECAO` = desmultiplicação da
  direção (ficha técnica, ≈ 14-17, ou ângulo do volante no batente / ângulo das rodas no batente);
  o `/can/steering_angle` sai já em ângulo das rodas.

Como `/can/raw` é sempre gravado, um bag feito antes de acertar o sinal não se perde: corrigir o `.env` e
correr `./ler_can.sh data/<pasta_do_bag>`. A velocidade das rodas (odómetro) vem também em `/can/raw`.

## Problemas comuns

- **Sem porta série**: `ls /dev/ttyUSB* /dev/ttyACM*` no portátil; confirmar cabo e alimentação.
- **Porta ocupada** ao detetar: parar o driver (`docker compose down`).
- **Sem dados em `inspva` / INS não alinhado**: o INS precisa de GNSS e de movimento para alinhar.
- **Ethernet**: preencher `OEM7_IP` no `.env` e confirmar com `ping`.
- **CAN sem tramas**: carro com a ignição ligada? bitrate certo (`can_identificar.py --bitrate auto`)?
  `docker compose logs can` e, no portátil, `ip -details -statistics link show can0` (erros de barramento = bitrate errado).
- **Mudar o bitrate com o serviço `can` a correr**: `docker compose down` primeiro (a interface só é reconfigurada se estiver em baixo).
- **Acesso ao USB falha**: em `compose.yaml`, no serviço `driver`, acrescentar `privileged: true`.
