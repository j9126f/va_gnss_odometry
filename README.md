# Aquisição GNSS/INS - NovAtel SPAN-IGM-S1 (ROS 2 Jazzy)

Pensado para Ubuntu 24.04 nativo com Docker Engine. No Windows só serve para ver e analisar bags.

| Ficheiro | Função |
|---|---|
| `Dockerfile` | Imagem: ROS 2 Jazzy + driver NovAtel + MCAP + Foxglove bridge |
| `compose.yaml` | Serviços: `driver`, `gravador` (profile `gravar`), `foxglove` (profile `foxglove`) |
| `.env` | Configuração: grupo, percurso, IP/porta/baud, pasta dos bags |
| `docker/` | Scripts que correm dentro do contentor (driver, gravação, verificação, deteção da porta) |
| `src/` | Pacotes ROS 2 do grupo (nós de processamento), compilados no build |
| `data/` | Bags gravados (na aquisição real, apontar `DATA_DIR` para o SSD externo) |
| `ler_bag.sh` / `ler_bag.py` | Análise off-line sem ROS: resumo, trajetória, atitude, CSV |

## Comandos

```bash
docker compose build                                  # uma vez (demora alguns minutos)
docker compose run --rm driver detectar_recetor.py    # só se a deteção automática falhar
docker compose up driver                              # arranca o driver
docker compose exec driver verificar.sh               # (outro terminal) confirma que há dados
docker compose --profile gravar up                    # driver + gravação (Ctrl+C para parar)
./ler_bag.sh data/<pasta_do_bag>                      # análise
sudo chown -R $USER data                              # se precisares de mover/apagar bags
```

## Problemas comuns

- **Sem porta série**: `ls /dev/ttyUSB* /dev/ttyACM*` no portátil; confirmar cabo e alimentação.
- **Porta ocupada** ao detetar: parar o driver (`docker compose down`).
- **Sem dados em `inspva` / INS não alinhado**: o INS precisa de GNSS e de movimento para alinhar.
- **Ethernet**: preencher `OEM7_IP` no `.env` e confirmar com `ping`.
- **Acesso ao USB falha**: em `compose.yaml`, no serviço `driver`, acrescentar `privileged: true`.
