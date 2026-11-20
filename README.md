# VA 2026/2027 · Grupo 07 · GNSS + odometria

Localização do carro com GNSS/INS NovAtel SPAN-IGM-S1 e odometria do CAN bus (ângulo de direção do
Mitsubishi i-MiEV): aquisição em ROS 2, análise off-line e, na parte 2, o próximo waypoint da rota
global publicado como objetivo do planeador local (A*).

<!-- ![Demonstração](docs/img/demo.gif)   (Ex. 6: gerar com tools/figura_bag.py) -->

| | |
|---|---|
| **Sensor** | NovAtel SPAN-IGM-S1 (GNSS + IMU ADIS16488), USB-série 230400 baud; INSPVAS 50 Hz, BESTPOS 10 Hz, RAWIMUSX 200 Hz · CAN bus do carro via USB2CAN (8devices), 500 kbit/s, listen-only |
| **Posição no carro** | `imu_link`: (-1,30; -0,50; 0,50) m, X baixo / Y trás / Z esquerda · `gps` (antena GPS-702-GG, tejadilho): (-1,00; 0; 1,30) m. `base_link` = centro das 4 rodas, 10 cm acima dos eixos (Bouça Nova, 2018, fig. 3.6) |
| **Driver ROS 2** | `ros-jazzy-novatel-oem7-driver` (com overrides para firmware OEM6) · `ros-jazzy-ros2-socketcan` |
| **Saídas** | `/novatel/oem7/*`, `/can/raw`, `/can/steering_angle`, `/g07/objetivo` (parte 2), `/g07/caminho_local`, `/g07/estado` |
| **Equipa** | João Ferreira (113571), Carlos Laranjeira (113524) |

## Arquitetura

| Serviço (compose) | Nó | Subscreve | Publica |
|---|---|---|---|
| `driver` | `novatel_oem7_driver` | | `/novatel/oem7/inspva`, `/novatel/oem7/bestpos`, `/novatel/oem7/rawimusx`, ... |
| `tf` | `static_transform_publisher` ×2 | | `/tf_static`: `base_link` → `imu_link`, `base_link` → `gps` |
| `can` | `socketcan_receiver` + `can_direcao.py` | `can0` | `/can/raw` (can_msgs/Frame), `/can/steering_angle` (ackermann_msgs/AckermannDriveStamped) |
| `rota` | `rota_objetivo` (**nó do G7**) | `/novatel/oem7/inspva` + rota (CSV lat,lon) | `/g07/objetivo` (geometry_msgs/PoseStamped), `/g07/rota_local` (nav_msgs/Path), `/g07/grelha_local` (toda -1) |
| `percecao` | `scan_to_grid` (exemplo do modelo, só com o simulador) | `/scan`, `/tf_static` | `/g07/grelha_local` (nav_msgs/OccupancyGrid) |
| `planeador` | `local_astar` (comum, `src/va_planeamento`) | `/g07/grelha_local`, `/g07/objetivo` | `/g07/caminho_local` (nav_msgs/Path), `/g07/estado` (std_msgs/String), `/g07/zona_proibida` |

## Estrutura

| Ficheiro | Função |
|---|---|
| `Dockerfile` | Imagem: ROS 2 Jazzy + driver NovAtel + SocketCAN + MCAP + Foxglove + workspace `src/` |
| `compose.yaml` / `compose.windows.yaml` | Serviços (Linux / Windows). Profiles: `sensor`, `simulado`, `reproduzir`, `gravar`, `foxglove`, `rviz` |
| `.env` | Configuração: grupo, ROS_DOMAIN_ID, percurso, recetor, CAN, bag a reproduzir |
| `docker/` | Scripts que correm dentro do contentor (driver, CAN, gravação, verificação, deteção da porta) |
| `src/va_planeamento/` | **Comum** (do modelo, sem alterações): grelha local + A* |
| `src/va_g07/` | **Do grupo**: simulador e nó de perceção do modelo (por agora), parâmetros em `config/params.yaml` |
| `tools/` | `figura_bag.py` (figuras e GIF a partir de um bag) |
| `ler_bag.sh` / `ler_bag.py` | Análise off-line sem ROS: resumo, trajetória, atitude, CSV |
| `ler_can.sh` / `ler_can.py` | CAN off-line sem ROS: IDs gravados e ângulo de direção descodificado de `/can/raw` (CSV + PNG) |
| `data/` | Bags MCAP (na aquisição real, apontar `DATA_DIR` para o SSD externo) |

## Como correr

Requisitos: Docker (Ubuntu 24.04 com Docker Engine; Windows com Docker Desktop, sem sensores).

```bash
git clone https://github.com/j9126f/va_gnss_odometry.git && cd va_gnss_odometry
docker compose build                                  # uma vez (demora alguns minutos)
docker compose --profile simulado up                  # sem sensor: LiDAR simulado + perceção + planeador
BAG=<pasta_em_data> docker compose --profile reproduzir up   # com um bag gravado
```

Windows (PowerShell): `$env:COMPOSE_FILE = "compose.windows.yaml"` antes dos comandos acima.
Visualização: `--profile foxglove` e, no Lichtblick/Foxglove, `ws://localhost:8765`.

### Aquisição no carro

```bash
docker compose run --rm driver detectar_recetor.py    # só se a deteção automática falhar
docker compose up driver                              # só o driver (para verificar)
docker compose exec driver verificar.sh               # (outro terminal) confirma que há dados
docker compose --profile sensor --profile gravar up   # driver + CAN + gravação (Ctrl+C para parar)
./ler_bag.sh data/<pasta_do_bag>                      # análise
sudo chown -R $USER data                              # se precisares de mover/apagar bags
```

Parâmetros que dependem do carro (`.env`): `OEM7_DEVICE` / `OEM7_IP`, `OEM7_BAUD`, `CAN_IF`,
`CAN_BITRATE`, `CAN_DIRECAO_*`, `CAN_RELACAO_DIRECAO`.

## Nó do grupo (especificação, Ex. 4 da aula 10)

O G7 não deteta obstáculos: a contribuição para a decisão local é o **objetivo** do planeador, o ponto da
rota global (aula 9) que está à frente do carro, no `base_link`.

| | Grupo 07 |
|---|---|
| **Subscreve** | `/novatel/oem7/inspva` (`novatel_oem7_msgs/INSPVA`, frame `gps`, 50 Hz, RELIABLE): latitude, longitude, azimute. Rota global: lista de waypoints (lat, lon) da aula 9 (parâmetro `ficheiro_rota`). Parte 2: `/can/steering_angle` (BEST_EFFORT) para odometria quando o GNSS falha |
| **Ocupado (100)** | Nada: o GNSS/INS só mede a pose do próprio carro, não vê o que o rodeia |
| **Livre (0)** | Nada |
| **Desconhecido (-1)** | A grelha toda. É publicada (mesma geometria do contrato: 0,2 m, x -5…35 m, y -10…10 m) para o planeador correr sozinho com o objetivo do G7; na parte 2 a grelha vem dos grupos de perceção |
| **Nó e ficheiro** | `rota_objetivo`, em `src/va_g07/va_g07/rota_objetivo.py` (cálculos sem ROS em `rota.py`, testes em `src/va_g07/test/`; entrada em `setup.py`; serviço `rota` do `compose.yaml`) |
| **Publica** | `/g07/objetivo` (`geometry_msgs/PoseStamped`, `base_link`): ponto da rota a ~25 m à frente, ao longo da rota (numa curva apertada, o último ponto ainda dentro da grelha) · `/g07/grelha_local` (`nav_msgs/OccupancyGrid`, `base_link`, toda -1) · `/g07/rota_local` (`nav_msgs/Path`, `base_link`): troço da rota à frente, para visualizar |

Cálculo do objetivo: waypoint mais próximo do carro → avançar 25 m ao longo da rota → (lat, lon) para
(Este, Norte) local à volta da posição atual → rodar pelo azimute do INSPVA (`x = dE·sin ψ + dN·cos ψ`,
`y = −dE·cos ψ + dN·sin ψ`, ψ medido de Norte para Este). `header.stamp` = o do INSPVA que lhe deu origem.

**Rota.** A aula 9 não define uma rota GNSS; usa-se um CSV `lat,lon` (qualquer origem: percurso gravado,
OSRM/OpenStreetMap, Google Directions como na dissertação de Bouça Nova, GPX convertido). A rota por
omissão, `src/va_g07/rotas/teste1.csv`, é a trajetória do INSPVA do bag `teste1` ("teach and repeat":
1192 waypoints a cada 2 m, 2,47 km de ida e volta). Para criar outra a partir de um bag:

```bash
docker compose run --rm planeador ros2 run va_g07 rota_de_bag /data/<bag> /data/rota.csv
cp data/rota.csv src/va_g07/rotas/<nome>.csv     # e ficheiro_rota em config/params.yaml
```

Testar com o bag (o INS só tem solução a partir de ~90 s; `--start-offset` salta o alinhamento):

```bash
docker compose up -d tf rota planeador
docker compose run --rm reprodutor ros2 bag play /data/g00_gnss-ins_20261113_140141_teste1 \
    --start-offset 150 --topics /novatel/oem7/inspva
docker compose logs -f rota planeador                # outro terminal
```

## Tópicos do sensor

Medido com `ros2 topic hz`, `ros2 topic echo --field header` e `ros2 topic info -v` a reproduzir os bags
`g00_gnss-ins_20261113_140141_teste1` e `g00_can_20261118_161425_exterior1` (Ex. 3 da aula 10).

| Tópico | Tipo | frame_id | Frequência | QoS (reliability) |
|---|---|---|---|---|
| `/novatel/oem7/inspva` | `novatel_oem7_msgs/msg/INSPVA` | `gps` | 50 Hz | RELIABLE |
| `/novatel/oem7/fix` | `sensor_msgs/msg/NavSatFix` | `gps` | 50 Hz | RELIABLE |
| `/novatel/oem7/odom` | `nav_msgs/msg/Odometry` | `odom` | 50 Hz | RELIABLE |
| `/novatel/oem7/bestpos` | `novatel_oem7_msgs/msg/BESTPOS` | `gps` | 10 Hz | RELIABLE |
| `/novatel/oem7/bestvel` | `novatel_oem7_msgs/msg/BESTVEL` | `gps` | 10 Hz | RELIABLE |
| `/novatel/oem7/rawimusx` | `novatel_oem7_msgs/msg/RAWIMUSX` | `imu_link` | 200 Hz ¹ | RELIABLE |
| `/novatel/oem7/inspvax` | `novatel_oem7_msgs/msg/INSPVAX` | `gps` | 1 Hz | RELIABLE |
| `/can/raw` | `can_msgs/msg/Frame` | `can` | ~1600 Hz (todas as tramas) | RELIABLE |
| `/can/steering_angle` | `ackermann_msgs/msg/AckermannDriveStamped` | `base_link` | 100 Hz | BEST_EFFORT |

¹ 105 007 mensagens em 525 s; o `ros2 topic hz` mede só ~175 Hz porque não acompanha esta taxa.
Os bags gravados até 2026-11-18 **não têm `/tf_static`**. O serviço `tf` (sempre ligado) publica agora
`base_link` → `imu_link` e `base_link` → `gps` com a instalação da dissertação de P. Bouça Nova (2018,
fig. 3.6 e tab. 3.3: `SETIMUTOANTOFFSET -0.80 -0.30 0.50`, `SETINSOFFSET 0.5 -1.3 0.5`), e o
`gravar.sh` grava-as nos bags seguintes. Ao reproduzir bags antigos, o serviço `tf` fornece-as.

## Dados

| Bag | Data | Percurso | Duração | Tópicos |
|---|---|---|---|---|
| `g00_gnss-ins_20261113_135159_teste` | 2026-11-13 | interior (sem fix GNSS) | 30 s | `/novatel/oem7/*` |
| `g00_gnss-ins_20261113_140141_teste1` | 2026-11-13 | exterior, 1.ª volta | 8,7 min | `/novatel/oem7/*` |
| `g00_can_20261118_161425_exterior1` | 2026-11-18 | exterior | 41,5 s | `/can/raw`, `/can/steering_angle` |

Os bags gravados antes da aula 10 têm o prefixo `g00` (o grupo ainda não estava definido).

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
