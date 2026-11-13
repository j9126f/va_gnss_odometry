"""Interpretação off-line do MCAP gravado no carro (sem ROS): localização por odómetro + CAN, GNSS e Kalman.
pip install mcap mcap-ros2-support matplotlib numpy

Usa SÓ o que o carro grava (o mesmo que o gerar_dados.py simula):
  - CAN bus: velocidade das rodas (odómetro) e ângulo de viragem das rodas da frente (rotação)
  - NovAtel GNSS (OEM615): posição (e velocidade) da antena
  - NovAtel SPAN-IGM-S1 (firmware OEM6, IMU ADIS16488): IMU em bruto (RAWIMUSX) e orientação do INS (INSPVA)
    Com firmware OEM6 o driver ROS 2 não publica sensor_msgs/Imu: o IMU vem em contagens no
    /novatel/oem7/rawimusx e é convertido aqui (IMU_ESCALA_*, EIXOS_IMU).
Não há ground truth: a qualidade avalia-se pela incerteza do Kalman e pelas inovações (GNSS - previsão).

ANTES DO 1.º MCAP REAL: o programa começa por listar os tópicos e tipos do ficheiro. Confirma os nomes em
TOPICO_*, as unidades / campos nas funções ler_* (secção "ADAPTAÇÃO AO CARRO") e as medidas do carro
(WHEEL_BASE, BRACO_ANTENA, RELACAO_DIRECAO).

Cada gráfico abre numa janela própria (roda do rato = zoom):
Janela 1: trajetória GNSS em lat/lon (interrompida nas falhas de GNSS).
Janela 2: sobreposição em metros (x = Este, y = Norte): posições GNSS, integração da velocidade GNSS,
          odómetro + CAN, navegação inercial (IMU) e filtro de Kalman.
Janelas 3 e 4: acelerómetro e giroscópio do SPAN.
Janela 5: inovações do Kalman (GNSS - previsão) com o intervalo ±3σ esperado -> verifica se o filtro está
          bem afinado (sem ground truth, é a forma de validar com dados reais).
Janela 6: incerteza do Kalman, offset do ângulo da roda e escala do odómetro estimados.
Janela 7: localização final - odómetro + CAN, apenas GNSS e filtro de Kalman.

Orientação: a ROTAÇÃO do trajeto vem do ângulo da roda (CAN bus, modelo bicicleta dθ = ds·tan(δ)/L).
O CAN só dá variações de rumo; o rumo ABSOLUTO vem do INS (SPAN), a partir do momento em que este alinha.
O GNSS não orienta a odometria (só em último recurso, se o ficheiro não tiver orientação do INS).
Todas as trajetórias em metros são desenhadas no ponto da ANTENA GNSS, para serem comparáveis com o GNSS.
"""
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mcap.exceptions import RecordLengthLimitExceeded
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages
from mcap_ros2.writer import Writer

# ##########################################################################################
#                                 ADAPTAÇÃO AO CARRO
# ##########################################################################################
# ------------------------------------- tópicos --------------------------------------------
TOPICO_RODAS = '/can/wheel_speeds'              # CAN: velocidade das rodas
TOPICO_DIRECAO = '/can/steering_angle'          # CAN: ângulo da roda
TOPICO_GNSS = '/novatel/oem7/fix'               # NovAtel: sensor_msgs/NavSatFix
TOPICO_GNSS_VEL = '/novatel/oem7/bestvel'       # NovAtel: BESTVEL (velocidade horizontal + rumo), opcional
TOPICO_IMU = '/novatel/oem7/rawimusx'           # NovAtel SPAN: RAWIMUSX (IMU em bruto, contagens, 200 Hz)
TOPICO_INS = '/novatel/oem7/inspva'             # NovAtel SPAN: INSPVA (posição, velocidade e atitude do INS)
TOPICO_INS_DESVIO = '/novatel/oem7/inspvax'     # NovAtel SPAN: INSPVAX (desvios-padrão do INS, 1 Hz), opcional

# --------------------------------- IMU (SPAN-IGM-S1) --------------------------------------
# ADIS16488: o RAWIMUSX dá incrementos por amostra (acel 200/2^31 m/s, giro 720/2^31 graus) a 200 Hz.
IMU_TAXA = 200.0                                           # Hz
IMU_ESCALA_ACEL = 200 / 2 ** 31 * IMU_TAXA                 # contagem -> m/s²
IMU_ESCALA_GIRO = math.radians(720 / 2 ** 31) * IMU_TAXA   # contagem -> rad/s
# Eixos do VEÍCULO (x frente, y esquerda, z cima) a partir dos eixos do IMU: {eixo do veículo: (eixo do IMU, sinal)}
#  - cima = -x do IMU: medido com o carro parado (gravidade em -x; SETIMUORIENTATION 2 = X do IMU para baixo)
#  - frente/esquerda: POR CONFIRMAR na 1.ª volta. O programa compara com o GNSS e o INS e avisa se estiver errado.
EIXOS_IMU = {'x': ('y', +1), 'y': ('z', -1), 'z': ('x', -1)}
# InertialSolutionStatus aceites: 3 = INS_SOLUTION_GOOD, 7 = INS_ALIGNMENT_COMPLETE (já alinhado, ainda a
# convergir; o SPAN pode ficar aqui bastante tempo). Em ambos o rumo só é usado se o σ do INSPVAX for
# menor que SIGMA_RUMO_INS_MAX.
INS_ESTADOS_VALIDOS = (3, 7)

# ------------------------------------- veículo --------------------------------------------
WHEEL_BASE = 2.7              # distância entre eixos (m)
BRACO_ANTENA = (1.2, 0.0)     # antena GNSS em relação ao centro do eixo traseiro (m: x frente, y esquerda)
RODAS_ODOMETRO = ('rl', 'rr')  # rodas usadas no odómetro (traseiras: o modelo bicicleta refere-se a esse eixo)
FATOR_VEL_RODAS = 1 / 3.6     # unidade da velocidade das rodas no CAN -> m/s (km/h -> m/s)
RELACAO_DIRECAO = 1.0         # se o CAN der o ângulo do VOLANTE, pôr a desmultiplicação da direção (ex.: 15)
USAR_TEMPO_HEADER = True      # tempo de cada medição = header.stamp (senão o log_time da gravação)


def ler_velocidades_rodas(msg):
    """Mensagem do TOPICO_RODAS -> {nome da roda: velocidade (m/s)}. Adaptar ao formato do carro."""
    return {nome: vel * FATOR_VEL_RODAS for nome, vel in zip(msg.name, msg.velocity)}


def ler_angulo_roda(msg):
    """Mensagem do TOPICO_DIRECAO -> ângulo de viragem das rodas da frente (rad, positivo = esquerda)."""
    return msg.drive.steering_angle / RELACAO_DIRECAO


# ##########################################################################################
#                                     PROCESSAMENTO
# ##########################################################################################
R_TERRA = 6378137.0
DIST_RUMO_INICIAL = 15.0      # (só sem INS) metros iniciais do GNSS usados para estimar o rumo inicial
T_RUMO_INS = 1.0              # segundos de orientação do INS (depois de alinhado) usados para o rumo inicial
SIGMA_RUMO_INS_MAX = math.radians(2.0)   # orientação do INS só é usada se σ do rumo for menor que isto
FALHA_GNSS_MIN_S = 1.0        # intervalo sem /fix válido a partir do qual se considera falha de GNSS

# ----------------------------- parâmetros do filtro de Kalman (EKF) -----------------------------
# Ruído de processo (quanto se confia no modelo de movimento / nos sensores de previsão)
EKF_SIGMA_V = 0.02          # erro aleatório da velocidade medida pelo odómetro (m/s)
EKF_SIGMA_DELTA = math.radians(0.05)   # erro do ângulo da roda lido do CAN (rad)
EKF_SIGMA_OFFSET = 1e-7     # variação do offset do ângulo da roda (rad por √s)
EKF_SIGMA_ESCALA = 1e-5     # variação do fator de escala do odómetro (por √s)
# (só sem ângulo da roda no CAN: rotação pelo giroscópio)
EKF_SIGMA_W = 0.001         # erro da velocidade angular do giroscópio (rad/s)
EKF_SIGMA_BIAS = 1e-6       # variação do bias do giroscópio (rad/s por √s)
# Ruído de medida: usa-se a covariância que o recetor NovAtel põe no /fix; se não houver, este valor
EKF_SIGMA_GPS = 1.0         # (m)
# Incerteza inicial: σ de x (m), y (m), rumo (rad), offset do ângulo da roda (rad), fator de escala
EKF_P0 = (0.05, 0.05, math.radians(1), math.radians(0.5), 0.01)
# Rejeição de outliers do GNSS (multipath): NIS = inovaçãoᵀ·S⁻¹·inovação > limiar (χ² com 2 g.l.: 13.8 = 0.1 %)
EKF_LIMIAR_NIS = 13.8
EKF_REJEICAO_MAX_S = 3.0    # se o GNSS for rejeitado seguido durante mais do que isto, volta a ser aceite

# ------------------------- exportação dos resultados para o Foxglove -------------------------
# Grava 2 MCAP ao lado do ficheiro lido: <nome>_odometro.mcap e <nome>_kalman.mcap, com a trajetória
# (no ponto da antena) como sensor_msgs/NavSatFix -> aparecem no painel Map do Foxglove.
# Abre-os juntamente com o MCAP original (arrastar os ficheiros juntos): os tempos são os mesmos.
EXPORTAR_MCAP = True
SUFIXOS_EXPORTADOS = ('_odometro', '_kalman')     # estes ficheiros nunca são lidos como dados de entrada
TOPICO_SAIDA_ODO = '/localizacao/odometro_can'
TOPICO_SAIDA_EKF = '/localizacao/kalman'

# ------------------------------------- ficheiro -------------------------------------------
if len(sys.argv) > 1:
    bag_path = Path(sys.argv[1])
else:
    bag_path = Path("C:\\Users\\asus\\Desktop\\VA_Prático\\Dados_recolhidos")
    print(f"Nenhum caminho indicado na linha de comandos. A utilizar o caminho predefinido: {bag_path}")
if bag_path.is_dir():
    mcaps = sorted((p for p in bag_path.glob('*.mcap') if not p.stem.endswith(SUFIXOS_EXPORTADOS)),
                   key=lambda p: p.stat().st_mtime)
    if not mcaps:
        sys.exit(f"Não há ficheiros .mcap em {bag_path}")
    mcap_file = mcaps[-1]
else:
    mcap_file = bag_path
print(f"A ler o ficheiro: {mcap_file}")

# ------------------------------ tópicos que o ficheiro tem --------------------------------
ESPERADOS = {TOPICO_RODAS: 'velocidade das rodas (CAN)', TOPICO_DIRECAO: 'ângulo da roda (CAN)',
             TOPICO_GNSS: 'GNSS NovAtel', TOPICO_GNSS_VEL: 'velocidade GNSS (opcional)',
             TOPICO_IMU: 'SPAN: IMU em bruto', TOPICO_INS: 'SPAN: atitude do INS',
             TOPICO_INS_DESVIO: 'SPAN: desvios-padrão do INS (opcional)'}
try:
    with open(mcap_file, 'rb') as f:
        resumo = make_reader(f).get_summary()
    if resumo is not None:
        contagens = resumo.statistics.channel_message_counts if resumo.statistics else {}
        print('Tópicos no ficheiro:')
        presentes = set()
        for cid, ch in sorted(resumo.channels.items(), key=lambda c: c[1].topic):
            tipo = resumo.schemas[ch.schema_id].name if ch.schema_id in resumo.schemas else '?'
            marca = f'  <- {ESPERADOS[ch.topic]}' if ch.topic in ESPERADOS else ''
            print(f'   {ch.topic:40s} {tipo:45s} {contagens.get(cid, 0):8d}{marca}')
            presentes.add(ch.topic)
        for topico, desc in ESPERADOS.items():
            if topico not in presentes:
                print(f'AVISO: falta o tópico {topico} ({desc}); corrige TOPICO_* se tiver outro nome')
except Exception as e:                       # o resumo é só informativo
    print(f'(não foi possível ler o resumo do ficheiro: {e})')

# ----------------------------------------- leitura ----------------------------------------
tempos_fix, latitudes, longitudes, var_fix, status_fix = [], [], [], [], []
tempos_vel, vel_e, vel_n = [], [], []
tempos_rodas, vel_odo = [], []          # velocidade do odómetro (m/s) = média das RODAS_ODOMETRO
tempos_dir, can_delta = [], []          # ângulo da roda (rad)
tempos_imu, acel, gyro = [], [], []     # acel = [ax, ay, az] (m/s²), gyro = [wx, wy, wz] (rad/s)
tempos_yaw, ins_yaw = [], []            # rumo do INS (rad, ENU: 0 = Este), só quando é válido
tempos_sig_rumo, sig_rumo = [], []      # σ do rumo do INS (rad), do INSPVAX
n_sem_fix = 0
avisos_rodas = set()


def tempo(m, msg):
    """Instante da medição (s): header.stamp do driver ou, se não houver, o instante de gravação."""
    if USAR_TEMPO_HEADER and hasattr(msg, 'header'):
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        if t > 0:
            return t
    return m.log_time_ns * 1e-9


try:
    for m in read_ros2_messages(str(mcap_file), topics=list(ESPERADOS)):
        topico, msg = m.channel.topic, m.ros_msg
        t = tempo(m, msg)
        if topico == TOPICO_GNSS:
            # sem solução (parque subterrâneo, túnel): status -1 (NO_FIX) ou coordenadas inválidas
            if msg.status.status < 0 or not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
                n_sem_fix += 1
                continue
            tempos_fix.append(t)
            latitudes.append(msg.latitude)
            longitudes.append(msg.longitude)
            status_fix.append(msg.status.status)
            cov = msg.position_covariance
            var_fix.append(max(cov[0], cov[4]) if msg.position_covariance_type > 0 and cov[0] > 0
                           else EKF_SIGMA_GPS ** 2)
        elif topico == TOPICO_GNSS_VEL:
            if msg.sol_status.status != 0:          # 0 = SOL_COMPUTED
                continue
            trk = math.radians(msg.trk_gnd)          # rumo sobre o solo, graus a partir do Norte (sentido horário)
            tempos_vel.append(t)
            vel_e.append(msg.hor_speed * math.sin(trk))
            vel_n.append(msg.hor_speed * math.cos(trk))
        elif topico == TOPICO_RODAS:
            rodas = ler_velocidades_rodas(msg)
            faltam = [r for r in RODAS_ODOMETRO if r not in rodas]
            if faltam:
                avisos_rodas.update(faltam)
                continue
            tempos_rodas.append(t)
            vel_odo.append(float(np.mean([rodas[r] for r in RODAS_ODOMETRO])))
        elif topico == TOPICO_DIRECAO:
            tempos_dir.append(t)
            can_delta.append(ler_angulo_roda(msg))
        elif topico == TOPICO_IMU:
            # RAWIMUSX traz -Y (y_acc, y_gyro com sinal trocado), por isso troca-se o sinal aqui
            bruto_a = {'x': msg.x_acc, 'y': -msg.y_acc, 'z': msg.z_acc}
            bruto_g = {'x': msg.x_gyro, 'y': -msg.y_gyro, 'z': msg.z_gyro}
            tempos_imu.append(t)
            acel.append(tuple(sn * bruto_a[e] * IMU_ESCALA_ACEL for e, sn in (EIXOS_IMU[k] for k in 'xyz')))
            gyro.append(tuple(sn * bruto_g[e] * IMU_ESCALA_GIRO for e, sn in (EIXOS_IMU[k] for k in 'xyz')))
        elif topico == TOPICO_INS:
            # antes de alinhar (WAITING_INITIAL_POS, ALIGNING, ...) a orientação não é válida
            if msg.status.status in INS_ESTADOS_VALIDOS:
                tempos_yaw.append(t)
                # azimute NovAtel: graus a partir do Norte, sentido horário -> rumo ENU (0 = Este, anti-horário)
                ins_yaw.append(math.atan2(math.cos(math.radians(msg.azimuth)), math.sin(math.radians(msg.azimuth))))
        elif topico == TOPICO_INS_DESVIO:
            tempos_sig_rumo.append(t)
            sig_rumo.append(math.radians(msg.azimuth_stdev))
except RecordLengthLimitExceeded as e:
    print(f"\n[ERRO] O leitor detetou um registo inválido ou corrompido no ficheiro MCAP: {e}")
    print("Sugestão: o ficheiro pode estar incompleto (gravação interrompida). Tenta 'mcap recover'.")
    sys.exit(1)
if avisos_rodas:
    print(f"AVISO: o {TOPICO_RODAS} não tem as rodas {sorted(avisos_rodas)}; corrige RODAS_ODOMETRO")


def ordenar(t, *series):
    """Ordena pelo tempo da medição (a gravação vem por ordem de chegada, que pode diferir do header.stamp)."""
    if not t:
        return [np.asarray(t, dtype=float)] + [np.asarray(s) for s in series]
    k = np.argsort(t, kind='stable')
    return [np.asarray(t, dtype=float)[k]] + [np.asarray(s)[k] for s in series]


tempos_fix, latitudes, longitudes, var_fix, status_fix = ordenar(tempos_fix, latitudes, longitudes, var_fix,
                                                                 status_fix)
tempos_vel, vel_e, vel_n = ordenar(tempos_vel, vel_e, vel_n)
tempos_rodas, vel_odo = ordenar(tempos_rodas, vel_odo)
tempos_dir, can_delta = ordenar(tempos_dir, can_delta)
tempos_imu, acel, gyro = ordenar(tempos_imu, acel, gyro)
tempos_yaw, ins_yaw = ordenar(tempos_yaw, ins_yaw)
tempos_sig_rumo, sig_rumo = ordenar(tempos_sig_rumo, sig_rumo)
if len(tempos_yaw) and len(tempos_sig_rumo):
    # rumo do INS só é usado se o último σ do rumo (INSPVAX, 1 Hz) for pequeno; antes do 1.º σ não é usado
    k = np.searchsorted(tempos_sig_rumo, tempos_yaw, side='right') - 1
    fiavel = (k >= 0) & (sig_rumo[np.clip(k, 0, None)] <= SIGMA_RUMO_INS_MAX)
    tempos_yaw, ins_yaw = tempos_yaw[fiavel], ins_yaw[fiavel]

print('Leitura concluída:')
print(f' - GNSS ({TOPICO_GNSS}): {len(latitudes)} posições válidas, {n_sem_fix} sem solução (NO_FIX)')
if len(status_fix):
    sig = np.sqrt(var_fix)
    print(f'      precisão reportada pelo recetor (σ por eixo): mediana {np.median(sig) * 100:.1f} cm, '
          f'máx. {np.max(sig) * 100:.1f} cm; status: '
          + ', '.join(f'{s}: {np.sum(status_fix == s)}' for s in np.unique(status_fix)))
print(f' - Velocidade GNSS ({TOPICO_GNSS_VEL}): {len(vel_e)}')
print(f' - Velocidade das rodas ({TOPICO_RODAS}): {len(vel_odo)}')
print(f' - Ângulo da roda ({TOPICO_DIRECAO}): {len(can_delta)}')
print(f' - IMU ({TOPICO_IMU}): {len(acel)}')
print(f' - Orientação válida do INS ({TOPICO_INS}): {len(ins_yaw)}')
if len(acel):
    print(f'      IMU médio (eixos do veículo): acel {np.round(acel.mean(0), 2)} m/s² '
          f'(parado: ~[0, 0, 9.8]), giro {np.round(np.degrees(gyro.mean(0)), 3)} °/s')


def verificar_eixos_imu():
    """Confirma EIXOS_IMU com dados em movimento: aceleração para a frente vs derivada da velocidade GNSS
    e giroscópio z vs derivada do rumo do INS (correlação deve ser claramente positiva)."""
    if len(acel) < 100:
        return
    if len(tempos_vel) > 50:
        vel = np.hypot(vel_e, vel_n)
        if np.ptp(vel) > 2.0:                        # houve acelerações a sério (> 2 m/s de variação)
            dv = np.gradient(vel, tempos_vel)
            n = max(1, int(IMU_TAXA / 5))            # média móvel de 0.2 s para tirar ruído
            a_frente = np.convolve(acel[:, 0], np.ones(n) / n, mode='same')
            r = np.corrcoef(dv, np.interp(tempos_vel, tempos_imu, a_frente))[0, 1]
            estado = 'OK' if r > 0.5 else 'AVISO: verificar o eixo/sinal "x" em EIXOS_IMU'
            print(f'   eixo da frente do IMU vs GNSS: correlação {r:+.2f} -> {estado}')
    if len(tempos_yaw) > 50:
        rumo = np.unwrap(ins_yaw)
        if np.ptp(rumo) > math.radians(30):         # houve curvas
            w_ins = np.gradient(rumo, tempos_yaw)
            r = np.corrcoef(w_ins, np.interp(tempos_yaw, tempos_imu, gyro[:, 2]))[0, 1]
            estado = 'OK' if r > 0.5 else 'AVISO: verificar o eixo/sinal "z" em EIXOS_IMU'
            print(f'   giroscópio z vs rumo do INS: correlação {r:+.2f} -> {estado}')


verificar_eixos_imu()


# ----------------------------------- funções auxiliares -----------------------------------
def latlon_para_local(lat, lon, lat0, lon0):
    """Equivalente ao latlon2local do MATLAB (aproximação local plana): x = Este, y = Norte (m)."""
    lat, lon = np.asarray(lat), np.asarray(lon)
    x = np.radians(lon - lon0) * R_TERRA * math.cos(math.radians(lat0))
    y = np.radians(lat - lat0) * R_TERRA
    return x, y


def na_antena(x, y, th):
    """Posição da antena GNSS a partir da pose do centro do eixo traseiro."""
    a, b = BRACO_ANTENA
    return x + a * np.cos(th) - b * np.sin(th), y + a * np.sin(th) + b * np.cos(th)


def media_angular(a):
    a = np.asarray(a)
    return math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a)))


def rumo_inicial(x, y, dist=DIST_RUMO_INICIAL):
    """(Só usado se o ficheiro não tiver orientação do INS.) Rumo (rad) a partir dos primeiros pontos GNSS
    (Ex. 4 b/c do enunciado): reta por polyfit nos primeiros `dist` metros percorridos, arco-tangente do
    declive e quadrante pelo sinal do deslocamento médio."""
    x, y = np.asarray(x) - x[0], np.asarray(y) - y[0]
    d = np.hypot(x, y)
    n = int(np.argmax(d > dist)) + 1 if np.any(d > dist) else len(x)
    n = max(n, 3)
    xs, ys = x[:n], y[:n]
    if np.ptp(xs) >= np.ptp(ys):            # reta y = a·x + b
        a, _ = np.polyfit(xs, ys, 1)
        ang = math.atan(a)
        if np.mean(xs) < 0:
            ang += math.pi
    else:                                   # reta quase vertical: ajusta x = a·y + b
        a, _ = np.polyfit(ys, xs, 1)
        ang = math.pi / 2 - math.atan(a)
        if np.mean(ys) < 0:
            ang += math.pi
    return math.atan2(math.sin(ang), math.cos(ang))


def filtro_kalman(t, v, u, t_gps, gx, gy, var_gps, th0, modo='can'):
    """Filtro de Kalman estendido (EKF) para localização 2D do centro do eixo traseiro.

    Estado:   X = [x, y, θ, b, k]
              posição (m), rumo (rad), b = offset do ângulo da roda do CAN (rad) [modo 'can'] ou bias do
              giroscópio (rad/s) [modo 'giro'], k = fator de escala do odómetro (raio real das rodas)
    Previsão (a cada amostra do odómetro, modelo bicicleta):
        x' = x + k·v·cos(θ)·dt      v = velocidade do odómetro (CAN)
        y' = y + k·v·sin(θ)·dt      u = ângulo da roda δ (CAN)          ω = k·v·tan(δ - b)/L   [modo 'can']
        θ' = θ + ω·dt               u = velocidade angular do giroscópio ω = u - b             [modo 'giro']
        b' = b,  k' = k             P' = F·P·Fᵀ + Q
    Correção (cada posição GNSS z, medida na ANTENA, a BRACO_ANTENA = (a, c) do eixo traseiro):
        h(X) = [x + a·cosθ - c·sinθ,  y + a·sinθ + c·cosθ]
        inovação = z - h(X)         S = H·P·Hᵀ + R  (R = covariância do /fix)      K = P·Hᵀ·S⁻¹
        X = X + K·inovação          P = (I - K·H)·P
        NIS = inovaçãoᵀ·S⁻¹·inovação; se > EKF_LIMIAR_NIS a posição é rejeitada (outlier / multipath)
    Começa no 1.º GNSS (posição) com o rumo th0. Devolve tempos, estados, desvios-padrão e as inovações."""
    a, c = BRACO_ANTENA
    X = np.array([gx[0] - a * math.cos(th0) + c * math.sin(th0),
                  gy[0] - a * math.sin(th0) - c * math.cos(th0), th0, 0.0, 1.0])
    P = np.diag(np.square(EKF_P0))
    P[0, 0] = P[1, 1] = var_gps[0] + EKF_P0[0] ** 2
    if modo != 'can':
        P[3, 3] = 0.001 ** 2                              # incerteza inicial do bias do giroscópio (rad/s)
    I5 = np.eye(5)
    i0 = int(np.searchsorted(t, t_gps[0]))
    k_gps = 1
    t_rejeicao = None                                    # início da sequência atual de rejeições

    def prever(X, P, vi, ui, dt):
        th, b, k = X[2], X[3], X[4]
        if modo == 'can':                                # rotação pelo ângulo da roda (CAN)
            tg = math.tan(ui - b)
            sec2 = 1 + tg ** 2
            dth = k * vi * tg / WHEEL_BASE * dt
            d_dth_db = -k * vi * sec2 / WHEEL_BASE * dt
            d_dth_dk = vi * tg / WHEEL_BASE * dt
            q_th = (k * vi * sec2 / WHEEL_BASE * EKF_SIGMA_DELTA * dt) ** 2
            q_b = EKF_SIGMA_OFFSET ** 2 * dt
        else:                                            # rotação pelo giroscópio
            dth = (ui - b) * dt
            d_dth_db, d_dth_dk = -dt, 0.0
            q_th = (EKF_SIGMA_W * dt) ** 2
            q_b = EKF_SIGMA_BIAS ** 2 * dt
        thm = th + dth / 2                               # rumo a meio do intervalo
        cs, sn = math.cos(thm), math.sin(thm)
        X = X + np.array([k * vi * cs * dt, k * vi * sn * dt, dth, 0.0, 0.0])
        F = np.array([[1, 0, -k * vi * sn * dt, 0, vi * cs * dt],
                      [0, 1, k * vi * cs * dt, 0, vi * sn * dt],
                      [0, 0, 1, d_dth_db, d_dth_dk],
                      [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])
        Q = np.diag([(EKF_SIGMA_V * dt) ** 2, (EKF_SIGMA_V * dt) ** 2, q_th, q_b, EKF_SIGMA_ESCALA ** 2 * dt])
        return X, F @ P @ F.T + Q

    tempos, estados, sigmas = [], [], []
    inov_t, inov_v, inov_s, inov_ok = [], [], [], []
    t_atual = t_gps[0]
    for i in range(max(i0, 1), len(t)):
        # a velocidade v[i] e o ângulo u[i] valem para o intervalo ]t[i-1], t[i]]
        t_atual = max(t_atual, t[i - 1])
        while k_gps < len(t_gps) and t_gps[k_gps] <= t[i]:
            # ---------- previsão até ao instante EXATO do GNSS ----------
            if t_gps[k_gps] > t_atual:
                X, P = prever(X, P, v[i], u[i], t_gps[k_gps] - t_atual)
                t_atual = t_gps[k_gps]
            # ---------- correção com a posição GNSS (na antena) ----------
            cs, sn = math.cos(X[2]), math.sin(X[2])
            hx = np.array([X[0] + a * cs - c * sn, X[1] + a * sn + c * cs])
            H = np.array([[1.0, 0, -a * sn - c * cs, 0, 0], [0, 1.0, a * cs - c * sn, 0, 0]])
            R = np.eye(2) * var_gps[k_gps]
            inov = np.array([gx[k_gps], gy[k_gps]]) - hx
            S = H @ P @ H.T + R
            S_inv = np.linalg.inv(S)
            nis = float(inov @ S_inv @ inov)
            aceite = nis <= EKF_LIMIAR_NIS
            if not aceite:
                t_rejeicao = t_gps[k_gps] if t_rejeicao is None else t_rejeicao
                aceite = t_gps[k_gps] - t_rejeicao > EKF_REJEICAO_MAX_S
            if aceite:
                t_rejeicao = None
                K = P @ H.T @ S_inv
                X = X + K @ inov
                P = (I5 - K @ H) @ P
            inov_t.append(t_gps[k_gps]); inov_v.append(inov); inov_s.append(np.sqrt(np.diag(S)))
            inov_ok.append(aceite)
            k_gps += 1
        # ---------- previsão do resto do intervalo ----------
        if t[i] > t_atual:
            X, P = prever(X, P, v[i], u[i], t[i] - t_atual)
            t_atual = t[i]
        tempos.append(t[i])
        estados.append(X.copy())
        sigmas.append(np.sqrt(np.diag(P)))
    inovacoes = (np.asarray(inov_t), np.asarray(inov_v).reshape(-1, 2), np.asarray(inov_s).reshape(-1, 2),
                 np.asarray(inov_ok, dtype=bool))
    return np.asarray(tempos), np.asarray(estados), np.asarray(sigmas), inovacoes


# --------------------------- conversões para o referencial local ---------------------------
tem_gps = len(latitudes) > 0
if tem_gps:
    lat0, lon0 = latitudes[0], longitudes[0]          # origem do referencial local = 1.º fix GNSS
    gps_x, gps_y = latlon_para_local(latitudes, longitudes, lat0, lon0)

# Períodos sem solução GNSS: intervalos entre posições válidas maiores que FALHA_GNSS_MIN_S
falhas_gnss = []
if tem_gps:
    for k in np.nonzero(np.diff(tempos_fix) > FALHA_GNSS_MIN_S)[0]:
        falhas_gnss.append((tempos_fix[k], tempos_fix[k + 1]))
    for ini, fim in falhas_gnss:
        print(f"Falha de GNSS: {fim - ini:.1f} s sem solução "
              f"(t = {ini - tempos_fix[0]:.1f} s a {fim - tempos_fix[0]:.1f} s desde o 1.º fix)")

# Integração da velocidade GNSS (Ex. 3): posição = soma de v·dt, a partir do 1.º ponto GNSS
int_x = int_y = None
if tem_gps and len(vel_e) > 1:
    dt = np.diff(tempos_vel, prepend=tempos_vel[0])
    dt[dt > FALHA_GNSS_MIN_S] = 0.0                   # durante uma falha não há velocidade
    int_x = gps_x[0] + np.cumsum(vel_e * dt)
    int_y = gps_y[0] + np.cumsum(vel_n * dt)

# ------------------- odómetro (distância) + rotação relativa (CAN ou giroscópio) -------------------
tem_odo = len(tempos_rodas) > 1
tem_can = tem_odo and len(can_delta) > 1
fonte_rot = None
if tem_odo:
    t_od = tempos_rodas
    dt_od = np.diff(t_od, prepend=t_od[0])
    ds = vel_odo * dt_od                              # distância em cada intervalo (odómetro)
    if tem_can:
        # ângulo da roda nos instantes do odómetro e média em cada intervalo
        delta_od = np.interp(t_od, tempos_dir, can_delta)
        u_pred = np.concatenate(([delta_od[0]], (delta_od[1:] + delta_od[:-1]) / 2))
        dth = ds * np.tan(u_pred) / WHEEL_BASE        # modelo bicicleta
        modo_rot, fonte_rot = 'can', f'ângulo da roda do CAN ({TOPICO_DIRECAO})'
    elif len(acel) > 1:
        # sem ângulo da roda: velocidade angular média do giroscópio em cada intervalo do odómetro
        ang_acum = np.concatenate(([0.0], np.cumsum(np.diff(tempos_imu) * gyro[:-1, 2])))
        ang_od = np.interp(t_od, tempos_imu, ang_acum)
        dth = np.diff(ang_od, prepend=ang_od[0])
        u_pred = dth / np.where(dt_od > 0, dt_od, 1.0)
        modo_rot, fonte_rot = 'giro', f'giroscópio ({TOPICO_IMU}) - AVISO: ficheiro sem ângulo da roda'
    else:
        tem_odo = False
        print('AVISO: sem ângulo da roda nem giroscópio: não é possível calcular a odometria')
if tem_odo:
    th_rel = np.cumsum(dth)                          # rumo relativo ao início da gravação

    # Rumo ABSOLUTO: o rumo do INS quando fica válido (alinhado), comparado com o rumo relativo do CAN
    # nesse mesmo intervalo, dá o desvio constante a somar ao rumo relativo (vale para trás e para a frente).
    desvio_rumo = fonte_rumo = None
    if len(ins_yaw):
        janela = tempos_yaw <= tempos_yaw[0] + T_RUMO_INS
        desvio_rumo = media_angular(ins_yaw[janela] - np.interp(tempos_yaw[janela], t_od, th_rel))
        fonte_rumo = f'INS ({TOPICO_INS}), alinhado {tempos_yaw[0] - t_od[0]:.1f} s após o início'
    elif tem_gps:
        k_ini = int(np.searchsorted(t_od, tempos_fix[0]))
        desvio_rumo = rumo_inicial(gps_x, gps_y) - th_rel[min(k_ini, len(th_rel) - 1)]
        fonte_rumo = 'GNSS - AVISO: ficheiro sem orientação válida do INS'
    if desvio_rumo is None:
        desvio_rumo, fonte_rumo = 0.0, 'nenhuma fonte (rumo inicial = 0, Este)'
    th_odo = th_rel + desvio_rumo
    print(f"Rumo no início da gravação: {math.degrees(math.atan2(math.sin(th_odo[0]), math.cos(th_odo[0]))):.2f}º "
          f"(ENU, 0 = Este), dado pelo {fonte_rumo}")
    print(f"Odómetro: {np.sum(np.abs(ds)):.0f} m percorridos em {t_od[-1] - t_od[0]:.0f} s; "
          f"rotação por {fonte_rot}")

    # Trajeto por ODÓMETRO + CAN (dead reckoning) do eixo traseiro, ancorado no 1.º fix GNSS
    thm = th_odo - dth / 2
    rx, ry = np.cumsum(ds * np.cos(thm)), np.cumsum(ds * np.sin(thm))
    odo_x = odo_y = None
    if tem_gps:
        th_f = np.interp(tempos_fix[0], t_od, th_odo)
        ax0, ay0 = na_antena(np.interp(tempos_fix[0], t_od, rx), np.interp(tempos_fix[0], t_od, ry), th_f)
        rx, ry = rx + gps_x[0] - ax0, ry + gps_y[0] - ay0
    odo_x, odo_y = na_antena(rx, ry, th_odo)          # desenhado no ponto da antena

# Navegação inercial (só IMU): rumo = rumo do INS no início + integral de wz; velocidade = integral de ax
# (assume-se que o carro começa parado). Sem correções, o erro cresce muito depressa (bias integrado 2×).
ine_x = ine_y = None
if len(acel) > 1 and tem_odo:
    dt = np.diff(tempos_imu, prepend=tempos_imu[0])
    rumo = np.interp(tempos_imu[0], t_od, th_odo) + np.cumsum(gyro[:, 2] * dt)
    vel = np.cumsum(acel[:, 0] * dt)
    ine_x = odo_x[0] + np.cumsum(vel * np.cos(rumo) * dt)
    ine_y = odo_y[0] + np.cumsum(vel * np.sin(rumo) * dt)

# Filtro de Kalman (EKF): previsão com o odómetro + CAN, correção com as posições GNSS
t_ekf = ekf = sig_ekf = inovacoes = None
if tem_odo and len(tempos_fix) > 1 and tempos_fix[0] < t_od[-1]:
    th0 = float(np.interp(tempos_fix[0], t_od, th_odo))
    t_ekf, ekf, sig_ekf, inovacoes = filtro_kalman(t_od, vel_odo, u_pred, tempos_fix, gps_x, gps_y, var_fix,
                                                   th0, modo_rot)
    ekf_ax, ekf_ay = na_antena(ekf[:, 0], ekf[:, 1], ekf[:, 2])
    print(f"EKF: previsão com odómetro (distância) + {fonte_rot} (rotação), "
          f"correção com {len(tempos_fix)} posições GNSS")
    txt_b = (f"offset do ângulo da roda: {math.degrees(ekf[-1, 3]):.4f}º" if modo_rot == 'can'
             else f"bias do giroscópio: {ekf[-1, 3]:.6f} rad/s")
    print(f"     estimado: {txt_b} | escala do odómetro: {ekf[-1, 4]:.5f} | "
          f"incerteza final: σx={sig_ekf[-1, 0] * 100:.1f} cm, σy={sig_ekf[-1, 1] * 100:.1f} cm, "
          f"σθ={math.degrees(sig_ekf[-1, 2]):.3f}º")
    ti, vi, si, oki = inovacoes
    nis = np.sum((vi / si) ** 2, axis=1)              # aproximação (ignora a correlação entre E e N)
    print(f"     inovações: RMS {np.sqrt(np.mean(np.sum(vi[oki] ** 2, axis=1))) * 100:.1f} cm; "
          f"{np.mean(nis <= 5.99) * 100:.0f} % com NIS ≤ 5.99 (ideal ≈ 95 %; muito menos -> Q/R pequenos demais; "
          f"perto de 100 % -> filtro pessimista ou erro GNSS correlacionado no tempo); "
          f"{np.sum(~oki)} posições GNSS rejeitadas")
    for ini, fim in falhas_gnss:
        k = int(np.searchsorted(ti, fim))
        if k < len(ti):
            print(f"     falha de {fim - ini:.0f} s: ao voltar o GNSS, a previsão (só odómetro + CAN) estava a "
                  f"{np.hypot(*vi[k]) * 100:.1f} cm do GNSS (σ da previsão {np.hypot(*si[k]) * 100:.1f} cm)")
elif tem_odo:
    print('AVISO: sem posições GNSS durante a odometria: o filtro de Kalman não corre')

# ------------------------- exportação para o Foxglove (2 ficheiros MCAP) -------------------------
_SEP = "=" * 80 + "\n"
MSGDEF_NAVSATFIX = (
    "std_msgs/Header header\nsensor_msgs/NavSatStatus status\n"
    "float64 latitude\nfloat64 longitude\nfloat64 altitude\n"
    "float64[9] position_covariance\nuint8 position_covariance_type\n"
    + _SEP + "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    + _SEP + "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
    + _SEP + "MSG: sensor_msgs/NavSatStatus\nint8 status\nuint16 service\n")


def exportar_trajetoria(caminho, topico, t, x, y, sig_x=None, sig_y=None):
    """Grava uma trajetória (m, referencial local com origem no 1.º fix GNSS) num MCAP como NavSatFix,
    com os mesmos instantes das medições (fica sincronizada com o MCAP original no Foxglove).
    sig_x, sig_y: desvio-padrão (m) de cada posição -> position_covariance (o Foxglove pode desenhá-la)."""
    lat = lat0 + np.degrees(np.asarray(y) / R_TERRA)
    lon = lon0 + np.degrees(np.asarray(x) / (R_TERRA * math.cos(math.radians(lat0))))
    with open(caminho, 'wb') as f:
        wr = Writer(f)
        esquema = wr.register_msgdef('sensor_msgs/msg/NavSatFix', MSGDEF_NAVSATFIX)
        for i in range(len(t)):
            ns = int(round(t[i] * 1e9))
            cov, tipo = [0.0] * 9, 0                     # 0 = covariância desconhecida
            if sig_x is not None:
                cov[0], cov[4], cov[8], tipo = float(sig_x[i]) ** 2, float(sig_y[i]) ** 2, 1e6, 2
            wr.write_message(topico, esquema, {
                'header': {'stamp': {'sec': ns // 10 ** 9, 'nanosec': ns % 10 ** 9}, 'frame_id': 'gps_link'},
                'status': {'status': 0, 'service': 0},
                'latitude': float(lat[i]), 'longitude': float(lon[i]),
                'altitude': 0.0,                             # só 2D: a altitude não é estimada
                'position_covariance': cov, 'position_covariance_type': tipo}, log_time=ns, publish_time=ns)
        wr.finish()
    print(f"Exportado: {caminho} ({topico}, {len(t)} posições)")


if EXPORTAR_MCAP and tem_gps:
    if tem_odo:
        exportar_trajetoria(mcap_file.with_name(mcap_file.stem + '_odometro.mcap'), TOPICO_SAIDA_ODO,
                            t_od, odo_x, odo_y)
    if ekf is not None:
        exportar_trajetoria(mcap_file.with_name(mcap_file.stem + '_kalman.mcap'), TOPICO_SAIDA_EKF,
                            t_ekf, ekf_ax, ekf_ay, sig_ekf[:, 0], sig_ekf[:, 1])

NOME_ODO = 'Odómetro + CAN' if tem_can else 'Odómetro + giroscópio'
NOME_GNSS = 'Apenas GNSS'
NOME_EKF = 'Filtro de Kalman (odómetro + CAN + GNSS)' if tem_can else 'Filtro de Kalman (EKF)'
t_base = min((s[0] for s in (tempos_fix, tempos_rodas, tempos_imu) if len(s)), default=0.0)

# ------------------------------------------ gráficos ------------------------------------------
# Cada gráfico abre na sua própria janela. Em todas: roda do rato = zoom à volta do cursor
# (botão "casa" da barra de ferramentas volta à vista inicial; o botão das setas cruzadas arrasta o gráfico).
FATOR_ZOOM = 1.25            # quanto cada clique da roda aproxima/afasta


def zoom_com_scroll(event):
    """Zoom com a roda do rato, centrado no ponto onde está o cursor (funciona em escalas lineares e log)."""
    if event.inaxes is None:
        return
    fator = 1 / FATOR_ZOOM if event.button == 'up' else FATOR_ZOOM
    feitos_x = []                                    # eixos com x partilhado (twinx) só fazem zoom em x uma vez
    for ax in event.canvas.figure.axes:
        if not ax.bbox.contains(event.x, event.y):
            continue
        xc, yc = ax.transData.inverted().transform((event.x, event.y))
        eixos = [('y', ax.get_ylim, ax.set_ylim, ax.get_yscale(), yc)]
        if not any(ax.get_shared_x_axes().joined(ax, outro) for outro in feitos_x):
            eixos.append(('x', ax.get_xlim, ax.set_xlim, ax.get_xscale(), xc))
            feitos_x.append(ax)
        for _, obter, definir, escala, c in eixos:
            a, b = obter()
            if escala == 'log':
                la, lb, lc = np.log10(a), np.log10(b), np.log10(c)
                definir(10 ** (lc + (la - lc) * fator), 10 ** (lc + (lb - lc) * fator))
            else:
                definir(c + (a - c) * fator, c + (b - c) * fator)
    event.canvas.draw_idle()


def nova_janela(titulo, tamanho=(9, 7)):
    """Cria uma figura (janela) com um único gráfico, dá-lhe o título indicado e ativa o zoom com a roda."""
    fig, ax = plt.subplots(figsize=tamanho)
    try:
        fig.canvas.manager.set_window_title(titulo)
    except AttributeError:          # backends sem janela (ex.: guardar em ficheiro)
        pass
    fig.canvas.mpl_connect('scroll_event', zoom_com_scroll)
    return fig, ax


def sem_dados(ax, texto):
    ax.text(0.5, 0.5, texto, ha='center', va='center', transform=ax.transAxes)


def com_quebras(t, *series):
    """Insere NaN onde há intervalos sem dados (> FALHA_GNSS_MIN_S), para a linha não ligar os dois lados."""
    t = np.asarray(t, dtype=float)
    q = np.nonzero(np.diff(t) > FALHA_GNSS_MIN_S)[0] + 1
    return [np.insert(t, q, np.nan)] + [np.insert(np.asarray(s, dtype=float), q, np.nan) for s in series]


def sombrear_falhas(ax, texto='Sem GNSS'):
    for j, (ini, fim) in enumerate(falhas_gnss):
        ax.axvspan(ini - t_base, fim - t_base, color='gray', alpha=0.25, label=texto if j == 0 else None)


def marcar_falhas(ax, xs, ys):
    """Marca onde o GNSS se perdeu (x) e voltou (+)."""
    for j, (ini, fim) in enumerate(falhas_gnss):
        k = int(np.searchsorted(tempos_fix, ini))
        ax.plot(xs[k], ys[k], 'x', color='black', markersize=10, mew=2,
                label='GNSS perdido' if j == 0 else None, zorder=4)
        ax.plot(xs[k + 1], ys[k + 1], '+', color='black', markersize=12, mew=2,
                label='GNSS recuperado' if j == 0 else None, zorder=4)


# Janela 1: Trajetória GNSS (Longitude vs Latitude)
fig1, ax1 = nova_janela('Trajetória GNSS')
if tem_gps:
    _, lon_p, lat_p = com_quebras(tempos_fix, longitudes, latitudes)
    ax1.plot(lon_p, lat_p, color='red', linewidth=1, label=f'GNSS ({TOPICO_GNSS})')
    marcar_falhas(ax1, longitudes, latitudes)
    ax1.legend(loc='best', fontsize=8)
    ax1.set_xlabel('Longitude')
    ax1.set_ylabel('Latitude')
    ax1.grid(True)
else:
    sem_dados(ax1, f'Sem dados para {TOPICO_GNSS} (GNSS)')
ax1.set_title('Trajetória GNSS')
fig1.tight_layout()

# Janela 2: Sobreposição de curvas (referencial local em metros, no ponto da antena)
fig2, ax2 = nova_janela('Sobreposição das técnicas de localização', (10, 9))
curvas = 0
if tem_gps:
    ax2.plot(gps_x, gps_y, '.', color='red', markersize=3, label='Posições GNSS'); curvas += 1
if int_x is not None:
    ax2.plot(*com_quebras(tempos_vel, int_x, int_y)[1:], color='tab:orange', linewidth=1,
             label='Integração da velocidade GNSS'); curvas += 1
if tem_odo:
    ax2.plot(odo_x, odo_y, color='tab:blue', linewidth=1, label=NOME_ODO); curvas += 1
if ine_x is not None:
    ax2.plot(ine_x, ine_y, color='tab:green', linewidth=1, label='Navegação inercial (IMU)'); curvas += 1
if ekf is not None:
    ax2.plot(ekf_ax, ekf_ay, color='magenta', linewidth=1, label=NOME_EKF); curvas += 1
if curvas:
    ax2.set_xlabel('x - Este (m)')
    ax2.set_ylabel('y - Norte (m)')
    ax2.set_aspect('equal', adjustable='datalim')
    # enquadra o percurso medido (a navegação inercial pode fugir para muito longe)
    ref_x, ref_y = (gps_x, gps_y) if tem_gps else ((odo_x, odo_y) if tem_odo else (None, None))
    if ref_x is not None:
        mx = 0.15 * np.ptp(ref_x) + 20
        my = 0.15 * np.ptp(ref_y) + 20
        ax2.set_xlim(np.min(ref_x) - mx, np.max(ref_x) + mx)
        ax2.set_ylim(np.min(ref_y) - my, np.max(ref_y) + my)
    ax2.grid(True)
    ax2.legend(loc='best', fontsize=8)
else:
    sem_dados(ax2, 'Sem dados para sobrepor')
ax2.set_title('Sobreposição das técnicas de localização (ponto da antena GNSS)')
fig2.tight_layout()

# Janelas 3 e 4: dados do IMU ao longo do tempo
fig3, ax3 = nova_janela('Acelerómetro', (10, 5))
fig4, ax4 = nova_janela('Giroscópio', (10, 5))
if len(acel):
    t_rel = tempos_imu - t_base
    for j, (eixo, cor) in enumerate(zip('xyz', ('tab:red', 'tab:green', 'tab:blue'))):
        ax3.plot(t_rel, acel[:, j], color=cor, linewidth=0.8, label=f'a{eixo}')
        ax4.plot(t_rel, gyro[:, j], color=cor, linewidth=0.8, label=f'ω{eixo}')
    ax3.set_ylabel('Aceleração (m/s²)')
    ax4.set_ylabel('Velocidade angular (rad/s)')
    for ax in (ax3, ax4):
        ax.set_xlabel('Tempo (s)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize=8)
else:
    for ax in (ax3, ax4):
        sem_dados(ax, f'Sem dados para {TOPICO_IMU}')
ax3.set_title(f'Acelerómetro ({TOPICO_IMU}, eixos do veículo)')
ax4.set_title(f'Giroscópio ({TOPICO_IMU}, eixos do veículo)')
fig3.tight_layout()
fig4.tight_layout()

# Janela 5: inovações do Kalman - diferença entre o GNSS e a previsão (odómetro + CAN), com ±3σ esperado.
# Bem afinado: as inovações ficam quase sempre dentro de ±3σ e sem tendência.
fig5, bx1 = nova_janela('Inovações do filtro de Kalman', (11, 6))
if inovacoes is not None and len(inovacoes[0]):
    ti, vi, si, oki = inovacoes
    tt = ti - t_base
    for j, (eixo, cor) in enumerate((('Este', 'tab:blue'), ('Norte', 'tab:green'))):
        t_q, v_q, s_q = com_quebras(tt, vi[:, j], si[:, j])
        bx1.plot(t_q, v_q, color=cor, linewidth=0.8, label=f'Inovação {eixo} (GNSS - previsão)')
        bx1.plot(t_q, 3 * s_q, color=cor, linewidth=0.8, linestyle='--', alpha=0.7, label=f'±3σ {eixo}')
        bx1.plot(t_q, -3 * s_q, color=cor, linewidth=0.8, linestyle='--', alpha=0.7)
    if np.any(~oki):
        bx1.plot(tt[~oki], np.hypot(vi[~oki, 0], vi[~oki, 1]), 'x', color='red', markersize=6,
                 label='GNSS rejeitado (outlier)')
    sombrear_falhas(bx1)
    bx1.set_yscale('symlog', linthresh=0.01)
    bx1.set_xlabel('Tempo (s)')
    bx1.set_ylabel('Inovação (m)')
    bx1.grid(True, which='both', alpha=0.4)
    bx1.legend(loc='upper left', fontsize=8, ncol=2)
else:
    sem_dados(bx1, 'O filtro de Kalman precisa do odómetro e do GNSS')
bx1.set_title('Inovações do filtro de Kalman (GNSS - previsão por odómetro + CAN)')
fig5.tight_layout()

# Janela 6: incerteza do EKF e offset do ângulo da roda (CAN) / bias do giroscópio estimado
fig6, bx2 = nova_janela('Filtro de Kalman (EKF)', (11, 6))
if tem_can:
    nome_b, conv_b = 'offset do ângulo da roda - CAN (º)', math.degrees(1)
else:
    nome_b, conv_b = 'bias do giroscópio (rad/s)', 1.0
if ekf is not None:
    tt = t_ekf - t_base
    bx2.plot(tt, sig_ekf[:, 0], label='σ x (m)', color='tab:blue')
    bx2.plot(tt, sig_ekf[:, 1], label='σ y (m)', color='tab:cyan')
    bx2.plot(tt, np.degrees(sig_ekf[:, 2]), label='σ rumo (º)', color='tab:purple')
    sombrear_falhas(bx2)
    bx2.set_yscale('log')
    bx2.set_ylabel('Desvio-padrão')
    bx2.set_xlabel('Tempo (s)')
    bx2.grid(True, which='both', alpha=0.4)
    bx2b = bx2.twinx()
    bx2b.plot(tt, ekf[:, 3] * conv_b, color='tab:red', linewidth=1, label=nome_b)
    bx2b.plot(tt, (ekf[:, 4] - 1) * 100, color='tab:orange', linewidth=1,
              label='erro de escala do odómetro (%)')
    bx2b.set_ylabel(f'{nome_b[0].upper() + nome_b[1:]} | escala (%)')
    l1, n1 = bx2.get_legend_handles_labels()
    l2, n2 = bx2b.get_legend_handles_labels()
    bx2.legend(l1 + l2, n1 + n2, loc='upper right', fontsize=8)
else:
    sem_dados(bx2, 'O filtro de Kalman precisa do odómetro e do GNSS')
bx2.set_title(f'Incerteza do EKF, {nome_b.split(" (")[0]} e escala do odómetro estimados')
fig6.tight_layout()

# Janela 7: localização final - odómetro + CAN, apenas GNSS e filtro de Kalman
fig7, cx = nova_janela('Localização final (odómetro + CAN, GNSS, Kalman)', (10, 9))
if tem_odo or tem_gps:
    if tem_odo:
        cx.plot(odo_x, odo_y, color='tab:blue', linewidth=1.5, label=NOME_ODO)
    if tem_gps:
        cx.plot(gps_x, gps_y, '.', color='red', markersize=2, alpha=0.5, label=NOME_GNSS, zorder=3)
        marcar_falhas(cx, gps_x, gps_y)
    if ekf is not None:
        cx.plot(ekf_ax, ekf_ay, color='magenta', linewidth=2, label=f'{NOME_EKF} - localização final')
        cx.plot(ekf_ax[0], ekf_ay[0], 'o', color='green', markersize=8, label='Início')
        cx.plot(ekf_ax[-1], ekf_ay[-1], 's', color='black', markersize=8, label='Fim')
    cx.set_xlabel('x - Este (m)')
    cx.set_ylabel('y - Norte (m)')
    cx.set_aspect('equal', adjustable='datalim')
    cx.grid(True)
    cx.legend(loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=3, fontsize=9)   # por baixo do mapa
else:
    sem_dados(cx, 'Sem odómetro nem GNSS')
cx.set_title(f'Trajeto: {NOME_ODO}, apenas GNSS e filtro de Kalman (ponto da antena)')
fig7.tight_layout()

plt.show()
