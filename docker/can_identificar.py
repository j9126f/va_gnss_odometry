#!/usr/bin/env python3
"""Descobre, no barramento CAN do carro, a mensagem e o campo com o ângulo de direção.

    docker compose run --rm can can_identificar.py                 (carro ligado, parado)
    docker compose run --rm can can_identificar.py --bitrate auto  (se não souberes o bitrate)
Fases: volante ao centro -> batente ESQUERDO e manter -> batente DIREITO e manter -> de volta ao centro e
exatamente UMA VOLTA (360º) para a esquerda (o símbolo do volante fica outra vez direito) -> dá a escala.
Um campo é candidato se estiver quieto com o volante parado, mudar suavemente ao rodar e ficar com
valores opostos à esquerda e à direita (assim exclui contadores, checksums, binário e velocidade do volante).
Só escuta (listen-only): não transmite nada para o carro.
"""
import argparse
import collections
import json
import os
import socket
import statistics
import struct
import subprocess
import sys
import time
from pathlib import Path

from can_sinal import extrair_bits

CAN_EFF_FLAG, CAN_ERR_FLAG = 0x80000000, 0x20000000
BITRATES = (500000, 250000, 1000000, 125000, 800000, 100000, 83333, 50000)


def abrir(interface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((interface,))
    s.settimeout(0.2)
    return s


def capturar(s, duracao, mostrar=True):
    """{(id, extendido): [(t, bytes), ...]} durante `duracao` segundos."""
    tramas = collections.defaultdict(list)
    fim = time.time() + duracao
    mostrar = mostrar and sys.stdout.isatty()
    proximo = 0.0
    while (agora := time.time()) < fim:
        if mostrar and agora >= proximo:          # contagem 2x por segundo (não a cada trama)
            print(f'\r   faltam {fim - agora:4.1f} s ', end='', flush=True)
            proximo = agora + 0.5
        try:
            f = s.recv(16)
        except socket.timeout:
            continue
        cid, dlc = struct.unpack('=IB3x', f[:8])
        if cid & CAN_ERR_FLAG:
            continue
        ext = bool(cid & CAN_EFF_FLAG)
        tramas[(cid & (0x1FFFFFFF if ext else 0x7FF), ext)].append((agora, f[8:8 + dlc]))
    if mostrar:
        print('\r' + ' ' * 24 + '\r', end='')
    return tramas


def configurar(interface, bitrate):
    return subprocess.run(['can_interface.sh', str(bitrate), 'forcar'], capture_output=True, text=True).returncode == 0


def procurar_bitrate(interface):
    for br in BITRATES:
        if not configurar(interface, br):
            sys.exit('Não consegui configurar a interface (ver mensagens do can_interface.sh).')
        s = abrir(interface)
        n = sum(len(v) for v in capturar(s, 1.5, mostrar=False).values())
        s.close()
        print(f'  {br:>7} bit/s: {n} tramas')
        if n > 20:
            print(f'-> bitrate {br}. Põe CAN_BITRATE={br} no .env')
            return br
    sys.exit('Nenhuma trama em nenhum bitrate: o carro está ligado? o adaptador está na ficha certa (CAN-H/CAN-L)?')


def nome_id(chave):
    cid, ext = chave
    return f'0x{cid:08X}' if ext else f'0x{cid:03X}'


def campos(dlc):
    """(descrição, bit DBC, n.º bits, ordem, com sinal) de cada campo candidato alinhado ao byte."""
    for k in range(dlc):
        yield (f'byte {k}', k * 8, 8, 'intel', False)
        yield (f'byte {k} c/ sinal', k * 8, 8, 'intel', True)
    for k in range(dlc - 1):
        for sinal in (False, True):
            s = ' c/ sinal' if sinal else ''
            yield (f'bytes {k}-{k + 1} big-endian{s}', k * 8 + 7, 16, 'motorola', sinal)
            yield (f'bytes {k}-{k + 1} little-endian{s}', k * 8, 16, 'intel', sinal)


def avaliar(base, esq, dir_):
    """Candidatos (pontuação, ...) para cada ID presente nas três fases."""
    cands = []
    for chave in base.keys() & esq.keys() & dir_.keys():
        dlc = min(len(d) for _, d in base[chave] + esq[chave] + dir_[chave])
        for desc, bit, bits, ordem, sinal in campos(dlc):
            val = lambda fase: [extrair_bits(d, bit, bits, ordem, sinal) for _, d in fase[chave]]
            vb, ve, vd = val(base), val(esq), val(dir_)
            if len(vb) < 5 or len(ve) < 10 or len(vd) < 10:
                continue
            fixo_e, fixo_d = ve[-max(3, len(ve) // 4):], vd[-max(3, len(vd) // 4):]   # parte final = batente
            m0, me, md = statistics.median(vb), statistics.median(fixo_e), statistics.median(fixo_d)
            de, dd = me - m0, md - m0
            ruido = max(vb) - min(vb)
            if de * dd >= 0 or min(abs(de), abs(dd)) < max(4, 10 * ruido):
                continue
            # suavidade: ao rodar o volante, o valor não pode dar saltos (exclui a ordem de bytes errada)
            serie = ve + vd
            amplitude = max(serie) - min(serie)
            saltos = sum(abs(b - a) > 0.25 * amplitude for a, b in zip(serie, serie[1:]))
            if saltos > 3:
                continue
            # o valor tem de ficar parado no batente (exclui binário/velocidade do volante)
            if max(fixo_e) - min(fixo_e) > 0.1 * abs(de) or max(fixo_d) - min(fixo_d) > 0.1 * abs(dd):
                continue
            pont = min(abs(de), abs(dd)) / (ruido + 1) * (2 if bits == 16 else 1)
            dt = base[chave][-1][0] - base[chave][0][0]
            taxa = (len(vb) - 1) / dt if dt > 0 else 0
            cands.append(dict(pont=pont, chave=chave, desc=desc, bit=bit, bits=bits, ordem=ordem, sinal=sinal,
                              m0=m0, me=me, md=md, ruido=ruido, taxa=taxa))
    cands.sort(key=lambda c: -c['pont'])
    return cands


def fase(s, texto, duracao, sem_pausa):
    print(f'\n>> {texto}')
    if sem_pausa:
        for i in range(3, 0, -1):
            print(f'   começa em {i}...', end='\r', flush=True)
            time.sleep(1)
    else:
        input('   [Enter] para começar ')
    s.setblocking(False)                      # descarta as tramas acumuladas enquanto se esperava pelo Enter
    try:
        while s.recv(16):
            pass
    except BlockingIOError:
        pass
    s.settimeout(0.2)
    t = capturar(s, duracao)
    print(f'   {sum(len(v) for v in t.values())} tramas, {len(t)} IDs')
    return t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default=None)
    ap.add_argument('--bitrate', default=None, help="bit/s, ou 'auto' para procurar")
    ap.add_argument('--duracao', type=float, default=20, help='segundos de cada fase de viragem (rodar + manter)')
    ap.add_argument('--sem-pausa', action='store_true', help='não esperar pelo Enter entre fases')
    ap.add_argument('--ficheiro', default=None, help='repete a análise a partir de um data/can_identificacao_*.json')
    a = ap.parse_args()
    if a.ficheiro:
        return analisar(*ler_fases(a.ficheiro))
    interface = a.interface or os.environ.get('CAN_IF', 'can0')

    if a.bitrate == 'auto':
        procurar_bitrate(interface)
    elif a.bitrate:
        configurar(interface, a.bitrate)
    elif subprocess.run(['can_interface.sh']).returncode:
        sys.exit(1)

    s = abrir(interface)
    print('\n=== Tráfego no barramento (2 s)')
    geral = capturar(s, 2)
    if not geral:
        sys.exit('Nenhuma trama. Carro desligado, bitrate errado (experimenta --bitrate auto) ou cabo mal ligado.')
    for chave, v in sorted(geral.items()):
        print(f'  {nome_id(chave):>11}  {len(v) / 2:6.0f} Hz  dlc {len(v[-1][1])}  {v[-1][1].hex(" ")}')

    d = a.duracao
    base = fase(s, 'Volante AO CENTRO (rodas direitas). Depois do Enter NÃO toques no volante (8 s).', 8, a.sem_pausa)
    esq = fase(s, f'Depois do Enter: roda o volante DEVAGAR até ao batente ESQUERDO (uns 5-8 s a rodar)\n'
                  f'   e DEIXA-O LÁ PARADO até a contagem acabar ({d:.0f} s no total).', d, a.sem_pausa)
    dir_ = fase(s, f'Depois do Enter: roda o volante DEVAGAR até ao batente DIREITO (passa pelo centro)\n'
                   f'   e DEIXA-O LÁ PARADO até a contagem acabar ({d:.0f} s no total).', d, a.sem_pausa)
    volta = fase(s, f'ANTES do Enter: põe o volante AO CENTRO (sem pressa).\n'
                    f'   Depois do Enter: roda-o exatamente UMA VOLTA (360º) para a ESQUERDA (o símbolo do volante\n'
                    f'   fica outra vez direito) e DEIXA-O LÁ PARADO até a contagem acabar ({d:.0f} s).',
                 d, a.sem_pausa)
    s.close()
    destino = gravar_fases(dict(base=base, esq=esq, dir=dir_, volta=volta))
    analisar(base, esq, dir_, volta, destino)


def gravar_fases(fases):
    """Grava as tramas das 4 fases em /data (pasta dos bags) para se poder repetir a análise."""
    pasta = Path('/data')
    if not pasta.is_dir():
        return None
    destino = pasta / f'can_identificacao_{time.strftime("%Y%m%d_%H%M%S")}.json'
    dados = {nome: [[t, cid, ext, d.hex()] for (cid, ext), v in f.items() for t, d in v] for nome, f in fases.items()}
    destino.write_text(json.dumps(dados))
    return destino


def ler_fases(caminho):
    dados = json.loads(Path(caminho).read_text())
    fases = {}
    for nome, linhas in dados.items():
        f = collections.defaultdict(list)
        for t, cid, ext, d in sorted(linhas):
            f[(cid, bool(ext))].append((t, bytes.fromhex(d)))
        fases[nome] = f
    return fases['base'], fases['esq'], fases['dir'], fases['volta']


def analisar(base, esq, dir_, volta, destino=None):
    cands = avaliar(base, esq, dir_)
    if not cands:
        sys.exit('\nNenhum campo candidato. Repete com mais tempo (--duracao 30) e confirma que o volante chega aos '
                 'batentes; se o carro só envia a direção com o motor a trabalhar, liga o motor.')

    rel = ['\n=== Candidatos (melhor primeiro)']
    rel.append(f'  {"ID":>11} {"Hz":>5}  {"campo":<32} {"centro":>8} {"esquerda":>9} {"direita":>8} {"ruído":>6}')
    for c in cands[:12]:
        rel.append(f'  {nome_id(c["chave"]):>11} {c["taxa"]:5.0f}  {c["desc"]:<32} {c["m0"]:8.0f} {c["me"]:9.0f} '
              f'{c["md"]:8.0f} {c["ruido"]:6.0f}')

    c = cands[0]
    sentido = 1 if c['me'] > c['m0'] else -1
    vv = [extrair_bits(d, c['bit'], c['bits'], c['ordem'], c['sinal']) for _, d in volta.get(c['chave'], [])]
    escala, nota = 0.1, 'NÃO MEDIDA: repete a última fase (uma volta de 360º para a esquerda)'
    if len(vv) >= 10:
        fixo = vv[-max(3, len(vv) // 4):]
        dv = statistics.median(fixo) - c['m0']
        if max(fixo) - min(fixo) > 0.1 * abs(dv) or dv * (c['me'] - c['m0']) <= 0 or abs(dv) < 4 * (c['ruido'] + 1):
            nota = ('NÃO MEDIDA: na última fase o volante não ficou parado uma volta à esquerda; '
                    'repete a identificação')
        else:
            escala = 360 / abs(dv)
            nota = 'medida com a volta de 360º do volante'
            if abs(dv) > abs(c['me'] - c['m0']):
                nota += ' (ATENÇÃO: maior do que o batente; confirma que foi só uma volta)'
    rel.append(f'\n=== Sugestão para o .env (melhor candidato)\n'
          f'CAN_DIRECAO_ID=0x{c["chave"][0]:X}\n'
          f'CAN_DIRECAO_EXT={int(c["chave"][1])}\n'
          f'CAN_DIRECAO_BIT={c["bit"]}\n'
          f'CAN_DIRECAO_BITS={c["bits"]}\n'
          f'CAN_DIRECAO_ORDEM={c["ordem"]}\n'
          f'CAN_DIRECAO_SINAL={int(c["sinal"])}\n'
          f'CAN_DIRECAO_ESCALA={escala:.6g}      # {nota}\n'
          f'CAN_DIRECAO_OFFSET={-c["m0"] * escala + 0.0:.6g}      # põe a 0 o valor lido com o volante ao centro\n'
          f'CAN_DIRECAO_SENTIDO={sentido}\n'
          f'\nCom estes valores, o volante no batente esquerdo está a {sentido * (c["me"] - c["m0"]) * escala:+.0f}º '
          f'e no direito a {sentido * (c["md"] - c["m0"]) * escala:+.0f}º.\n'
          f'O ângulo das RODAS = ângulo do volante / CAN_RELACAO_DIRECAO (desmultiplicação da direção, ≈ 14-17;\n'
          f'ver ficha técnica, ou medir o ângulo das rodas no batente: relação = ângulo do volante / ângulo das rodas).')

    texto = '\n'.join(rel)
    print(texto)
    if destino:
        (destino.parent / (destino.stem + '.txt')).write_text(texto + '\n', encoding='utf-8')
        print(f'\nResultado e tramas gravados em data/{destino.stem}.txt e .json')

if __name__ == '__main__':
    main()
