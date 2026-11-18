"""Descodificação de um sinal numa trama CAN, com a mesma convenção dos ficheiros DBC (cantools).

Usado pelo nó can_direcao.py (no carro) e pelo ler_can.py (off-line, a partir do /can/raw do bag).
Configuração por variáveis de ambiente (ficheiro .env), prefixo CAN_DIRECAO_:
  ID          identificador da mensagem (hex, ex.: 0x25 ou 25)       EXT    1 = ID de 29 bits
  BIT         bit de início, como no DBC                             BITS   n.º de bits
  ORDEM       intel (little-endian) ou motorola (big-endian)         SINAL  1 = com sinal (complemento para 2)
  ESCALA      graus por unidade                                      OFFSET graus
  SENTIDO     1 ou -1, para que positivo = viragem à ESQUERDA
Mais CAN_RELACAO_DIRECAO: desmultiplicação volante -> rodas (1 se o CAN já dá o ângulo das rodas).
Ângulo das rodas (graus) = SENTIDO * (valor * ESCALA + OFFSET) / RELACAO_DIRECAO
"""
import math
import os
from dataclasses import dataclass


def extrair_bits(data, bit, bits, ordem='motorola', com_sinal=False):
    """Valor inteiro de um sinal CAN (convenção DBC para o bit de início)."""
    d = bytes(data).ljust(8, b'\0')[:8]
    mascara = (1 << bits) - 1
    if ordem.lower().startswith(('i', 'l')):     # intel / little-endian: bit = LSB
        v = (int.from_bytes(d, 'little') >> bit) & mascara
    else:                                         # motorola / big-endian: bit = MSB (numeração "dente de serra")
        msb = (bit // 8) * 8 + (7 - bit % 8)      # posição do MSB contando do bit mais alto do byte 0
        v = (int.from_bytes(d, 'big') >> (64 - msb - bits)) & mascara
    if com_sinal and v >> (bits - 1):
        v -= 1 << bits
    return v


@dataclass
class SinalDirecao:
    can_id: int
    extendido: bool = False
    bit: int = 7
    bits: int = 16
    ordem: str = 'motorola'
    com_sinal: bool = True
    escala: float = 0.1
    offset: float = 0.0
    sentido: float = 1.0
    relacao: float = 1.0

    @classmethod
    def do_ambiente(cls, env=os.environ):
        cid = env.get('CAN_DIRECAO_ID', '').strip()
        if not cid:
            return None
        return cls(can_id=int(cid, 16),
                   extendido=env.get('CAN_DIRECAO_EXT', '0').strip() == '1',
                   bit=int(env.get('CAN_DIRECAO_BIT', '7')),
                   bits=int(env.get('CAN_DIRECAO_BITS', '16')),
                   ordem=env.get('CAN_DIRECAO_ORDEM', 'motorola').strip(),
                   com_sinal=env.get('CAN_DIRECAO_SINAL', '1').strip() == '1',
                   escala=float(env.get('CAN_DIRECAO_ESCALA', '0.1')),
                   offset=float(env.get('CAN_DIRECAO_OFFSET', '0')),
                   sentido=float(env.get('CAN_DIRECAO_SENTIDO', '1')),
                   relacao=float(env.get('CAN_RELACAO_DIRECAO', '1') or 1))

    def bruto(self, data):
        return extrair_bits(data, self.bit, self.bits, self.ordem, self.com_sinal)

    def volante_graus(self, data):
        """Ângulo tal como vem no CAN (volante ou rodas, conforme o carro), positivo = esquerda."""
        return self.sentido * (self.bruto(data) * self.escala + self.offset)

    def rodas_rad(self, data):
        """Ângulo de viragem das rodas da frente (rad, positivo = esquerda)."""
        return math.radians(self.volante_graus(data) / self.relacao)

    def descricao(self):
        return (f"ID 0x{self.can_id:X}{' (29 bits)' if self.extendido else ''}, bit {self.bit}, "
                f"{self.bits} bits {self.ordem}, {'com' if self.com_sinal else 'sem'} sinal, "
                f"escala {self.escala} º, offset {self.offset} º, sentido {self.sentido:+g}, "
                f"relação volante/rodas {self.relacao:g}")
