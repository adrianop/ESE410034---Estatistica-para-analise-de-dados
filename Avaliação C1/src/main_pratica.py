"""Estimativa da inclinação em tempo real com os dados da IMU pela Serial."""

import argparse
import itertools
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from kalman import KalmanFilter


# Configuração
BAUDRATE = 115200
TAXA_HZ = 50.0
JANELA_S = 10.0

DESVIO_GIROSCOPIO = 0.3
DESVIO_ACELEROMETRO = 3.0
PASSEIO_DO_BIAS = 0.05

try:
    plt.style.use('seaborn-v0_8-colorblind')
except OSError:
    plt.style.use('seaborn-colorblind')

plt.rcParams['axes.labelsize'] = 13
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['legend.fontsize'] = 11
plt.rcParams['lines.linewidth'] = 2


# Fontes de dados
class FonteSerial:
    """Lê as linhas que chegam pela porta Serial sem travar o gráfico."""

    def __init__(self, porta):
        try:
            import serial
        except ImportError:
            sys.exit('Instale o pyserial: pip install pyserial')

        self.conexao = serial.Serial(porta, BAUDRATE, timeout=0)
        self.resto = ''

    def novas_linhas(self):
        self.resto += self.conexao.read(4096).decode('ascii', 'ignore')
        linhas = self.resto.split('\n')
        self.resto = linhas.pop()
        return linhas


class FonteArquivo:
    """Reproduz um arquivo CSV gravado, em blocos, para simular a chegada."""

    def __init__(self, caminho, por_quadro=5):
        with open(caminho, encoding='utf-8') as arquivo:
            self.linhas = iter(arquivo.read().splitlines())
        self.por_quadro = por_quadro

    def novas_linhas(self):
        return list(itertools.islice(self.linhas, self.por_quadro))


def interpretar(linha):
    """Converte uma linha CSV do firmware em tempo, aceleração e rotação."""
    linha = linha.strip()
    if not linha or linha.startswith('#'):
        return None

    partes = linha.split(',')
    if len(partes) != 7:
        return None

    try:
        valores = [float(p) for p in partes]
    except ValueError:
        return None

    return {'t': valores[0] / 1000.0,
            'acel': valores[1:4],
            'giro': valores[4:7]}


# Modelo da IMU
def montar_filtro(dt):
    """Monta o filtro com as matrizes do problema da inclinação."""
    F = np.array([[1.0, -dt],
                  [0.0, 1.0]])

    B = np.array([[dt],
                  [0.0]])

    H = np.array([[1.0, 0.0]])

    Q = np.array([[(DESVIO_GIROSCOPIO * dt) ** 2, 0.0],
                  [0.0, (PASSEIO_DO_BIAS ** 2) * dt]])

    R = np.array([[DESVIO_ACELEROMETRO ** 2]])

    x0 = np.array([[0.0],
                   [0.0]])

    P0 = np.diag([10.0 ** 2, 4.0 ** 2])

    return KalmanFilter(F=F, B=B, H=H, Q=Q, R=R, P0=P0, x0=x0)


def angulo_do_acelerometro(acel_x, acel_y, acel_z):
    """Inclinação em torno do eixo X a partir da direção da gravidade."""
    return math.degrees(math.atan2(acel_y, acel_z))


# Medida do ruído para sintonizar Q e R
def medir_ruido(fonte, segundos):
    """Coleta amostras com a placa parada e mostra o desvio de cada sensor."""
    angulos = []
    rotacoes = []
    inicio = None

    print(f'Deixe a placa parada por {segundos:.0f} s...')
    while True:
        for linha in fonte.novas_linhas():
            amostra = interpretar(linha)
            if amostra is None:
                continue

            if inicio is None:
                inicio = amostra['t']
            if amostra['t'] - inicio > segundos:
                print(f'\nAmostras: {len(angulos)}')
                print(f'Acelerometro: desvio {np.std(angulos):.3f} graus')
                print(f'Giroscopio:   desvio {np.std(rotacoes):.3f} graus/s, '
                      f'media {np.mean(rotacoes):.3f} graus/s (bias)')
                return

            angulos.append(angulo_do_acelerometro(*amostra['acel']))
            rotacoes.append(amostra['giro'][0])


# Programa principal
def main():
    interpretador = argparse.ArgumentParser(
        description='Filtro de Kalman sobre os dados da IMU')
    interpretador.add_argument('--porta', help='porta Serial (COM3, /dev/ttyUSB0)')
    interpretador.add_argument('--replay', help='arquivo CSV gravado')
    interpretador.add_argument('--medir-ruido', type=float, metavar='SEGUNDOS',
                               help='mede o desvio de cada sensor e sai')
    argumentos = interpretador.parse_args()

    if not argumentos.porta and not argumentos.replay:
        interpretador.error('escolha --porta ou --replay')

    if argumentos.replay:
        fonte = FonteArquivo(argumentos.replay)
    else:
        fonte = FonteSerial(argumentos.porta)

    if argumentos.medir_ruido:
        medir_ruido(fonte, argumentos.medir_ruido)
        return

    dt_nominal = 1.0 / TAXA_HZ
    filtro = montar_filtro(dt_nominal)

    tempos = []
    medidos = []
    filtrados = []
    biases = []
    anterior = [None]

    figura, (eixo_angulo, eixo_bias) = plt.subplots(
        2, 1, figsize=(11, 8), sharex=True,
        gridspec_kw={'height_ratios': [3, 1]})

    linha_medido, = eixo_angulo.plot([], [], linewidth=1, alpha=0.5,
                                     label='Acelerômetro')
    linha_filtrada, = eixo_angulo.plot([], [], label='Filtro de Kalman')
    eixo_angulo.set_ylabel('Inclinação [°]')
    eixo_angulo.set_title('Estimativa da inclinação em tempo real')
    eixo_angulo.set_ylim(-95, 95)
    eixo_angulo.legend(loc='upper right')
    eixo_angulo.grid(alpha=0.3)

    linha_bias, = eixo_bias.plot([], [], color='tab:red')
    eixo_bias.set_xlabel('Tempo [s]')
    eixo_bias.set_ylabel('Bias [°/s]')
    eixo_bias.grid(alpha=0.3)

    maximo = int(JANELA_S * TAXA_HZ)

    def processar(amostra):
        # o intervalo vem do relógio do Arduino, que varia um pouco
        dt = dt_nominal
        if anterior[0] is not None:
            intervalo = amostra['t'] - anterior[0]
            if 0 < intervalo < 0.5:
                dt = intervalo
        anterior[0] = amostra['t']

        filtro.F[0, 1] = -dt
        filtro.B[0, 0] = dt

        angulo = angulo_do_acelerometro(*amostra['acel'])
        filtro.predict(u=amostra['giro'][0])
        estado, _covariancia, _ganho = filtro.update(z=angulo)

        tempos.append(amostra['t'])
        medidos.append(angulo)
        filtrados.append(estado[0, 0])
        biases.append(estado[1, 0])

        del tempos[:-maximo], medidos[:-maximo]
        del filtrados[:-maximo], biases[:-maximo]

    def atualizar(_quadro):
        for linha in fonte.novas_linhas():
            amostra = interpretar(linha)
            if amostra is not None:
                processar(amostra)

        if tempos:
            linha_medido.set_data(tempos, medidos)
            linha_filtrada.set_data(tempos, filtrados)
            linha_bias.set_data(tempos, biases)
            eixo_angulo.set_xlim(max(0.0, tempos[-1] - JANELA_S), tempos[-1] + 0.5)
            eixo_bias.set_ylim(min(biases) - 0.5, max(biases) + 0.5)

        return linha_medido, linha_filtrada, linha_bias

    _animacao = FuncAnimation(figura, atualizar, interval=40,
                              cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
