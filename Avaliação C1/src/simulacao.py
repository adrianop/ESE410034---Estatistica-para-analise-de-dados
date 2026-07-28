"""Simulação do filtro de Kalman para fusão de dados de uma IMU."""

import os

import matplotlib.pyplot as plt
import numpy as np

from kalman import KalmanFilter


# Configuração dos gráficos
try:
    plt.style.use('seaborn-v0_8-colorblind')
except OSError:
    plt.style.use('seaborn-colorblind')

plt.rcParams['figure.figsize'] = (12, 7)
plt.rcParams['axes.labelsize'] = 16
plt.rcParams['axes.titlesize'] = 16
plt.rcParams['legend.fontsize'] = 13
plt.rcParams['lines.linewidth'] = 3

PASTA_FIGURAS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '..', 'docs', 'figuras')


# Geração dos sinais simulados
def gerar_sinais_sinteticos(dt=0.01,
                            duracao_s=20.0,
                            amplitude_graus=30.0,
                            frequencia_hz=0.25,
                            desvio_giroscopio=0.5,
                            bias_giroscopio=2.0,
                            desvio_acelerometro=4.0,
                            semente=42):
    """Gera os sinais simulados do giroscópio e acelerômetro."""
    gerador = np.random.default_rng(semente)
    tempo = np.arange(0.0, duracao_s, dt)

    angulo_real = amplitude_graus * np.sin(2 * np.pi * frequencia_hz * tempo)
    velocidade_real = (amplitude_graus * 2 * np.pi * frequencia_hz
                       * np.cos(2 * np.pi * frequencia_hz * tempo))

    leitura_giroscopio = (velocidade_real
                          + bias_giroscopio
                          + gerador.normal(0.0, desvio_giroscopio, tempo.size))

    leitura_acelerometro = (angulo_real
                            + gerador.normal(0.0, desvio_acelerometro, tempo.size))

    return tempo, angulo_real, leitura_giroscopio, leitura_acelerometro


# Matrizes do modelo
def montar_matrizes_imu(dt, desvio_giroscopio, desvio_acelerometro,
                        passeio_do_bias=0.05):
    """Monta as matrizes do filtro para o modelo da IMU."""
    F = np.array([[1.0, -dt],
                  [0.0, 1.0]])

    B = np.array([[dt],
                  [0.0]])

    H = np.array([[1.0, 0.0]])

    Q = np.array([[(desvio_giroscopio * dt) ** 2, 0.0],
                  [0.0, (passeio_do_bias ** 2) * dt]])

    R = np.array([[desvio_acelerometro ** 2]])

    x0 = np.array([[0.0],
                   [0.0]])

    P0 = np.diag([10.0 ** 2, 4.0 ** 2])

    return F, B, H, Q, R, P0, x0


# Execução do filtro
def rodar_filtro(leitura_giroscopio, leitura_acelerometro, dt,
                 desvio_giroscopio, desvio_acelerometro):
    """Executa o filtro ponto a ponto sobre os dados ruidosos."""
    F, B, H, Q, R, P0, x0 = montar_matrizes_imu(dt, desvio_giroscopio,
                                                desvio_acelerometro)

    filtro = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R, P0=P0, x0=x0)

    total = leitura_giroscopio.size
    angulo_filtrado = np.zeros(total)
    bias_estimado = np.zeros(total)
    ganho_da_inclinacao = np.zeros(total)

    for k in range(total):
        filtro.predict(u=leitura_giroscopio[k])
        estado, _covariancia, ganho = filtro.update(z=leitura_acelerometro[k])

        angulo_filtrado[k] = estado[0, 0]
        bias_estimado[k] = estado[1, 0]
        ganho_da_inclinacao[k] = ganho[0, 0]

    return angulo_filtrado, bias_estimado, ganho_da_inclinacao


def calcular_eqm(estimativa, verdade):
    """Erro quadrático médio entre a estimativa e o valor verdadeiro."""
    return np.mean((estimativa - verdade) ** 2)


# Geração dos gráficos
def salvar(nome_do_arquivo):
    """Salva a figura atual em docs/figuras/, criando a pasta se preciso."""
    os.makedirs(PASTA_FIGURAS, exist_ok=True)
    plt.savefig(os.path.join(PASTA_FIGURAS, nome_do_arquivo),
                dpi=120, bbox_inches='tight')


def plotar_comparativo(tempo, angulo_real, leitura_acelerometro,
                       angulo_filtrado, eqm_acelerometro, eqm_filtro):
    """Compara o sinal real, o medido e o filtrado."""
    plt.figure()
    plt.plot(tempo, leitura_acelerometro, linewidth=1, alpha=0.5,
             label='Acelerômetro')
    plt.plot(tempo, angulo_real, linestyle='--', color='black',
             label='Inclinação real')
    plt.plot(tempo, angulo_filtrado,
             label='Filtro de Kalman')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Inclinação do corpo [°]')
    plt.title('Comparação entre o sinal real, medido e filtrado')
    plt.legend()
    plt.grid(alpha=0.3)
    salvar('comparativo_real_medido_filtrado.png')


def plotar_deriva(tempo, angulo_real, angulo_so_giroscopio, angulo_filtrado,
                  eqm_giroscopio, eqm_filtro):
    """Mostra a deriva da integração pura do giroscópio."""
    plt.figure()
    plt.plot(tempo, angulo_real, linestyle='--', color='black',
             label='Inclinação real')
    plt.plot(tempo, angulo_so_giroscopio,
             label='Giroscópio integrado')
    plt.plot(tempo, angulo_filtrado,
             label='Filtro de Kalman')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Inclinação do corpo [°]')
    plt.title('Deriva da integração do giroscópio')
    plt.legend()
    plt.grid(alpha=0.3)
    salvar('deriva_giroscopio.png')


def plotar_bias(tempo, bias_estimado, bias_verdadeiro):
    """Mostra a convergência do bias estimado."""
    plt.figure()
    plt.axhline(bias_verdadeiro, linestyle='--', color='black',
                label=f'Bias verdadeiro ({bias_verdadeiro:.1f} °/s)')
    plt.plot(tempo, bias_estimado, label='Bias estimado pelo filtro')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Vício de zero do giroscópio [°/s]')
    plt.title('Estimativa do bias do giroscópio')
    plt.legend()
    plt.grid(alpha=0.3)
    salvar('estimativa_bias.png')


def plotar_ganho(tempo, ganho_da_inclinacao):
    """Mostra a queda do ganho de Kalman ao longo do tempo."""
    plt.figure()
    plt.plot(tempo, ganho_da_inclinacao)
    plt.xlabel('Tempo [s]')
    plt.ylabel('Ganho de Kalman da inclinação  K[0]')
    plt.title('Evolução do ganho de Kalman')
    plt.yscale('log')  # a queda passa de uma ordem de grandeza
    plt.grid(alpha=0.3, which='both')
    salvar('ganho_de_kalman.png')


# Programa principal
def main(mostrar_graficos=True):
    dt = 0.01
    desvio_giroscopio = 0.5
    bias_giroscopio = 2.0
    desvio_acelerometro = 4.0

    tempo, angulo_real, leitura_giroscopio, leitura_acelerometro = (
        gerar_sinais_sinteticos(dt=dt,
                                desvio_giroscopio=desvio_giroscopio,
                                bias_giroscopio=bias_giroscopio,
                                desvio_acelerometro=desvio_acelerometro))

    angulo_filtrado, bias_estimado, ganho_da_inclinacao = rodar_filtro(
        leitura_giroscopio, leitura_acelerometro, dt,
        desvio_giroscopio, desvio_acelerometro)

    angulo_so_giroscopio = np.cumsum(leitura_giroscopio * dt)

    # Validação estatística
    eqm_acelerometro = calcular_eqm(leitura_acelerometro, angulo_real)
    eqm_giroscopio = calcular_eqm(angulo_so_giroscopio, angulo_real)
    eqm_filtro = calcular_eqm(angulo_filtrado, angulo_real)
    reducao = (1 - eqm_filtro / eqm_acelerometro) * 100

    print('=' * 62)
    print('VALIDACAO ESTATISTICA - Erro Quadratico Medio (EQM)')
    print('=' * 62)
    print(f'  Acelerometro sozinho ............ {eqm_acelerometro:9.3f} graus^2')
    print(f'  Giroscopio sozinho (integrado) .. {eqm_giroscopio:9.3f} graus^2')
    print(f'  Filtro de Kalman ................ {eqm_filtro:9.3f} graus^2')
    print('-' * 62)
    print(f'  Reducao do erro vs acelerometro:  {reducao:8.1f} %')
    print(f'  Bias do giroscopio - real: {bias_giroscopio:.2f} graus/s | '
          f'estimado: {bias_estimado[-1]:.2f} graus/s')
    print(f'  Ganho de Kalman K[0] - inicial: {ganho_da_inclinacao[0]:.4f} | '
          f'final: {ganho_da_inclinacao[-1]:.4f}')
    print('=' * 62)

    # Gráficos
    plotar_comparativo(tempo, angulo_real, leitura_acelerometro,
                       angulo_filtrado, eqm_acelerometro, eqm_filtro)
    plotar_deriva(tempo, angulo_real, angulo_so_giroscopio, angulo_filtrado,
                  eqm_giroscopio, eqm_filtro)
    plotar_bias(tempo, bias_estimado, bias_giroscopio)
    plotar_ganho(tempo, ganho_da_inclinacao)

    print(f'\nFiguras salvas em: {os.path.normpath(PASTA_FIGURAS)}')

    if mostrar_graficos:
        plt.show()

    return {'eqm_acelerometro': eqm_acelerometro,
            'eqm_giroscopio': eqm_giroscopio,
            'eqm_filtro': eqm_filtro,
            'reducao_percentual': reducao}


if __name__ == '__main__':
    main()