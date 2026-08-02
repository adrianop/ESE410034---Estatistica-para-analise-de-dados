"""Aquisição em tempo real do MPU6050 e fusão por filtro de Kalman.

O firmware envia: t_ms,ax,ay,az,gx,gy,gz (g e graus/s). Por padrão é
estimado o roll, usando atan2(ay, az) e a velocidade gx.
"""

from pathlib import Path
import argparse
from bisect import bisect_left
from collections import deque
import math
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

try:
    import serial
except ImportError:
    serial = None

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.append(str(CURRENT_DIR))

from simulacao import criar_filtro_imu_1d


def _converter_linha(linha, eixo):
    """Converte tanto o protocolo atual (7 campos) quanto o legado (2)."""
    valores = [float(v.strip()) for v in linha.split(",")]
    if len(valores) >= 7:
        t_ms, ax, ay, az, gx, gy, gz = valores[:7]
        if eixo == "roll":
            angulo = math.degrees(math.atan2(ay, az))
            velocidade = gx
        else:
            angulo = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            velocidade = gy
        return t_ms / 1000.0, velocidade, angulo
    if len(valores) >= 2:
        return None, valores[0], valores[1]
    raise ValueError("amostra serial incompleta")


def leitor_serial(porta, baudrate, eixo):
    if serial is None:
        raise ImportError("pyserial não está instalado. Execute: py -m pip install pyserial")

    ser = serial.Serial(porta, baudrate, timeout=0.03)
    print(f"Conectado em {porta} @ {baudrate} baud")
    time.sleep(2.0)  # o Arduino pode reiniciar ao abrir a porta
    ser.reset_input_buffer()

    linhas_invalidas = 0
    try:
        while True:
            linha = ser.readline().decode("utf-8", errors="ignore").strip()
            if not linha or linha.startswith("#"):
                continue
            try:
                yield _converter_linha(linha, eixo)
            except (ValueError, OverflowError):
                linhas_invalidas += 1
                if linhas_invalidas <= 3:
                    print(f"Amostra ignorada: {linha!r}")
    finally:
        ser.close()


def leitor_demo(dt, seed=123):
    rng = np.random.default_rng(seed)
    t = 0.0
    while True:
        theta = 20.0 * np.sin(0.8 * t) + 6.0 * np.sin(2.2 * t)
        omega = 16.0 * np.cos(0.8 * t) + 13.2 * np.cos(2.2 * t)
        yield t, omega + 0.8 + rng.normal(0.0, 0.8), theta + rng.normal(0.0, 5.0)
        t += dt
        time.sleep(dt)


def calibrar(fonte, quantidade):
    """Estima os zeros com a IMU parada na orientação inicial."""
    print(f"Calibrando com {quantidade} amostras. Mantenha o sensor imóvel...")
    giros, angulos = [], []
    while len(giros) < quantidade:
        _t, giro, angulo = next(fonte)
        if np.isfinite(giro) and np.isfinite(angulo):
            giros.append(giro)
            angulos.append(angulo)
    bias_giro = float(np.mean(giros))
    zero_angulo = float(np.mean(angulos))
    desvio_giro = float(np.std(giros, ddof=1)) if quantidade > 1 else 0.0
    desvio_angulo = float(np.std(angulos, ddof=1)) if quantidade > 1 else 0.0
    print(
        f"Calibração concluída: bias giro={bias_giro:.3f} °/s, "
        f"zero={zero_angulo:.3f}°, desvios=({desvio_giro:.3f} °/s, {desvio_angulo:.3f}°)"
    )
    return bias_giro, zero_angulo, desvio_angulo


def executar_dashboard(args):
    fonte = leitor_demo(args.dt) if args.demo else leitor_serial(
        args.port, args.baud, args.axis
    )
    titulo_fonte = "modo demo/simulado" if args.demo else f"MPU6050 - {args.port}"

    bias_giro = zero_angulo = 0.0
    r_acc = args.r_acc
    if not args.no_calibration:
        bias_giro, zero_angulo, desvio_acc = calibrar(fonte, args.calibration_samples)
        if args.auto_noise and desvio_acc > 0:
            r_acc = desvio_acc ** 2

    kf = criar_filtro_imu_1d(
        dt=args.dt,
        var_angulo=args.q_angle,
        var_bias=args.q_bias,
        var_acelerometro=r_acc,
    )
    tempos, angulos_acc, angulos_kf, biases = [], [], [], []
    janelas = ((10.0, "Últimos 10 segundos"),
               (60.0, "Últimos 60 segundos"),
               (3600.0, "Última hora"))
    # Três escalas de tempo na mesma tela. O painel de uma hora é reduzido
    # apenas para desenho; todas as amostras continuam sendo processadas.
    fig, eixos = plt.subplots(3, 1, figsize=(12, 9))
    # Margens fixas são muito mais leves que recalcular constrained_layout em
    # todos os quadros da animação.
    fig.subplots_adjust(left=0.08, right=0.98, bottom=0.07, top=0.90, hspace=0.38)
    fig.suptitle(f"Filtro de Kalman em tempo real — {titulo_fonte}", fontsize=12)
    texto_status = fig.text(0.5, 0.925, "Aguardando amostras...", ha="center", fontsize=10)
    linhas = []
    for ax, (_duracao, titulo) in zip(eixos, janelas):
        linha_acc, = ax.plot([], [], label="Acelerômetro", alpha=0.55, linewidth=1)
        linha_kf, = ax.plot([], [], label="Kalman", linewidth=1.7)
        linhas.append((linha_acc, linha_kf))
        ax.set_title(titulo, loc="left", fontsize=11)
        ax.set_ylabel("Ângulo (°)")
        ax.grid(True, alpha=0.3)
    eixos[0].legend(loc="upper right", ncol=2)
    eixos[-1].set_xlabel("Tempo desde o início (s)")
    inicio_pc = time.monotonic()
    primeiro_t_sensor = None
    fila = deque(maxlen=max(1000, args.window * 2))
    parar_leitura = threading.Event()
    erro_leitura = []

    def adquirir_continuamente():
        """Lê o sensor sem depender da velocidade de desenho do Matplotlib."""
        try:
            while not parar_leitura.is_set():
                fila.append(next(fonte))
        except StopIteration:
            pass
        except Exception as exc:  # a interface exibe o erro na próxima atualização
            erro_leitura.append(exc)

    thread_leitura = threading.Thread(
        target=adquirir_continuamente, name="leitura-mpu6050", daemon=True
    )
    thread_leitura.start()
    print(
        f"Aquisição: {1000 * args.dt:.0f} ms por amostra; "
        f"tela: até uma atualização a cada {args.refresh_ms} ms."
    )

    def update(_frame):
        nonlocal primeiro_t_sensor
        if erro_leitura:
            texto_status.set_text(f"Erro na leitura serial: {erro_leitura[0]}")
            texto_status.set_color("tab:red")
            return tuple(linha for par in linhas for linha in par)

        # Processa tudo que chegou desde o último desenho. A thread continua
        # adquirindo enquanto o Matplotlib renderiza este quadro.
        while fila:
            t_sensor, giro, angulo = fila.popleft()

            giro_calibrado = giro - bias_giro
            angulo_calibrado = angulo - zero_angulo
            kf.predict(u=giro_calibrado)
            x, _p, _k = kf.update(z=angulo_calibrado)

            if t_sensor is None:
                t = time.monotonic() - inicio_pc
            else:
                if primeiro_t_sensor is None:
                    primeiro_t_sensor = t_sensor
                t = t_sensor - primeiro_t_sensor
            tempos.append(t)
            angulos_acc.append(angulo_calibrado)
            angulos_kf.append(float(x[0, 0]))
            biases.append(float(x[1, 0]))

        if not tempos:
            return tuple(linha for par in linhas for linha in par)

        agora = tempos[-1]
        for ax, (duracao, _titulo), (linha_acc, linha_kf) in zip(
            eixos, janelas, linhas
        ):
            inicio = bisect_left(tempos, agora - duracao)
            quantidade = len(tempos) - inicio
            # Limita o custo do desenho da visão longa sem alterar o filtro.
            passo = max(1, quantidade // args.max_plot_points)
            xs = tempos[inicio::passo]
            ys_acc = angulos_acc[inicio::passo]
            ys_kf = angulos_kf[inicio::passo]
            linha_acc.set_data(xs, ys_acc)
            linha_kf.set_data(xs, ys_kf)
            ax.set_xlim(max(0.0, agora - duracao), max(duracao, agora))

            ys = ys_acc + ys_kf
            minimo, maximo = min(ys), max(ys)
            centro = (minimo + maximo) / 2.0
            amplitude = max(args.min_y_span, (maximo - minimo) * 1.20)
            ax.set_ylim(centro - amplitude / 2.0, centro + amplitude / 2.0)

        # Mantém somente a última hora. A limpeza em blocos evita custo a cada
        # amostra depois que a execução ultrapassa 3600 segundos.
        if tempos[-1] > 3610.0 and tempos[0] < tempos[-1] - 3610.0:
            remover = bisect_left(tempos, tempos[-1] - 3600.0)
            del tempos[:remover], angulos_acc[:remover]
            del angulos_kf[:remover], biases[:remover]

        texto_status.set_text(
            f"Ângulo={angulos_kf[-1]:.2f}°   |   bias={biases[-1]:.2f}°/s   |   "
            f"amostras={len(tempos)}"
        )
        return tuple(linha for par in linhas for linha in par)

    # A referência precisa permanecer viva; sem ela a animação pode parar.
    animacao = FuncAnimation(
        fig, update, interval=args.refresh_ms, blit=False,
        cache_frame_data=False,
    )
    fig._animacao_kalman = animacao
    fig.canvas.mpl_connect("close_event", lambda _event: parar_leitura.set())
    plt.show()
    parar_leitura.set()
    thread_leitura.join(timeout=0.2)


def parse_args():
    parser = argparse.ArgumentParser(description="Filtro de Kalman com MPU6050")
    parser.add_argument("--demo", action="store_true", help="usa dados simulados")
    parser.add_argument("--port", help="porta serial, por exemplo COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--axis", choices=("roll", "pitch"), default="roll")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument(
        "--refresh-ms", type=int, default=100,
        help="intervalo de atualização visual em milissegundos",
    )
    parser.add_argument("--window", type=int, default=400)
    parser.add_argument(
        "--max-plot-points", type=int, default=1000,
        help="máximo de pontos desenhados em cada painel",
    )
    parser.add_argument(
        "--min-y-span", type=float, default=5.0,
        help="amplitude vertical mínima do gráfico em graus",
    )
    parser.add_argument("--calibration-samples", type=int, default=150)
    parser.add_argument("--no-calibration", action="store_true")
    parser.add_argument("--auto-noise", action="store_true", help="estima R durante a calibração")
    parser.add_argument("--q-angle", type=float, default=1e-3)
    parser.add_argument("--q-bias", type=float, default=1e-5)
    parser.add_argument("--r-acc", type=float, default=25.0)
    args = parser.parse_args()
    if not args.demo and not args.port:
        parser.error("informe --port COMx ou use --demo")
    if args.calibration_samples < 2:
        parser.error("--calibration-samples deve ser pelo menos 2")
    if args.dt <= 0 or args.refresh_ms < 10:
        parser.error("--dt deve ser positivo e --refresh-ms deve ser pelo menos 10")
    if args.max_plot_points < 100:
        parser.error("--max-plot-points deve ser pelo menos 100")
    return args


if __name__ == "__main__":
    executar_dashboard(parse_args())
