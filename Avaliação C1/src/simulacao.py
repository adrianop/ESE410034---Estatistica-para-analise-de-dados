
"""
simulacao.py
Parte B - Simulação sintética de fusão sensorial IMU com Filtro de Kalman.

O exemplo estima um ângulo 1D (roll ou pitch) combinando:
- Giroscópio: bom no curto prazo, mas sofre com bias/drift ao integrar.
- Acelerômetro: fornece ângulo absoluto ruidoso.

Unidades usadas: graus e graus/segundo.
"""

from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt

# Permite executar como: python src/simulacao.py
CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.append(str(CURRENT_DIR))

from kalman import KalmanFilter


def erro_quadratico_medio(y_true, y_pred):
    y_true = np.asarray(y_true, dtype=float)
    y_pred = np.asarray(y_pred, dtype=float)
    return float(np.mean((y_true - y_pred) ** 2))


def criar_filtro_imu_1d(dt, var_angulo=1e-3, var_bias=1e-5, var_acelerometro=25.0):
    """Cria o KF 1D para ângulo + bias do giroscópio.

    Estado:
        x = [theta, bias]^T

    Entrada:
        u = omega_giro_medido

    Modelo:
        theta_k = theta_{k-1} + dt * (omega_giro - bias_{k-1})
        bias_k  = bias_{k-1}

    Em forma matricial:
        x_k = F x_{k-1} + B u_k + w_k
        z_k = H x_k + v_k
    """
    F = np.array([[1.0, -dt],
                  [0.0,  1.0]])
    B = np.array([[dt],
                  [0.0]])
    H = np.array([[1.0, 0.0]])
    Q = np.diag([var_angulo, var_bias])
    R = np.array([[var_acelerometro]])
    P0 = np.diag([10.0, 1.0])
    x0 = np.array([[0.0], [0.0]])
    return KalmanFilter(F, B, H, Q, R, P0, x0)


def simular(seed=42, salvar_figuras=True):
    rng = np.random.default_rng(seed)

    # Tempo de simulação
    dt = 0.01
    t_final = 20.0
    t = np.arange(0.0, t_final, dt)

    # Sinal real de inclinação: combinação de senos para tornar a dinâmica mais rica
    theta_real = 25.0 * np.sin(0.7 * t) + 8.0 * np.sin(1.8 * t)
    omega_real = np.gradient(theta_real, dt)

    # Sensores simulados
    bias_giro_real = 0.8  # graus/s: pequeno erro constante do giroscópio
    ruido_giro_std = 0.6
    ruido_acc_std = 5.0

    gyro_medido = omega_real + bias_giro_real + rng.normal(0.0, ruido_giro_std, size=t.shape)
    acc_medido = theta_real + rng.normal(0.0, ruido_acc_std, size=t.shape)

    # Estimativa ingênua por integração do giroscópio para comparar drift
    theta_giro_integrado = np.cumsum(gyro_medido) * dt

    # Filtro de Kalman
    kf = criar_filtro_imu_1d(
        dt=dt,
        var_angulo=1e-3,
        var_bias=1e-5,
        var_acelerometro=ruido_acc_std ** 2,
    )

    theta_filtrado = []
    bias_estimado = []
    ganho_kalman_angulo = []

    for omega, z_acc in zip(gyro_medido, acc_medido):
        kf.predict(u=omega)
        x, P, K = kf.update(z=z_acc)
        theta_filtrado.append(x[0, 0])
        bias_estimado.append(x[1, 0])
        ganho_kalman_angulo.append(K[0, 0])

    theta_filtrado = np.asarray(theta_filtrado)
    bias_estimado = np.asarray(bias_estimado)
    ganho_kalman_angulo = np.asarray(ganho_kalman_angulo)

    mse_acc = erro_quadratico_medio(theta_real, acc_medido)
    mse_giro = erro_quadratico_medio(theta_real, theta_giro_integrado)
    mse_kf = erro_quadratico_medio(theta_real, theta_filtrado)

    resultados = {
        "dt": dt,
        "ruido_giro_std": ruido_giro_std,
        "ruido_acc_std": ruido_acc_std,
        "bias_giro_real": bias_giro_real,
        "mse_acelerometro": mse_acc,
        "mse_giro_integrado": mse_giro,
        "mse_kalman": mse_kf,
        "melhoria_vs_acelerometro_pct": 100.0 * (1.0 - mse_kf / mse_acc),
        "melhoria_vs_giro_pct": 100.0 * (1.0 - mse_kf / mse_giro),
    }

    if salvar_figuras:
        out_dir = Path(__file__).resolve().parents[1] / "docs" / "figures"
        out_dir.mkdir(parents=True, exist_ok=True)

        plt.figure(figsize=(12, 6))
        plt.plot(t, theta_real, label="Ângulo real")
        plt.plot(t, acc_medido, label="Acelerômetro ruidoso", alpha=0.35)
        plt.plot(t, theta_giro_integrado, label="Giroscópio integrado", alpha=0.70)
        plt.plot(t, theta_filtrado, label="Kalman filtrado", linewidth=2)
        plt.xlabel("Tempo (s)")
        plt.ylabel("Ângulo (graus)")
        plt.title("Fusão sensorial IMU: Real vs. Medido vs. Filtrado")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / "simulacao_kalman.png", dpi=160)

        plt.figure(figsize=(12, 4))
        plt.plot(t, bias_estimado, label="Bias estimado pelo KF")
        plt.axhline(bias_giro_real, linestyle="--", label="Bias real do giroscópio")
        plt.xlabel("Tempo (s)")
        plt.ylabel("Bias (graus/s)")
        plt.title("Estimativa do bias do giroscópio")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / "bias_estimado.png", dpi=160)

        plt.figure(figsize=(12, 4))
        plt.plot(t, ganho_kalman_angulo, label="K[ângulo]")
        plt.xlabel("Tempo (s)")
        plt.ylabel("Ganho")
        plt.title("Evolução do ganho de Kalman associado ao ângulo")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / "ganho_kalman.png", dpi=160)

        plt.show()

    return resultados


if __name__ == "__main__":
    resultados = simular(salvar_figuras=True)
    print("\nResultados da simulação")
    print("-" * 32)
    for chave, valor in resultados.items():
        if isinstance(valor, float):
            print(f"{chave}: {valor:.6f}")
        else:
            print(f"{chave}: {valor}")
