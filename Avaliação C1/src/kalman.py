
import numpy as np


class KalmanFilter:

    def __init__(self, F, B, H, Q, R, P0, x0):
        self.F = np.asarray(F, dtype=float)
        self.B = None if B is None else np.asarray(B, dtype=float)
        self.H = np.asarray(H, dtype=float)
        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)
        self.P = np.asarray(P0, dtype=float)
        self.x = np.asarray(x0, dtype=float)

        if self.x.ndim == 1:
            self.x = self.x.reshape(-1, 1)

        self.n = self.x.shape[0]
        self.I = np.eye(self.n)

        self._validate_dimensions()

    def _validate_dimensions(self):
        if self.F.shape != (self.n, self.n):
            raise ValueError(f"F deve ter dimensão {(self.n, self.n)}, recebido {self.F.shape}")
        if self.Q.shape != (self.n, self.n):
            raise ValueError(f"Q deve ter dimensão {(self.n, self.n)}, recebido {self.Q.shape}")
        if self.P.shape != (self.n, self.n):
            raise ValueError(f"P0 deve ter dimensão {(self.n, self.n)}, recebido {self.P.shape}")
        if self.H.shape[1] != self.n:
            raise ValueError("H deve ter o mesmo número de colunas do tamanho do estado")
        if self.R.shape[0] != self.R.shape[1] or self.R.shape[0] != self.H.shape[0]:
            raise ValueError("R deve ser quadrada e compatível com a dimensão da medição")
        if self.B is not None and self.B.shape[0] != self.n:
            raise ValueError("B deve ter o mesmo número de linhas do tamanho do estado")

    def predict(self, u=None):
        if self.B is not None and u is not None:
            u = np.asarray(u, dtype=float)
            if u.ndim == 0:
                u = u.reshape(1, 1)
            elif u.ndim == 1:
                u = u.reshape(-1, 1)
            self.x = self.F @ self.x + self.B @ u
        else:
            self.x = self.F @ self.x

        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x.copy(), self.P.copy()

    def update(self, z):
        z = np.asarray(z, dtype=float)
        if z.ndim == 0:
            z = z.reshape(1, 1)
        elif z.ndim == 1:
            z = z.reshape(-1, 1)

        y = z - self.H @ self.x                       # inovação/resíduo
        S = self.H @ self.P @ self.H.T + self.R       # covariância da inovação
        K = self.P @ self.H.T @ np.linalg.inv(S)      # ganho de Kalman

        self.x = self.x + K @ y

        # Forma de Joseph: numericamente mais estável que P=(I-KH)P
        IKH = self.I - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T

        return self.x.copy(), self.P.copy(), K.copy()
