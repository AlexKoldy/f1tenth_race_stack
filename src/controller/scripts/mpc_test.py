import numpy as np
import casadi

if __name__ == "__main__":
    opti = casadi.Opti()

    n = 2
    dt = 0.01

    # x = [p_x, v_x]
    X = opti.variable(2, n)
    U = opti.variable(1, n)

    # dynamics
    for i in range(n - 1):
        X[0, i + 1] = X[1, i + 1] * dt
        X[1, i + 1] = U[0, i]
