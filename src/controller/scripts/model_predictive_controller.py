import numpy as np
import casadi
from typing import Tuple


class ModelPredictiveController:
    def __init__(self, n: int) -> None:
        """"""
        pass

    def declare_decision_variables(
        self, opti: casadi.Opti, state_dim: int, control_dim: int, n: int
    ) -> Tuple[casadi.Opti.variable, casadi.Opti.variable]:
        """"""
        # Set up state decision variables
        X = opti.variable(state_dim, n)

        # Set up input decision variables
        U = opti.variable(control_dim, n)

        return (X, U)

    def f(self, x: np.ndarray, u: np.ndarray) -> None:
        """"""
        pass
