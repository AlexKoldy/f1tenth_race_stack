import numpy as np
import casadi as ca
from typing import Dict, Tuple
import os

from utils.utils import nearest_point


class ModelPredictiveController:
    def __init__(
        self,
        Q_track: np.ndarray,
        Q_drift: float,
        R_input: np.ndarray,
        R_steering_rate: float,
        N: int,
        dt: float,
        distance_step: float,
        rk4_refinement: int,
        model_params: Dict[str, float],
        state_bounds: np.ndarray,
        input_bounds: np.ndarray,
    ) -> None:
        """
        Default constructor for model predictive control

        Arguments:
            Q_track (np.ndarray): position tracking cost of shape (2, 2)
            Q_drift (float): cost on lateral velocity
            R_input (np.ndarray): input usage cost of shape (2, 2)
            R_steering_rate (float): cost on change in steering angle
            N (int): horizon
            dt (float): timestep size [s]
            distance_step (float): reference trajectory distance for interpolation [m]
            rk4_refinement (int): timestep refinement for integration
            model_params (Dict[str, float]): model parameter dictionary
            state_bounds (np.ndarray): bounds on state variables of shape (6, 2)
            input_bounds (np.ndarray): bounds on input variables of shape (2, 2)
        """
        # Costs
        self.Q_track = Q_track
        self.Q_drift = Q_drift
        self.R_input = R_input
        self.R_steering_rate = R_steering_rate

        # MPC parameters
        self.N = N
        self.dt = dt
        self.rk4_refinement = rk4_refinement

        # Reference trajectory parameters
        self.distance_step = distance_step

        # Constraint parameters
        self.state_bounds = state_bounds
        self.input_bounds = input_bounds

        # Model parameters
        self.state_dim = 6
        self.input_dim = 2
        self.m = model_params["m"]  # mass of robot [kg]
        self.ell_f = model_params[
            "ell_f"
        ]  # distance between front wheel and center of rotation [m]
        self.ell_r = model_params[
            "ell_r"
        ]  # distance between rear wheel and center of rotation [m]
        self.I_z = model_params[
            "I_z"
        ]  # rotational moment of intertia about z-axis [kgm^2]
        self.h_cog = model_params["h_cog"]  # height of center of gravity [m]
        self.mu = model_params["mu"]  # coefficient of friction
        self.C_f = model_params["C_f"]  # front axle Pacejka parameters
        self.C_r = model_params["C_r"]  # rear axke Pacejka parameters

        # Initialize fourth-order runge-kutta integration of dynamics
        self.eff = self.initialize_dynamics()

        # Initialize costs
        self.J_track, self.J_drift, self.J_steering_rate, self.J_input = (
            self.initialize_costs()
        )

        # TODO:
        self.so_path = "./nmpc.so"

        self.initialize_nonlinear_optimization()

    def pacejka_tire_model(self, alpha: ca.SX, F_z: ca.SX, C: np.ndarray) -> ca.SX:
        """
        Returns the lateral force given the current normal force and slip angle of
        the vehicle

        Arguments:
            alpha (ca.SX): slip angle [rad]
            F_z (ca.SX): normal force [N]
            C (np.ndarray): tire parameters in order of
                (stiffness factor, shape factor, peak value, curvature factor)

        Returns:
            (ca.SX) lateral force [N]

        """
        return (
            self.mu
            * F_z
            * C[2]
            * ca.sin(
                C[1]
                * ca.arctan(
                    C[0] * alpha - C[3] * (C[0] * alpha - ca.arctan(C[0] * alpha))
                )
            )
        )

    def initialize_dynamics(self) -> ca.Function:
        """
        Sets up nonlinear dynamics ordinary differential equation

            x_dot = f(x, u),

        where x = [p_x, p_y, theta, v_x, v_y, omega]^T and
        u = [delta, a_x]^T. Returns the fourth order Runge-
        Kutta integration of dynamics, i.e.,

            x_kp1 = eff(x_k, u_k, dt)

        Returns:
            (ca.Function): integrated dynamics function
                eff(x, u, dt)
        """
        # State 'x':
        #   x_0 ('p_x') := x-position (global frame) [m]
        #   x_1 ('p_y') := y-position (gobal frame) [m]
        #   x_2 ('theta') := yaw [rad]
        #   x_3 ('v_x') := longitudinal velocity (robot frame) [m/s]
        #   x_4 ('v_y') := lateral velocity (robot frame) [m/s]
        #   x_5 ('omega') := yaw rate [rad/s]
        p_x = ca.SX.sym("p_x")
        p_y = ca.SX.sym("p_y")
        theta = ca.SX.sym("theta")
        v_x = ca.SX.sym("v_x")
        v_y = ca.SX.sym("v_y")
        omega = ca.SX.sym("omega")
        x = ca.vertcat(p_x, p_y, theta, v_x, v_y, omega)

        # Control input 'u':
        #   u_0 ('delta') := steering angle [rad]
        #   u_1 ('a_x'):= longitudinal acceleration
        delta = ca.SX.sym("delta")
        a_x = ca.SX.sym("a_x")
        u = ca.vertcat(delta, a_x)

        # Calculate lateral forces due to slip
        g = -9.81  # acceleration in z due to gravity [m/s^2]
        alpha_f = (
            ca.arctan((v_y - omega * self.ell_f) / v_x) - delta
        )  # front side slip angle [rad]
        alpha_r = ca.arctan(
            (v_y - omega * self.ell_r) / v_x
        )  # rear side slip angle [rad]
        F_z_f = (self.m * g * self.ell_r - self.m * a_x * self.h_cog) / (
            self.ell_r + self.ell_f
        )  # force on front axle [N]
        F_z_r = (self.m * g * self.ell_f + self.m * a_x * self.h_cog) / (
            self.ell_r + self.ell_f
        )  # force on rear axle [N]

        # F_y_f = self.pacejka_tire_model(
        #     alpha_f, F_z_f, self.C_f
        # )  # lateral force on front axle [N]
        # F_y_r = self.pacejka_tire_model(
        #     alpha_r, F_z_r, self.C_r
        # )  # lateral force on rear axle [N]

        F_y_f = self.mu * F_z_f * self.C_f[0] * alpha_f
        F_y_r = self.mu * F_z_r * self.C_r[0] * alpha_r

        # Dynamics x_dot = f(x, u)
        x_dot = ca.vertcat(
            v_x * ca.cos(theta) - v_y * ca.sin(theta),
            v_x * ca.sin(theta) + v_y * ca.cos(theta),
            omega,
            a_x - (F_y_f / self.m) * ca.sin(delta) + v_y * omega,
            (F_y_r / self.m) + (F_y_f / self.m) * ca.cos(delta) - v_x * omega,
            (1 / self.I_z) * (F_y_f * self.ell_f * ca.cos(delta) - F_y_r * self.ell_r),
            # delta_dot,
        )
        f = ca.Function("f", [x, u], [x_dot], ["x", "u"], ["ode"])

        # Fourth-order Runge-Kutta integration
        X_0 = ca.SX.sym("X", x.shape[0])
        U = ca.SX.sym("U", u.shape[0])
        X = X_0
        DT = ca.SX.sym("dt", 1)
        dt = DT / self.rk4_refinement
        for _ in range(self.rk4_refinement):
            k_1 = dt * f(X, U)
            k_2 = dt * f(X + 0.5 * k_1, U)
            k_3 = dt * f(X + 0.5 * k_2, U)
            k_4 = dt * f(X + k_3, U)

            X = X + (k_1 + 2 * k_2 + 2 * k_3 + k_4) / 6

        eff = ca.Function("eff", [X_0, U, DT], [X])
        return eff

    def initialize_costs(
        self,
    ) -> Tuple[ca.Function, ca.Function, ca.Function, ca.Function]:
        """
        Sets up costs on tracking, drifting, steering rate and inputs

        Returns
            (Tuple[ca.Function, ca.Function, ca.Function, ca.Function]): cost functions
        """
        e_position = ca.SX.sym("e_position", 3)  # position error [m]
        v_y = ca.SX.sym("v_y", 1)  # lateral velocity [m/s]
        ddelta = ca.SX.sym("ddelta", 1)  # change in steering angle [rad]
        u = ca.SX.sym("u", 2)  # control input

        cost_track = e_position.T @ self.Q_track @ e_position
        cost_drift = v_y * self.Q_drift * v_y
        cost_steering_rate = ddelta * self.R_steering_rate * ddelta
        cost_input = u.T @ self.R_input @ u

        J_track = ca.Function("J_track", [e_position], [cost_track])
        J_drift = ca.Function("J_drift", [v_y], [cost_drift])
        J_steering_rate = ca.Function("J_steering_rate", [ddelta], [cost_steering_rate])
        J_input = ca.Function("J_input", [u], [cost_input])

        return (J_track, J_drift, J_steering_rate, J_input)

    def initialize_nonlinear_optimization(self) -> None:
        """
        TODO
        """
        # Nonlinear programming problem
        w = []  # nlp variables
        w_0 = []  # initial guess of nlp variables
        lb_w = []  # lower bound on nlp variables
        ub_w = []  # upper bound on nlp variables

        cost = 0  # objective
        g = []  # nlp constraint
        lb_g = []  # lower bound on constraint
        ub_g = []  # upper bound on constraint

        x_min = list(self.state_bounds[:, 0])
        x_max = list(self.state_bounds[:, 1])
        u_min = list(self.input_bounds[:, 0])
        u_max = list(self.input_bounds[:, 1])
        g_min = [0 for _ in range(self.state_dim)]
        g_max = [0 for _ in range(self.state_dim)]

        # 'P' represents all parameters which are not to be optimized,
        # i.e., the initial state (represented by self.state_dim), and
        # the path to track (represented by 2 * self.N). The order for
        # 'P' is as follows: p_x_0, p_y_0, p_theta_0, p_v_x_0, p_v_y_0,
        # p_omega_0, x_waypoint_0, y_waypoint_0, x_waypoint_1,
        # y_waypoint_1, ..., x_waypoint_N, y_waypoint_N
        P = ca.SX.sym("P", self.state_dim + (3 * self.N))
        X = ca.SX.sym("X", self.state_dim, self.N + 1)
        U = ca.SX.sym("U", self.input_dim, self.N)

        # TODO: change?
        # state_guess = [0 for _ in range(self.state_dim)]  # TODO
        # state_guess[3] = 0.1  # TODO
        state_guess = [4.97914549, 23.11841581, -1.9881565, 1.0, 0.0, 0.0]
        control_guess = [0.0, 1.0]

        # Get the next state
        X_next = self.eff(X[:, : self.N], U, self.dt)

        # Initial conditions
        w += [X[:, 0]]
        w_0 += state_guess  # TODO
        lb_w += x_min
        ub_w += x_max

        g += [X[:, 0] - P[0 : self.state_dim]]
        lb_g += g_min
        ub_g += g_max

        for k in range(self.N):
            w += [U[:, k]]
            w_0 += control_guess  # TODO
            lb_w += u_min
            ub_w += u_max

            # Calculate costs
            tracking_cost_k = self.J_track(
                X[0:3, k + 1]
                - P[self.state_dim + (3 * k) : self.state_dim + (3 * k) + 3]
            )
            drift_cost_k = self.J_drift(X[4, k + 1])
            input_cost_k = self.J_input(U[:, k])
            if not k == self.N - 1:
                steering_rate_cost_k = self.J_steering_rate(U[0, k + 1] - U[0, k])
            else:
                steering_rate_cost_k = 0.0
            cost = (
                cost
                + tracking_cost_k
                # + drift_cost_k
                # + input_cost_k
                # + steering_rate_cost_k
            )

            w += [X[:, k + 1]]
            w_0 += state_guess
            lb_w += x_min
            ub_w += x_max

            g += [X_next[:, k] - X[:, k + 1]]
            lb_g += g_min
            ub_g += g_max

        nlp = {
            "f": cost,
            "x": ca.vertcat(*w),
            "p": P,
            "g": ca.vertcat(*g),
        }

        self.w_0 = w_0
        self.lb_w = lb_w
        self.ub_w = ub_w
        self.lb_g = lb_g
        self.ub_g = ub_g

        self.solver = ca.nlpsol("solver", "ipopt", nlp)
        # cname = solver.generate_dependencies("nmpc.c")
        # print("starting compilation")
        # os.system("gcc -fPIC -shared -O3 " + cname + " -o " + self.so_path)
        # print("end")
        # self.solver = ca.nlpsol("solver", "ipopt", self.so_path)

    def calculate_reference_trajectory(
        self,
        x: np.ndarray,
        u_prev: np.ndarray,
        path: np.ndarray,
        project_state: bool = True,
        ind=1,
    ) -> np.ndarray:
        """"""
        # Find total number of waypoints in the full path
        num_waypoints = path.shape[1]

        # Set up reference path for MPC
        mpc_path = np.zeros((3, self.N + 1))

        # Project the state forward to account for computation
        if project_state:
            x = self.eff(x, u_prev, self.dt)

        # Find nearest index/setpoint from where the trajectories are calculated
        # _, _, _, ind = nearest_point(x[0:2], path.T)

        # ind = 1

        # Start the trajectory at the nearest point
        mpc_path[0, 0] = path[0, ind]
        mpc_path[1, 0] = path[1, ind]

        # Pick waypoints based off a distance step and travel at current velocity
        travel = abs(x[3]) * self.dt
        dind = travel / self.distance_step

        # Fill the reference trajectory
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.N)), 0, 0
        ).astype(int)

        ind_list[ind_list >= num_waypoints] -= num_waypoints
        # print(ind_list)
        # exit()

        mpc_path[0, :] = path[0, ind_list]
        mpc_path[1, :] = path[1, ind_list]

        # TODO
        final_idx = ind_list[-1] + 1
        if final_idx >= num_waypoints:
            final_idx -= num_waypoints

        # diff = path[:, final_idx] - path[:, ind_list[-1]]
        diff = path[:, ind_list[-1]] - path[:, ind]
        angle = np.arctan2(diff[1], diff[0])
        mpc_path[2] = angle

        return mpc_path[:, :-1]

    def step(self, x: np.ndarray, path: np.ndarray) -> None:
        """
        TODO
        """
        import time

        start_time = time.time()
        sol = self.solver(
            x0=self.w_0,
            lbx=self.lb_w,
            ubx=self.ub_w,
            p=np.concatenate((x, path.flatten("F"))),
            lbg=self.lb_g,
            ubg=self.ub_g,
        )
        print(1 / (time.time() - start_time))
        sol_x0 = sol["x"].full()
        breh = sol_x0[: -self.state_dim].flatten()
        bruh = breh.reshape(-1, 8).T

        # print(bruh[6:, :])

        import matplotlib.pyplot as plt

        # plt.figure()
        # # plt.plot(bruh[0, :], bruh[1, :])
        # plt.plot(np.linspace(0, 10, 10), np.round(bruh[7, :], 3))
        # plt.show()
        # exit()
        return bruh


if __name__ == "__main__":
    N = 10
    mpc = ModelPredictiveController(
        Q_track=np.diag([5.0, 5.0, 1.0]),
        Q_drift=0.0,
        R_input=np.diag([0.0, 0.0]),
        R_steering_rate=1.0,
        N=N,
        dt=0.025,
        distance_step=0.01,
        rk4_refinement=4,
        model_params={
            "m": 3.54,
            "ell_f": 0.162,
            "ell_r": 0.145,
            "I_z": 0.05797,
            "mu": 1.0,
            "h_cog": 0.014,
            "C_f": np.array(
                [
                    4.47161357602916,
                    0.1320293068694414,
                    12.267008918241816,
                    1.5562751013900538,
                ]
            ),
            "C_r": np.array(
                [
                    9.999999999999812,
                    1.4999999999992566,
                    1.3200250015860229,
                    1.0999999999999999,
                ]
            ),
        },
        state_bounds=np.array(
            [
                [-ca.inf, ca.inf],
                [-ca.inf, ca.inf],
                [-ca.inf, ca.inf],
                [1.0, ca.inf],  # [0.0, 5.0],
                [-ca.inf, ca.inf],  # [-5.0, 5.0],
                [-ca.inf, ca.inf],
            ]
        ),
        input_bounds=np.array(
            [
                [-0.05, 0.05],
                [-10.0, 10.0],
                # [-ca.inf, ca.inf],
                # [-ca.inf, ca.inf],
            ]
        ),
    )

    # x = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    # u = np.array([0.4, 2.0])
    # x_list = [x[0]]
    # y_list = [x[1]]
    # for _ in range(10000):
    #     x = mpc.eff(x, u, 0.001)
    #     x_list.append(x[0])
    #     y_list.append(x[1])

    #     alpha_f = np.arctan((x[4] - x[5] * mpc.ell_f) / x[3]) - u[0]
    #     print(alpha_f)
    #     # if u[0] < 0.05:
    #     #     u[0] += 0.001
    #     # else:
    #     #     break

    #     # print(x[4])

    # import matplotlib.pyplot as plt

    # plt.figure()
    # plt.plot(x_list, y_list)
    # plt.show()
    # exit()
    # path = np.loadtxt("testing/waypoints.csv", delimiter=",", skiprows=1)
    trajectory_data = np.load("final.npz")
    path = trajectory_data["path"]

    # start_idx = np.random.randint(0, path.shape[1])
    start_idx = 700
    x1, y1 = path[:, start_idx]
    x2, y2 = path[:, start_idx + 1]
    angle = np.arctan2(y2 - y1, x2 - x1)

    x_0 = np.array([path[0, start_idx], path[1, start_idx], angle, 4.0, 0.0, 0.0])
    print(x_0)

    mpc_path = mpc.calculate_reference_trajectory(
        x=x_0,
        u_prev=np.array([0.0, 0.0]),
        path=path,
        project_state=False,
        ind=start_idx,
    )
    # mpc_path = path[:, start_idx : start_idx + 10]
    mpc_path = np.kron(mpc_path[:, -1].reshape((3, 1)), np.ones(N))

    bruh = mpc.step(
        x=x_0,
        path=mpc_path[0:3, :],
    )
    angle = mpc_path[2, -1]
    print(start_idx)

    import matplotlib.pyplot as plt

    plt.figure()

    plt.plot(path[0, :], path[1, :], label="global path")
    plt.scatter(mpc_path[0, :], mpc_path[1, :], label="reference path")
    plt.arrow(
        mpc_path[0, -1],
        mpc_path[1, -1],
        np.cos(angle),
        np.sin(angle),
        head_width=0.2,
        head_length=0.3,
        fc="b",
        ec="b",
    )
    # plt.scatter(x_0[0], x_0[1])
    plt.scatter(bruh[0, :], bruh[1, :], label="MPC trajectory")
    plt.legend()
    plt.show()
