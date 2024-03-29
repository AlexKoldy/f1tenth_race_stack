import casadi
import numpy as np
import scipy
import matplotlib.pyplot as plt
from typing import Tuple, Dict


class TrajectoryBuilder:
    def __init__(
        self,
        alpha_min: float,
        alpha_max: float,
        a_x_accel_max: float,
        a_x_decel_max: float,
        a_y_max: float,
        v_x_min: float,
        v_x_max: float,
    ) -> None:
        """"""
        # Set minimum and maximum deviations from centerline
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max

        # Set maximum lateral and longitudinal acceleration
        self.a_x_accel_max = a_x_accel_max  # [m/s^2]
        self.a_x_decel_max = a_x_decel_max  # [m/s^2]
        self.a_y_max = a_y_max  # [m/s^2]

        # Set longitudinal velocity limit
        self.v_x_min = v_x_min  # [m/s^2]
        self.v_x_max = v_x_max  # [m/s^2]

        # Set up list of paths for plotting
        self.paths = []

        # Set up velocity profile
        self.velocity_profile = None

    def generate_trajectory(
        self, path: np.ndarray, num_waypoints: int = 200, num_iterations: int = 1
    ) -> Tuple[np.ndarray, np.ndarray]:
        """"""
        # Append the original path to list of paths
        self.paths.append(path)

        # Rerun the optimization as many times as wanted
        for _ in range(num_iterations):
            # Generate global path
            spline_x, spline_y, _ = self.optimize_path(self.paths[-1][:, :-1])

            # Discretize and append new path
            self.paths.append(self.discretize_spline(spline_x, spline_y, num_waypoints))

        # Generate a velocity profile
        velocity_profile, curvatures = self.generate_raw_profile_data(
            spline_x, spline_y, num_waypoints
        )
        velocity_profile = self.generate_forward_pass_profile(
            velocity_profile, self.paths[-1], curvatures
        )
        velocity_profile = self.generate_backward_pass_profile(
            velocity_profile, self.paths[-1], curvatures
        )
        self.velocity_profile = velocity_profile

        # TODO: this is cheating
        self.velocity_profile[-15:] = self.v_x_max

        return (self.paths[-1], velocity_profile)

    def optimize_path(
        self, path: np.ndarray, warm_start_params: Dict[str, np.ndarray] = None
    ) -> Tuple[scipy.interpolate.PPoly, scipy.interpolate.PPoly, Dict[str, np.ndarray]]:
        """"""
        # Set up optimization problem
        opti = casadi.Opti()

        # Declare decision variables
        alpha, C_x, C_y = self.declare_decision_variables(opti, path.shape[1])

        # Generate the set of unit orthogonal vectors
        n = self.generate_orthogonal_vectors(path)

        # Set the spline constraints
        self.set_spline_constraints(opti, path, n, alpha, C_x, C_y)

        # Set the deviation constraints
        self.set_deviation_constraints(opti, alpha, self.alpha_min, self.alpha_max)

        # Set up minimum curvature cost
        self.set_minimum_curvature_cost(opti, C_x, C_y)

        # Set initial conditions
        self.set_initial_conditions(opti, alpha, C_x, C_y, warm_start_params)

        # Set the solver
        opti.solver("ipopt")

        # Solve the optimization problem
        sol = opti.solve()

        # Construct piecewise polynomial spline
        spline_x, spline_y = self.construct_splines(sol.value(C_x), sol.value(C_y))

        # Set up the maximum parameterization varible, i.e., the original number of waypoints,
        # since each spline is parameterized between 0 and 1
        self.m = path.shape[1]

        return (
            spline_x,
            spline_y,
            {
                "alpha": sol.value(alpha),
                "C_x": sol.value(C_x),
                "C_y": sol.value(C_y),
            },
        )

    def set_initial_conditions(
        self,
        opti: casadi.Opti,
        alpha: casadi.Opti.variable,
        C_x: casadi.Opti.variable,
        C_y: casadi.Opti.variable,
        warm_start_params: Dict[str, np.ndarray] = None,
    ) -> None:
        """"""
        # If we are warm starting, then use the warm start parameters,
        # otherwise initialize to some default
        if warm_start_params:
            opti.set_initial(alpha, warm_start_params["alpha"])
            opti.set_initial(C_x, warm_start_params["C_x"])
            opti.set_initial(C_y, warm_start_params["C_y"])
        else:
            # Set initial conditions on optimization variables. We set C_x and
            # C_y to avoid issues with dividing by zero in the cost
            opti.set_initial(alpha, 0)
            opti.set_initial(C_x, 1)
            opti.set_initial(C_y, 1)

    def declare_decision_variables(
        self, opti: casadi.Opti, m: int
    ) -> casadi.Opti.variable:
        """"""
        # Set up deviation optimization variable
        alpha = opti.variable(m)

        # Set up spline coefficient optimization variables
        C_x = opti.variable(4, m)
        C_y = opti.variable(4, m)
        return alpha, C_x, C_y

    def generate_orthogonal_vectors(self, path: np.ndarray) -> np.ndarray:
        """"""
        # Generate vector between p_{i+1} and p_i
        s = path[:, 1:] - path[:, 0:-1]

        # Assuming we have a closed loop path, generate an additional
        # vector from the path's start to the path's end
        s = np.hstack((s, np.reshape(path[:, 0] - path[:, -1], (2, 1))))

        # Ensure each vector is a unit vector
        s = s / np.linalg.norm(s, axis=0)

        # Find the orthogonal vector relative to s at each point p on
        # the path
        n = np.array([-s[1], s[0]])

        return n

    def set_spline_constraints(
        self,
        opti: casadi.Opti,
        path: np.ndarray,
        n: np.ndarray,
        alpha: casadi.Opti.variable,
        C_x: casadi.Opti.variable,
        C_y: casadi.Opti.variable,
    ) -> None:
        """"""
        # Number of waypoints
        m = path.shape[1]

        # Loop through splines between path waypoints (m - 1) splines
        for i in range(m - 1):
            # The start of each spline should be placed on the
            # current discretization point
            opti.subject_to(C_x[0, i] == path[0, i] + alpha[i, 0] * n[0, i])
            opti.subject_to(C_y[0, i] == path[1, i] + alpha[i, 0] * n[1, i])

            # The end of each spline should be placed on the
            # next discretization point
            opti.subject_to(
                C_x[0, i] + C_x[1, i] + C_x[2, i] + C_x[3, i]
                == path[0, i + 1] + alpha[i + 1, 0] * n[0, i + 1]
            )
            opti.subject_to(
                C_y[0, i] + C_y[1, i] + C_y[2, i] + C_y[3, i]
                == path[1, i + 1] + alpha[i + 1, 0] * n[1, i + 1]
            )

            # First derivative continuity
            opti.subject_to(
                C_x[1, i] + 2 * C_x[2, i] + 3 * C_x[3, i] - C_x[1, i + 1] == 0
            )
            opti.subject_to(
                C_y[1, i] + 2 * C_y[2, i] + 3 * C_y[3, i] - C_y[1, i + 1] == 0
            )

            # Second derivative continuity
            opti.subject_to(2 * C_x[2, i] + 6 * C_x[3, i] - 2 * C_x[2, i + 1] == 0)
            opti.subject_to(2 * C_y[2, i] + 6 * C_y[3, i] - 2 * C_y[2, i + 1] == 0)

        # The start of the closing spline should be placed at the final
        # discretization point
        opti.subject_to(C_x[0, m - 1] == path[0, m - 1] + alpha[m - 1, 0] * n[0, m - 1])
        opti.subject_to(C_y[0, m - 1] == path[1, m - 1] + alpha[m - 1, 0] * n[1, m - 1])

        # The end of the closing spline should be placed at the initial
        # discretization point
        opti.subject_to(
            C_x[0, m - 1] + C_x[1, m - 1] + C_x[2, m - 1] + C_x[3, m - 1]
            == path[0, 0] + alpha[0, 0] * n[0, 0]
        )
        opti.subject_to(
            C_y[0, m - 1] + C_y[1, m - 1] + C_y[2, m - 1] + C_y[3, m - 1]
            == path[1, 0] + alpha[0, 0] * n[1, 0]
        )

        # First derivative continuity for closing spline
        opti.subject_to(
            C_x[1, m - 1] + 2 * C_x[2, m - 1] + 3 * C_x[3, m - 1] - C_x[1, 0] == 0
        )
        opti.subject_to(
            C_y[1, m - 1] + 2 * C_y[2, m - 1] + 3 * C_y[3, m - 1] - C_y[1, 0] == 0
        )

        # Second derivative continuity
        opti.subject_to(2 * C_x[2, m - 1] + 6 * C_x[3, m - 1] - 2 * C_x[2, 0] == 0)
        opti.subject_to(2 * C_y[2, m - 1] + 6 * C_y[3, m - 1] - 2 * C_y[2, 0] == 0)

    def set_deviation_constraints(
        self,
        opti: casadi.Opti,
        alpha: casadi.Opti.variable,
        alpha_min: float,
        alpha_max: float,
    ) -> None:
        """"""
        for i in range(alpha.shape[0]):
            opti.subject_to(alpha_min <= alpha[i, 0])
            opti.subject_to(alpha[i, 0] <= alpha_max)

    def set_minimum_curvature_cost(
        self, opti: casadi.Opti, C_x: casadi.Opti.variable, C_y: casadi.Opti.variable
    ) -> None:
        """"""
        # Number of waypoints
        m = C_x.shape[1]

        # Set up cost
        cost = 0
        for i in range(m):
            # Evaluate spline derivatives at t = 0
            x_dot_i = C_x[1, i]
            y_dot_i = C_y[1, i]
            x_ddot_i = 2 * C_x[2, i]
            y_ddot_i = 2 * C_y[2, i]

            # Calculate curvature
            kappa_i = (x_dot_i * y_ddot_i - y_dot_i * x_ddot_i) / (
                x_dot_i**2 + y_dot_i**2
            ) ** (3 / 2)

            # Add squared curvature to cost
            cost += kappa_i**2

        opti.minimize(cost)

    def construct_splines(
        self, C_x: np.ndarray, C_y: np.ndarray
    ) -> Tuple[scipy.interpolate.PPoly, scipy.interpolate.PPoly]:
        """ """
        spline_x = scipy.interpolate.PPoly(
            np.flip(C_x, axis=0), np.arange(0, C_x.shape[1] + 1, 1)
        )
        spline_y = scipy.interpolate.PPoly(
            np.flip(C_y, axis=0), np.arange(0, C_y.shape[1] + 1, 1)
        )

        return spline_x, spline_y

    def plot_paths(self) -> None:
        """"""
        # Plot results
        plt.figure()
        plt.title("Results of minimum curvature optimization")
        plt.xlabel(r"$x$ [m]")
        plt.ylabel(r"$y$ [m]")
        plt.plot(
            np.concatenate([self.paths[0][0], np.reshape(self.paths[0][0, 0], (1))]),
            np.concatenate([self.paths[0][1], np.reshape(self.paths[0][1, 0], (1))]),
            label="original",
        )
        for i, path in enumerate(self.paths[1:]):
            plt.scatter(path[0, 0], path[1, 0])
            plt.plot(
                path[0, :],
                path[1, :],
                label=rf"optimized $k={i + 1}$",
            )
        plt.legend()
        plt.show()

    def plot_optimized_trajectory(self) -> None:
        """"""
        plt.figure()
        plt.title("Optimized trajectory")
        plt.xlabel(r"$x$ [m]")
        plt.ylabel(r"$y$ [m]")
        scatter = plt.scatter(
            self.paths[-1][0],
            self.paths[-1][1],
            c=self.velocity_profile,
            cmap="viridis",
        )
        plt.colorbar(scatter, label="Velocity [m/s]")
        plt.show()

    def discretize_spline(
        self,
        spline_x: scipy.interpolate.PPoly,
        spline_y: scipy.interpolate.PPoly,
        num_waypoints: int = 200,
    ) -> np.ndarray:
        t = np.linspace(0, self.m, num_waypoints)
        return np.array([spline_x(t), spline_y(t)])

    def calculate_spline_curvature(
        self,
        spline_x: scipy.interpolate.PPoly,
        spline_y: scipy.interpolate.PPoly,
        t: float,
    ) -> float:
        """"""
        return (spline_x(t, 1) * spline_y(t, 2) - spline_y(t, 1) * spline_x(t, 2)) / (
            (spline_x(t, 1) ** 2 + spline_y(t, 1) ** 2) ** (3 / 2)
        )

    def generate_raw_profile_data(
        self,
        spline_x: scipy.interpolate.PPoly,
        spline_y: scipy.interpolate.PPoly,
        num_waypoints: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """"""
        # Set up raw velocity profile
        velocity_profile = np.zeros(num_waypoints)  # [m/s]
        curvatures = np.zeros(num_waypoints)  # [rad/m]
        for i, t in enumerate(np.linspace(0, self.m, num_waypoints)):
            # Calculate curvature
            kappa = self.calculate_spline_curvature(spline_x, spline_y, t)
            curvatures[i] = kappa

            # Calculate velocity based off centripital acceleration
            v_x = np.sqrt(self.a_y_max / np.abs(kappa))

            # Clip the velocity
            v_x = np.clip(v_x, self.v_x_min, self.v_x_max)

            # Add the velocity to the profile
            velocity_profile[i] = v_x

        return (velocity_profile, curvatures)

    def calculate_accelerations(
        self, v_x: float, a_x_max: float, kappa: float
    ) -> Tuple[float, float]:
        """"""
        # Calculate lateral acceleration
        a_y = v_x**2 * np.abs(kappa)  # [m/s^2]
        a_y = np.min([self.a_y_max, a_y])

        # Calculate longitudinal accleration
        a_x = a_x_max * np.sqrt(1 - (a_y / self.a_y_max) ** 2)  # [m/s^2]

        return (a_x, a_y)

    def generate_forward_pass_profile(
        self, velocity_profile: np.ndarray, path: np.ndarray, curvatures: np.ndarray
    ) -> np.ndarray:
        """
        Based off forward algorithm for velocity profile computation in:
        https://github.com/TUMFTM/trajectory_planning_helpers/tree/master
        """
        # Get the number of waypoints
        num_waypoints = len(velocity_profile)

        # Find the indices where any accelerations begin
        acceleration_indices = np.where(np.diff(velocity_profile) > 0.0)[0]
        acceleration_start_indices = list(
            acceleration_indices[np.insert(np.diff(acceleration_indices), 0, 2) > 1]
        )

        # Loop through while we have segments of acceleration
        while acceleration_start_indices:
            # Grab the latest acceleration start index
            i = acceleration_start_indices.pop(0)

            # Loop through until we reach the end of the path
            while i < num_waypoints - 1:
                # Calculate accelerations
                a_x, a_y = self.calculate_accelerations(
                    velocity_profile[i], self.a_x_accel_max, curvatures[i]
                )

                # Calculate distance between waypoints
                dist = np.linalg.norm(path[:, i + 1] - path[:, i])

                # Calculate velocity at the next discretization point. If it is
                # larger than the current profile's velocity, keep the velocity
                # as is
                v_x_ip1 = np.sqrt(velocity_profile[i] ** 2 + 2 * a_x * dist)
                velocity_profile[i + 1] = np.min(
                    [
                        v_x_ip1,
                        velocity_profile[i + 1],
                    ]
                )

                # Incrememt the index
                i += 1

                # Break out of the loop if our calculated velocity would have been higher
                # than our maximum velocity or if we are going to move on to the next
                # acceleration start index
                if v_x_ip1 > self.v_x_max or (
                    acceleration_start_indices and i >= acceleration_start_indices[0]
                ):
                    break

        return velocity_profile

    def generate_backward_pass_profile(
        self, velocity_profile: np.ndarray, path: np.ndarray, curvatures: np.ndarray
    ) -> np.ndarray:
        """
        Based off backward algorithm for velocity profile computation in:
        https://github.com/TUMFTM/trajectory_planning_helpers/tree/master
        """
        # Find the indices where any decelerations begin. Reverse the list as we
        # will work through the profile backwards
        deceleration_indices = np.where(np.diff(velocity_profile) < 0.0)[0]
        deceleration_end_indices = list(
            deceleration_indices[np.insert(np.diff(deceleration_indices), -1, 2) > 1]
        )[::-1]

        # Loop through while we have segments of deceleration
        while deceleration_end_indices:
            # Grab the latest deceleration start index
            i = deceleration_end_indices.pop(0)

            # Loop through until we reach the end of the path
            while i > 0:
                # Calculate "accelerations"
                a_x, a_y = self.calculate_accelerations(
                    velocity_profile[i], self.a_x_decel_max, curvatures[i]
                )

                # Calculate distance between waypoints
                dist = np.linalg.norm(path[:, i + 1] - path[:, i])

                # Calculate velocity at the next discretization point. If it is
                # larger than the current profile's velocity, keep the velocity
                # as is
                v_x_ip1 = np.sqrt(velocity_profile[i] ** 2 + 2 * a_x * dist)
                velocity_profile[i - 1] = np.min(
                    [
                        v_x_ip1,
                        velocity_profile[i - 1],
                    ]
                )

                # Decrememt the index
                i -= 1

                # Break out of the loop if our calculated velocity would have been lower
                # than our minimum velocity or if we are going to move on to the next
                # deceleration start index
                if v_x_ip1 > self.v_x_max or (
                    deceleration_end_indices and i <= deceleration_end_indices[0]
                ):
                    break

        return velocity_profile


if __name__ == "__main__":

    # Define the corners of the square
    corners = np.array([[0, 0], [2, 0], [2, 2], [0, 2]])

    # Define the number of points between each corner
    num_points_between_corners = 10

    # Generate evenly spaced points between each corner
    path = np.empty((0, 2))  # Initialize empty array to store the path points

    for i in range(4):
        start_point = corners[i]
        end_point = corners[(i + 1) % 4]
        points_between = np.linspace(
            start_point, end_point, num_points_between_corners, endpoint=False
        )
        path = np.vstack((path, points_between))

    path = path.T

    trajectory_builder = TrajectoryBuilder(-0.01, 0.01, 2, 10)
    spline_x, spline_y, solution_dict = trajectory_builder.optimize_path(path)
    trajectory_builder.plot_path(spline_x, spline_y, path)
    path = trajectory_builder.discretize_spline(spline_x, spline_y, path.shape[1], 1000)
    velocity_profile = trajectory_builder.generate_raw_velocity_profile(
        spline_x, spline_y, path.shape[1], 1000
    )
    trajectory_builder.plot_path_with_profile(path, velocity_profile)
