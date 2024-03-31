import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


class FunnySplines:
    def __init__(self, plot_stuff: bool = False):
        # Data points
        data = np.array(
            [
                [-2.59579, 5.10734],
                [-3.13684, 5.55813],
                [-3.52349, 5.68366],
                [-4.14947, 5.69393],
                [-4.82913, 5.28649],
                [-4.9403, 4.28687],
                [-4.32566, 3.6492],
                [-3.83496, 3.11253],
                [-2.97029, 2.39227],
                [-2.22876, 1.69494],
                [-1.42675, 1.16838],
                [-0.740714, 0.574631],
                [0.0214173, -0.0569689],
                [0.417583, -0.370999],
                [1.20481, -0.954837],
                [1.84598, -1.6679],
                [2.67081, -1.7596],
                [3.64857, -1.39936],
                [4.32032, -0.499779],
                [5.12331, 0.382067],
                [5.6881, 1.35298],
                [5.61442, 2.34968],
                [5.33962, 2.81398],
                [4.69962, 3.08666],
                [4.18221, 3.05498],
                [3.56644, 2.49632],
                [2.90545, 1.83064],
                [2.07986, 1.50602],
                [1.26531, 1.85907],
                [0.497527, 2.48851],
                [-0.258423, 3.13188],
                [-2.59579, 5.10734],
            ]
        )
        trajectory_load_file = "optimized_trajectory_7.npz"
        import os

        current_directory = os.path.dirname(os.path.abspath(__file__))
        trajectory_directory = os.path.join(current_directory, "..", "trajectories")
        trajectory_load_file = os.path.join(trajectory_directory, trajectory_load_file)

        trajectory_data = np.load(trajectory_load_file)
        data = trajectory_data["path"]

        # self.velocity_profile = trajectory_data["velocity_profile"]

        # data = np.array(
        #     [
        #         [-4.67223, 3.96087],
        #         [1.17708, -1.5418],
        #         [2.65297, -1.80738],
        #         [3.85163, -1.24358],
        #         [5.24086, 1.45793],
        #         [4.82124, 2.34273],
        #         [4.05527, 2.45525],
        #         [2.5122, 1.74302],
        #         [0.593781, 1.83187],
        #         [-0.889274, 3.50298],
        #         [-2.10452, 5.25646],
        #         [3.47531, 5.89926],
        #         [-4.5715, 5.45313],
        #         # [4.9281, 4.05209],
        #         [-4.67223, 3.96087],
        #     ]
        # )

        # Splitting x and y coordinates
        x = data[:, 0]
        y = data[:, 1]

        # Define parameterization
        t = np.arange(len(data))

        # Fit cubic splines for x and y coordinates
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)

        # New parameterization for smoother plotting
        t_smooth = np.linspace(0, len(data) - 1, 100)

        # Evaluate splines
        x_smooth = cs_x(t_smooth)
        y_smooth = cs_y(t_smooth)

        x_smooth = data[0, :]
        y_smooth = data[1, :]

        # self.path = np.array([x_smooth, y_smooth])
        x = np.concatenate(
            [x_smooth[0::30][:-12], np.array([1, 0, -2]), x_smooth[0::30][-7:]]
        )
        y = np.concatenate(
            [y_smooth[0::30][:-12], np.array([2.75, 3.5, 5]), y_smooth[0::30][-7:]]
        )

        # Define parameterization
        t = np.arange(len(x))

        # Fit cubic splines for x and y coordinates
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)

        t_smooth = np.linspace(0, len(x) - 1, 100)

        x_smooth = cs_x(t_smooth)
        y_smooth = cs_y(t_smooth)

        bruh = np.array([x_smooth, y_smooth])
        trajectory_save_file = "raw_waypoints_3.npz"
        trajectory_save_file = os.path.join(trajectory_directory, trajectory_save_file)
        np.savez(
            trajectory_save_file,
            path=bruh,
            velocity_profile=np.zeros_like(bruh[0]),
        )

        if plot_stuff:
            # Plotting
            plt.figure(figsize=(8, 6))
            # plt.plot(x, y, "o", label="Data Points")
            # plt.plot(x_smooth, y_smooth, label="Piecewise Polynomial Spline")
            # plt.plot(
            #     x_smooth[0::30][-12:-7],
            #     y_smooth[0::30][-12:-7],
            #     label="Piecewise Polynomial Spline",
            # )
            # plt.plot(
            #     x_smooth[0::30][:-12],
            #     y_smooth[0::30][:-12],
            #     label="Piecewise Polynomial Spline",
            # )
            plt.scatter(x, y)
            plt.plot(x_smooth, y_smooth, label="Piecewise Polynomial Spline")
            plt.title("Piecewise Polynomial Spline Fit")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.legend()
            plt.grid(True)
            plt.show()


if __name__ == "__main__":
    funny_spline = FunnySplines(True)
