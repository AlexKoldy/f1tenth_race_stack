import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate
from scipy.interpolate import CubicSpline
import scipy
import os

# Given waypoints
waypoints = np.array(
    [
        [250, 100],
        [159, 147],
        [80, 200],
        [130, 37],
        [250, 100],
    ]
)
# waypoints = np.array(
#     [
#         [61, 178],
#         [100, 50],
#         [125, 27],
#         [150, 33],
#         [245, 60],
#         [256, 86],
#         [250, 123],
#         # [226, 131],
#         # [166, 123],
#         # [147, 127],
#         # [136, 135],
#         # [120, 196],
#         # [102, 210],
#         # [75, 208],
#         # [61, 178],
#     ]
# )
# waypoints = np.array(
#     [
#         [61, 178],
#         [100, 50],
#         [125, 27],
#         [150, 33],
#         [245, 60],
#         [256, 95],
#         [210, 123],
#         [144, 133],
#         [110, 197],
#         [82, 210],
#         [65, 200],
#         [61, 178],
#     ]
# )


# Extract x and y coordinates
x = waypoints[:, 0]
y = waypoints[:, 1]

# # Parameterize t based on the number of waypoints
# t = np.linspace(0, 1, len(x))

# # Create a cubic spline for x and y separately
# spline_x = CubicSpline(t, x)
# spline_y = CubicSpline(t, y)

tck, u = scipy.interpolate.splprep(waypoints.T, u=None, s=0.0, per=1)
u_new = np.linspace(u.min(), u.max(), 1000)
x_new, y_new = scipy.interpolate.splev(u_new, tck, der=0)


# # Generate points along the spline using parameter t
# t_new = np.linspace(0, 1, 500)
# x_new = spline_x(t_new)
# y_new = spline_y(t_new)

current_directory = os.path.dirname(os.path.abspath(__file__))
trajectory_directory = os.path.join(current_directory, "..", "trajectories")
trajectory_directory = os.path.join(trajectory_directory, "race3")
bruh = np.array([x_new, y_new])


trajectory_save_file = "race3_raw_1.npz"
trajectory_save_file = os.path.join(trajectory_directory, trajectory_save_file)
np.savez(
    trajectory_save_file,
    path=bruh,
    velocity_profile=np.zeros_like(bruh[0]),
)

import cv2

image = cv2.imread(
    "/home/alko/ese6150/f1tenth_race_stack/maps/race3/race3_map_short.png", 0
)  # contours of the image


# Plot the original waypoints and the spline
plt.figure(figsize=(8, 6))
plt.plot(x, y, "o", label="Waypoints")
plt.plot(bruh[0, :], bruh[1, :], label="Spline")
plt.imshow(image, cmap="gray", origin="lower")
plt.title("Spline Interpolation of Waypoints")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid(True)
plt.show()


# import numpy as np
# from scipy.interpolate import CubicSpline
# import matplotlib.pyplot as plt


# class FunnySplines:
#     def __init__(self, plot_stuff: bool = False):
#         # Data points
#         data = np.array(
#             [
#                 [61, 178],
#                 [100, 50],
#                 [125, 27],
#                 [150, 33],
#                 [245, 60],
#                 [256, 86],
#                 [250, 123],
#                 [226, 131],
#                 [166, 123],
#                 [147, 127],
#                 [136, 135],
#                 [120, 196],
#                 [102, 210],
#                 [75, 208],
#             ]
#         )
#         # trajectory_load_file = "optimized_trajectory_13.npz"
#         import os

#         current_directory = os.path.dirname(os.path.abspath(__file__))
#         trajectory_directory = os.path.join(current_directory, "..", "trajectories")
#         trajectory_load_file = os.path.join(trajectory_directory, trajectory_load_file)

#         trajectory_data = np.load(trajectory_load_file)
#         # data = trajectory_data["path"]

#         # self.velocity_profile = trajectory_data["velocity_profile"]

#         # data = np.array(
#         #     [
#         #         [-4.67223, 3.96087],
#         #         [1.17708, -1.5418],
#         #         [2.65297, -1.80738],
#         #         [3.85163, -1.24358],
#         #         [5.24086, 1.45793],
#         #         [4.82124, 2.34273],
#         #         [4.05527, 2.45525],
#         #         [2.5122, 1.74302],
#         #         [0.593781, 1.83187],
#         #         [-0.889274, 3.50298],
#         #         [-2.10452, 5.25646],
#         #         [3.47531, 5.89926],
#         #         [-4.5715, 5.45313],
#         #         # [4.9281, 4.05209],
#         #         [-4.67223, 3.96087],
#         #     ]
#         # )

#         # Splitting x and y coordinates
#         # x = data[0, :]
#         # y = data[1, :]

#         # # # Define parameterization
#         # t = np.arange(len(data[0]))

#         # # # Fit cubic splines for x and y coordinates
#         # cs_x = CubicSpline(t, x)
#         # cs_y = CubicSpline(t, y)

#         # # # # New parameterization for smoother plotting
#         # t_smooth = np.linspace(0, len(data), 100)

#         # # # # Evaluate splines
#         # x_smooth = cs_x(t_smooth)
#         # y_smooth = cs_y(t_smooth)

#         x_smooth = data[0, :]
#         y_smooth = data[1, :]

#         # # self.path = np.array([x_smooth, y_smooth])
#         x = np.concatenate(
#             [
#                 np.array([-3.5]),
#                 x_smooth[0::30][1:-7],
#                 np.array([-4, -4.4, -3.85]),
#             ]
#         )
#         x[-3:-2] -= 0.5
#         y = np.concatenate(
#             [
#                 np.array([2.75]),
#                 y_smooth[0::30][1:-7],
#                 np.array([5.3, 4.5, y_smooth[0::30][-1]]),
#             ]
#         )
#         y[-3:-2] += 0.5

#         # # Define parameterization
#         t = np.arange(len(x))

#         # # Fit cubic splines for x and y coordinates
#         cs_x = CubicSpline(t, x)
#         cs_y = CubicSpline(t, y)

#         t_smooth = np.linspace(0, len(x) - 1, 50)

#         x_smooth = cs_x(t_smooth)[1:]
#         y_smooth = cs_y(t_smooth)[1:]

#         # x_smooth = np.concatenate([x_smooth, np.array([x_smooth[0] - 0.02])])
#         # y_smooth = np.concatenate([y_smooth, np.array([y_smooth[0] + 0.02])])

#         bruh = np.array([x_smooth, y_smooth])
#         trajectory_save_file = "raw_waypoints_8.npz"
#         trajectory_save_file = os.path.join(trajectory_directory, trajectory_save_file)
#         np.savez(
#             trajectory_save_file,
#             path=bruh,
#             velocity_profile=np.zeros_like(bruh[0]),
#         )

#         if plot_stuff:
#             # Plotting
#             plt.figure(figsize=(8, 6))
#             # plt.plot(x, y, "o", label="Data Points")
#             # plt.scatter(x_smooth, y_smooth, label="Piecewise Polynomial Spline")

#             # plt.scatter(
#             #     x_smooth[0::30][-8:],
#             #     y_smooth[0::30][-8:],
#             #     label="Piecewise Polynomial Spline",
#             # )
#             # plt.plot(
#             #     x_smooth[0::30][:-12],
#             #     y_smooth[0::30][:-12],
#             #     label="Piecewise Polynomial Spline",
#             # )
#             plt.scatter(x, y)
#             plt.scatter(x_smooth[-1], y_smooth[-1], label="Piecewise Polynomial Spline")
#             plt.plot(x_smooth, y_smooth, label="Piecewise Polynomial Spline")
#             plt.title("Piecewise Polynomial Spline Fit")
#             plt.xlabel("X")
#             plt.ylabel("Y")
#             plt.legend()
#             plt.grid(True)
#             plt.show()


# if __name__ == "__main__":
#     funny_spline = FunnySplines(True)
