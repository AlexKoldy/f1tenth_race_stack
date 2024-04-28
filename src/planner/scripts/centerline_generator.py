import cv2
import numpy as np
from matplotlib import pyplot as plt

# Read the image as a grayscale image
image = cv2.imread(
    "/home/alko/ese6150/f1tenth_race_stack/maps/race3/race3_map_short.png", 0
)  # contours of the image
contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
# Select white contours
image = cv2.inRange(image, 240, 255)
# Thin the contours of the image image
center_line = cv2.ximgproc.thinning(image)
points = np.argwhere(center_line > 0)

# points = points[::10, :]
# print(points.shape)
# for _ in range(100):
#     i = np.random.randint(0, 65)
#     print(i)
#     plt.imshow(center_line, cmap="gray")
#     plt.plot(points[i, 1], points[i, 0], "ro")
#     # plt.scatter(points[::10, 1], points[::10, 0])
#     plt.gca().invert_yaxis()
#     plt.show()

import os

current_directory = os.path.dirname(os.path.abspath(__file__))
trajectory_directory = os.path.join(current_directory, "..", "trajectories")
trajectory_directory = os.path.join(trajectory_directory, "race3")
trajectory_load_file = os.path.join(trajectory_directory, "race3_raw_1.npz")

trajectory_data = np.load(trajectory_load_file)
data = trajectory_data["path"]
print(data.shape)

# Overlaying the center line on the original image
center_line_inv = cv2.bitwise_not(center_line)
result = cv2.bitwise_and(image, image, mask=center_line_inv)
plt.imshow(result, cmap="gray", origin="lower")
plt.plot(data[0, :], data[1, :])
plt.show()
