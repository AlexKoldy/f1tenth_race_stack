import os

# Get the current directory of the script
current_directory = os.path.dirname(os.path.abspath(__file__))


# Define the destination directory
destination_directory = os.path.join(current_directory, "..", "trajectories")

# Ensure that the destination directory exists, if not, create it
if not os.path.exists(destination_directory):
    print("bruh")

print(destination_directory)

import numpy as np

A = np.zeros(4)

np.savez(destination_directory + "/test.npz", A=A)
