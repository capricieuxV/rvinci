import numpy as np
from scipy.spatial.transform import Rotation as R

# Rotation matrix data
rotation_matrix_data = [
    0.99963039, -0.02577597, -0.00864171,
    0.0257793 ,  0.99966762,  0.00027365,
    0.00863178, -0.00049633,  0.99996262
]

# Define the rotation matrix
rotation_matrix = np.array([
    [0.99963039, -0.02577597, -0.00864171],
    [0.0257793 ,  0.99966762,  0.00027365],
    [0.00863178, -0.00049633,  0.99996262]
])

# Convert the rotation matrix to a quaternion
r = R.from_matrix(rotation_matrix)
quaternion = r.as_quat()  # This returns [x, y, z, w]

print("Quaternion (x, y, z, w):", quaternion)
