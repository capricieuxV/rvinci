import numpy as np

"""
Created Sept. 30, 2023
@author: Shiyue(Vanessa) Wang, Yuxuan(Juno) Zhao
@summary: This is the method for point cloud registration
"""


def horn_trans(sta_set, end_set):
    """
    Compute the transformation between two sets of points using Horn's method.

    Parameters:
    - sta_set: The starting set of points.
    - end_set: The ending set of points.

    Returns:
    - R: Rotation matrix.
    - P: Translation vector.
    - Q: Transformation quaternion.
    - Q_inv: Inverse transformation quaternion.
    """
    # Compute centroid of the staring frame point cloud and the ending frame point cloud
    centroid_sta = np.mean(sta_set, axis=0)
    centroid_end = np.mean(end_set, axis=0)

    # Compute the new coordinates of point cloud in both staring and ending frames
    centered_sta = sta_set - centroid_sta
    centered_end = end_set - centroid_end

    # Compute covariance matrix
    H = np.dot(np.transpose(centered_end), centered_sta)

    # Singular Value Decomposition (SVD)
    U, _, VT = np.linalg.svd(H)

    # Compute the rotation matrix, R
    R = np.dot(VT.T, U.T).T

    # Compute the translation Vector, P
    P = centroid_end - np.dot(R, centroid_sta)

    # Construct transformation quaternion, Q
    Q = np.identity(4)
    Q[:3, :3] = R
    Q[:3, 3] = P

    Q_inv = np.identity(4)
    Q_inv[:3, :3] = np.transpose(R)
    Q_inv[:3, 3] = -np.matmul(np.transpose(R), P)

    return R, P, Q, Q_inv


# Example usage
if __name__ == "__main__":
    # Generate two sets of 3D points for demonstration
    set_A = np.random.rand(10, 3)  # Replace with your actual data
    set_B = np.random.rand(10, 3)  # Replace with your actual data
    print(set_A.shape, "\n", set_B.shape)

    # Apply Horn's method to align set_A with set_B
    [R, P, Q, Q_inv] = horn_trans(set_A, set_B)

    print(Q_inv)
    print(np.around(np.matmul(Q, Q_inv)))
