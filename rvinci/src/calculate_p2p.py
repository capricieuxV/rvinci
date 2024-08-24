import numpy as np
import pandas as pd
import csv
import re
from datetime import datetime


class P2PRegistration:
    def __init__(self, source_points, target_points):
        self.source_points = source_points
        self.target_points = target_points

    def horn_trans(self):
        """
        Compute the transformation between two sets of points using Horn's method.

        Returns:
        - R: Rotation matrix.
        - T: Translation vector.
        - H: Transformation matrix (4x4 homogeneous).
        - H_inv: Inverse transformation matrix (4x4 homogeneous).
        """
        # Compute centroid of the source and target point clouds
        centroid_source = np.mean(self.source_points, axis=0)
        centroid_target = np.mean(self.target_points, axis=0)

        # Compute the new coordinates of point clouds in both source and target frames
        centered_source = self.source_points - centroid_source
        centered_target = self.target_points - centroid_target

        # Compute covariance matrix
        Cov = np.dot(np.transpose(centered_target), centered_source)

        # Singular Value Decomposition (SVD)
        U, _, VT = np.linalg.svd(Cov)

        # Compute the rotation matrix, R
        R = np.dot(VT.T, U.T).T

        # Compute the translation Vector, T
        T = centroid_target - np.dot(R, centroid_source)

        # Construct transformation matrix, H
        H = np.identity(4)
        H[:3, :3] = R
        H[:3, 3] = T

        # Compute inverse transformation matrix, H_inv
        H_inv = np.identity(4)
        H_inv[:3, :3] = np.transpose(R)
        H_inv[:3, 3] = -np.matmul(np.transpose(R), T)

        return R, T, H, H_inv

# Modify the data format into NumPy array structure 
def convert_to_array(matrix_str):
    
    matrix_str = matrix_str.replace(';', ',').replace('    ', '')
    numbers = list(map(float, re.findall(r"[-+]?\d*\.\d+|\d+", matrix_str)))
    matrix = np.array(numbers).reshape(4, 3)

    return matrix


if __name__ == "__main__":

    # load calibration data from csv file
    current_date = datetime.now().strftime("%m%d")
    pair_name = 'R1' # which pair fo calibration data (eg.MTML & PSM2 is 'L2')
    filepath = f"./data/{current_date}_cal_data_{pair_name}.csv"
    data = pd.read_csv(filepath)

    # initialize lists to hold data resprectively
    psm_rot = []
    psm_trans = []
    mtm_rot = []
    mtm_trans = []

    # set the iteration number the same as how you collect the calibration data
    n_iteration = 5

    # parse data
    for i in range(n_iteration):

        # extract psm data
        psm_position = data.iloc[i][0]
        psm_matrix = convert_to_array(psm_position)
        psm_rot.append(psm_matrix[:3, :3])
        psm_trans.append(psm_matrix[3, :])

    for i in range(n_iteration, n_iteration+n_iteration):

        # extract mtm data
        mtm_position = data.iloc[i][0]
        mtm_matrix = convert_to_array(mtm_position)
        mtm_rot.append(mtm_matrix[:3, :3])
        mtm_trans.append(mtm_matrix[3, :])

    # convert psm and mtm data into np array
    psm_rot = np.array(psm_rot)
    psm_trans = np.array(psm_trans)
    mtm_rot = np.array(mtm_rot)
    mtm_trans = np.array(mtm_trans)

    p2p_reg = P2PRegistration(psm_trans, mtm_trans)
    R, P, Q, Q_inv = p2p_reg.horn_trans()

    print(R)
    print('====================')
    print(P)
    print('====================')
    print(Q)
