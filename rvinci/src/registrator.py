import numpy as np
import csv
from datetime import datetime

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

def parse_matrix_from_string(matrix_str):
    """
    Parse a matrix from a string and convert it into a NumPy array.
    
    Parameters:
    - matrix_str: The matrix as a string.
    
    Returns:
    - matrix: The parsed matrix as a NumPy array.
    """
    # Remove brackets and newlines, replace semicolons with commas
    matrix_str = matrix_str.replace("[", "").replace("]", "").replace("\n", "").replace(";", ",")
    
    # Split the string into rows
    rows = matrix_str.split(",")
    
    # Convert the rows into a list of floats
    matrix = [list(map(float, row.split())) for row in rows if row.strip()]
    
    # Reshape into a 4x4 matrix
    if len(matrix) == 4:
        return np.array(matrix).reshape((4, 4))
    else:
        return np.array(matrix)

def load_calibration_data(csv_filename):
    """
    Load PSM and MTM positions from the calibration CSV file.
    
    Parameters:
    - csv_filename: Path to the CSV file.

    Returns:
    - psm_positions: List of PSM positions as numpy arrays.
    - mtm_positions: List of MTM positions as numpy arrays.
    """
    psm_positions = []
    mtm_positions = []
    
    with open(csv_filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row
        for i, row in enumerate(reader):
            if i < 5:
                psm_position = parse_matrix_from_string(row[0])
                psm_positions.append(psm_position)
            else:
                mtm_position = parse_matrix_from_string(row[0])
                mtm_positions.append(mtm_position)
    
    return np.array(psm_positions), np.array(mtm_positions)

if __name__ == "__main__":
    # Load the calibration data from CSV
    current_date = datetime.now().strftime("%m%d")
    filename = f"{current_date}_cal_data_L2.csv"
   
    psm_positions, mtm_positions = load_calibration_data(filename)
    
    # Extract only the translation components for Horn's method
    psm_translations = np.array([pos[1] for pos in psm_positions])
    mtm_translations = np.array([pos[1] for pos in mtm_positions])

    # Apply Horn's method to align PSM2 positions with MTML positions
    R, P, Q, Q_inv = horn_trans(psm_translations, mtm_translations)

    # Print the results
    print("Rotation Matrix (R):")
    print(R)
    print("\nTranslation Vector (P):")
    print(P)
    print("\nTransformation Matrix (Q):")
    print(Q)
    print("\nInverse Transformation Matrix (Q_inv):")
    print(Q_inv)
