import numpy as np
import registrator as reg

# Load the calibration data from the .npy file
calibration_data = np.load('calibration_data.npy', allow_pickle=True)

# Inspect the first data entry
first_entry = calibration_data[0]

psm_position = first_entry[0]  # PSM position (PyKDL.Frame object)
mtm_position = first_entry[1]  # MTM position (PyKDL.Frame object)

print("PSM Position:", psm_position)
print("MTM Position:", mtm_position)

# Extract translation (position) from PSM and MTM positions
psm_translation = psm_position.p
mtm_translation = mtm_position.p

# Extract rotation matrix from PSM and MTM positions
psm_rotation = psm_position.M
mtm_rotation = mtm_position.M

print("PSM Translation:", psm_translation)
print("MTM Translation:", mtm_translation)

# Calculate the difference in translation between PSM and MTM
translation_diff = psm_translation - mtm_translation
print("Translation Difference:", translation_diff)

analysis = input("Would you like to analyze the data? (y/n): ")

if analysis.lower() == 'y': # data (5,2)
    # do the calibration analysis
    # extract psm position
    psm_position = calibration_data[:, 0]

    # extract mtm position
    mtm_position = calibration_data[:, 1]

    # reigstration
    R, P, Q, Q_inv = reg.horn_trans(psm_position, mtm_position)

    print("Rotation Matrix:", R)
    print("Translation Vector:", P)

else:
    print("Exiting...")
