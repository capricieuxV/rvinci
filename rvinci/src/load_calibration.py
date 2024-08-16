import numpy as np

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
