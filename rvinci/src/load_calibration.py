import numpy as np

# Load the calibration data from the .npy file
calibration_data = np.load('calibration_data.npy', allow_pickle=True)

# Inspect the first data entry
first_entry = calibration_data[0]

psm_position = first_entry[0]  # PSM position (PyKDL.Frame object)
mtm_position = first_entry[1]  # MTM position (PyKDL.Frame object)
left_image = first_entry[2]    # Left endoscope image (OpenCV image)
right_image = first_entry[3]   # Right endoscope image (OpenCV image)

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

import cv2
import matplotlib.pyplot as plt

# Display the left and right images
plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB))
plt.title('Left Image')

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB))
plt.title('Right Image')

plt.show()

# Convert images to grayscale
left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

# Save the processed grayscale images
cv2.imwrite('left_image_gray.png', left_gray)
cv2.imwrite('right_image_gray.png', right_gray)

# Calculate the difference in translation between PSM and MTM
translation_diff = psm_translation - mtm_translation
print("Translation Difference:", translation_diff)

