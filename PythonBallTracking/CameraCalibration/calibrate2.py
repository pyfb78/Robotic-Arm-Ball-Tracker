import cv2
import numpy as np
import glob

# Define the chessboard dimensions
chessboard_rows = 8
chessboard_cols = 6
square_size_mm = 20

# Prepare object points (0,0,0), (1,0,0), ..., (7,5,0)
objp = np.zeros((chessboard_rows * chessboard_cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_cols, 0:chessboard_rows].T.reshape(-1, 2)
objp = objp * square_size_mm

# Arrays to store object points and image points from all the images
obj_points = []  # 3D points in real-world space
img_points = []  # 2D points in image plane

# Load images and find chessboard corners
image_paths = glob.glob("images/*.png")
for image_path in image_paths:
    image = cv2.imread(image_path)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray_image, (chessboard_cols, chessboard_rows), None)

    if ret:
        obj_points.append(objp)
        img_points.append(corners)

        # Optionally, you can draw the chessboard corners on the image
        cv2.drawChessboardCorners(image, (chessboard_cols, chessboard_rows), corners, ret)
        cv2.imshow("Image", image)
        cv2.waitKey(500)  # Show each image for 500 milliseconds

cv2.destroyAllWindows()

# Calibrate the camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray_image.shape[::-1], None, None)

# Save the calibration data to a file
calibration_data = {
    'camera_matrix': camera_matrix,
    'dist_coeffs': dist_coeffs,
    'focal_length_mm': [camera_matrix[0, 0], camera_matrix[1, 1]],
    'sensor_size_mm': [gray_image.shape[1] * square_size_mm, gray_image.shape[0] * square_size_mm],
}

np.savez('../camera_calibration.npz', **calibration_data)

# Print the calibration data
print("Camera Matrix:")
print(camera_matrix)

print("\nDistortion Coefficients:")
print(dist_coeffs)

print("\nFocal Length (mm):")
print([camera_matrix[0, 0], camera_matrix[1, 1]])

print("\nSensor Size (mm):")
print([gray_image.shape[1] * square_size_mm, gray_image.shape[0] * square_size_mm])

print("\nCamera calibration completed and data saved to camera_calibration.npz")

