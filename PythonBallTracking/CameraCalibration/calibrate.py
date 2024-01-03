import numpy as np
import cv2
import glob

# Dimensions of the calibration pattern (number of inner corners)
pattern_size = (8, 6)

# Size of each square in the calibration pattern (in millimeters)
square_size = 25

# Termination criteria for calibration
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points in the calibration pattern's coordinate system
obj_points = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
obj_points[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

# Arrays to store object points and image points from all calibration images
calibration_obj_points = []
calibration_img_points = []

# Load calibration images
calibration_images = glob.glob("images/*.png")

# Get the frame size from the first calibration image
first_image = cv2.imread(calibration_images[0])
frame_size = first_image.shape[1::-1]

# Calculate the sensor size in millimeters
square_pixels = (pattern_size[0] - 1) * (pattern_size[1] - 1)
sensor_size_pixels = square_pixels * (square_size / square_pixels) ** 2
sensor_size_mm = np.sqrt(frame_size[0] * frame_size[1] * (square_size / sensor_size_pixels))
sensor_length = sensor_size_mm

for image_file in calibration_images:
    # Load the image
    img = cv2.imread(image_file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners in the image
    ret, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)

    # If corners are found, add object points and image points to the calibration arrays
    if ret:
        calibration_obj_points.append(obj_points)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        calibration_img_points.append(refined_corners)

        # Visualize the corners
        cv2.drawChessboardCorners(img, pattern_size, refined_corners, ret)
        cv2.imshow("Calibration Image", img)
        cv2.waitKey(500)

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    calibration_obj_points, calibration_img_points, frame_size, None, None
)

# Focal length calculation
focal_length = camera_matrix[0, 0]

# Print the camera matrix, distortion coefficients, focal length, and sensor length
print("Camera Matrix:")
print(camera_matrix)
print("\nDistortion Coefficients:")
print(dist_coeffs)
print("\nFocal Length:")
print(focal_length)
print("\nSensor Length (in millimeters):")
print(sensor_length)

# Save camera calibration parameters to a file
# np.savez("camera_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, focal_length=focal_length, sensor_length=sensor_length)
np.savez("../camera_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, focal_length=focal_length, sensor_length=sensor_length)

# Release the OpenCV windows
cv2.destroyAllWindows()

