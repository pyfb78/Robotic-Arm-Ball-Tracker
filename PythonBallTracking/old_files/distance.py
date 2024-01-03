import cv2
import numpy as np


def calculate_distance(pixel_diameter, focal_length_mm, real_diameter):
    """
    Calculate the distance to an object using the formula: distance = (real_diameter * focal_length_mm) / pixel_diameter
    :param pixel_diameter: Diameter of the object in pixels
    :param focal_length_mm: Focal length of the camera in millimeters
    :param real_diameter: Actual diameter of the object in meters
    :return: Distance to the object in millimeters
    """
    return round((real_diameter * focal_length_mm) / pixel_diameter, 6)


def filter_contours(contours, min_area, circularity_threshold):
    """
    Filter contours based on area and circularity
    :param contours: List of contours to filter
    :param min_area: Minimum area threshold
    :param circularity_threshold: Circularity threshold
    :return: Filtered contours
    """
    filtered_contours = []
    for contour in contours:
        # Calculate contour area
        area = cv2.contourArea(contour)
        if area > min_area:
            # Calculate circularity of the contour
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity > circularity_threshold:
                filtered_contours.append(contour)
    return filtered_contours


def main():
    # Load camera calibration parameters
    calibration_file = "camera_calibration.npz"
    calibration_data = np.load(calibration_file)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

    # Get the focal length in millimeters from the calibration data
    focal_length_mm = calibration_data["focal_length_mm"][0]

    # Create a VideoCapture object for the camera
    cap = cv2.VideoCapture(0)

    real_diameter = 0.04  # Actual diameter of the ping pong ball in meters

    # Set frame rate to 120 FPS
    cap.set(cv2.CAP_PROP_FPS, 120)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            break

        # Increase exposure
        cap.set(cv2.CAP_PROP_EXPOSURE, 0.5)

        # Undistort the frame using the calibration parameters
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Convert the frame to HSV color space
        img_hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

        # Threshold the image to isolate the ball
        orange_lower = np.array([0, 100, 100])
        orange_upper = np.array([30, 255, 255])

        mask_ball = cv2.inRange(img_hsv, orange_lower, orange_upper)

        # Apply morphological operations to remove noise
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_OPEN, kernel_open)
        mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_CLOSE, kernel_close)

        # Find contours of the ball
        contours, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size and circularity
        min_area = 1000  # Adjusted minimum area threshold
        circularity_threshold = 0.5  # Adjusted circularity threshold
        filtered_contours = filter_contours(contours, min_area, circularity_threshold)

        if len(filtered_contours) > 0:
            # Find the contour with the largest area (the ball)
            ball_contour = max(filtered_contours, key=cv2.contourArea)

            # Find the precise boundary of the ball
            epsilon = 0.001 * cv2.arcLength(ball_contour, True)
            ball_boundary = cv2.approxPolyDP(ball_contour, epsilon, True)

            if len(ball_boundary) > 2:
                # Draw the precise boundary of the ball
                cv2.drawContours(frame, [ball_boundary], -1, (0, 255, 0), 2)

                # Calculate the diameter of the ball
                (x, y), radius = cv2.minEnclosingCircle(ball_boundary)
                diameter = radius * 2

                # Calculate and print the distance to the ball
                distance = calculate_distance(diameter, focal_length_mm, real_diameter)
                print("Distance to the ball: {:.6f} mm".format(distance))

                # Calculate the x and y coordinates in the camera's image plane
                x_coordinate = (x - camera_matrix[0, 2]) / camera_matrix[0, 0]
                y_coordinate = -(y - camera_matrix[1, 2]) / camera_matrix[1, 1]
                x_coordinate = 1000 * round(x_coordinate, 5)
                y_coordinate = 1000 * round(y_coordinate, 5)
                print("x-coordinate: {:.5f}, y-coordinate: {:.5f}".format(x_coordinate, y_coordinate))

            # Display the frame
            cv2.imshow("Live Feed", frame)
        else:
            # Display the original frame if the ball is not detected
            cv2.imshow("Live Feed", frame)

        # Check for key press (press 'q' to exit)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture object and close windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
