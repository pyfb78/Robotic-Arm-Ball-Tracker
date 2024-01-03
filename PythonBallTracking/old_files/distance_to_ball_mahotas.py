import cv2
import numpy as np
from skimage import color, morphology, measure
from skimage.transform import rescale
from concurrent.futures import ThreadPoolExecutor

def calculate_distance(pixel_diameter, camera_matrix, real_diameter):
    # Calculate the distance using the formula: distance = (real_diameter * focal_length) / pixel_diameter
    focal_length = camera_matrix[0, 0]
    return round((real_diameter * focal_length) / pixel_diameter, 6)

def filter_contours(contours, min_area, circularity_threshold):
    filtered_contours = []
    for contour in contours:
        # Calculate contour area
        area = measure.label(contour).sum()
        if area > min_area:
            # Calculate circularity of the contour
            perimeter = measure.perimeter(contour)
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity > circularity_threshold:
                filtered_contours.append(contour)
    return filtered_contours

def process_frame(frame, camera_matrix, real_diameter, min_area, circularity_threshold):
    # Convert the frame to grayscale
    gray_frame = color.rgb2gray(frame)

    # Threshold the image to isolate the ball
    orange_lower = np.array([0, 100, 100])
    orange_upper = np.array([30, 255, 255])
    mask_ball = np.logical_and(gray_frame >= orange_lower[0], gray_frame <= orange_upper[0])

    # Apply morphological operations to remove noise
    mask_ball = morphology.opening(mask_ball, morphology.disk(5))
    mask_ball = morphology.closing(mask_ball, morphology.disk(15))

    # Find contours of the ball
    labeled, n_objects = measure.label(mask_ball, return_num=True)
    contours = measure.find_contours(labeled, 0.5)

    # Filter contours based on size and circularity
    filtered_contours = filter_contours(contours, min_area, circularity_threshold)

    if len(filtered_contours) > 0:
        # Find the contour with the largest area (the ball)
        ball_contour = max(filtered_contours, key=lambda c: measure.label(c).sum())

        # Find the minimum enclosing circle
        (y, x), radius = measure.regionprops(measure.label(ball_contour))[0].centroid, \
                         np.sqrt(measure.label(ball_contour).sum() / np.pi)

        if radius >= 10:  # Adjusted minimum enclosing circle radius threshold
            # Draw a circle around the ball
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

            # Calculate and print the distance to the ball
            diameter = radius * 2
            distance = calculate_distance(diameter, camera_matrix, real_diameter)
            print("Distance to the ball: {:.6f} meters".format(distance))

            # Calculate the x and y coordinates in the camera's image plane
            x_coordinate = (x - camera_matrix[0, 2]) / camera_matrix[0, 0]
            y_coordinate = (y - camera_matrix[1, 2]) / camera_matrix[1, 1]
            x_coordinate = round(x_coordinate, 5)
            y_coordinate = round(y_coordinate, 5)
            print("x-coordinate: {:.5f}, y-coordinate: {:.5f}".format(x_coordinate, y_coordinate))

    return frame

def main():
    # Load camera calibration parameters
    calibration_file = "camera_calibration.npz"
    calibration_data = np.load(calibration_file)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

    # Create a VideoCapture object for the camera
    cap = cv2.VideoCapture(0)

    real_diameter = 0.04  # Actual diameter of the ping pong ball in meters
    min_area = 1000  # Adjusted minimum area threshold
    circularity_threshold = 0.5  # Adjusted circularity threshold

    with ThreadPoolExecutor(max_workers=1) as executor:
        while True:
            # Read a frame from the camera
            ret, frame = cap.read()

            if not ret:
                break

            # Increase exposure
            cap.set(cv2.CAP_PROP_EXPOSURE, 0.5)

            # Rescale the frame to a lower resolution
            frame = rescale(frame, scale=0.5, channel_axis=2)

            # Process the frame in a separate thread
            future = executor.submit(process_frame, frame, camera_matrix, real_diameter, min_area, circularity_threshold)

            # Display the frame (without waiting for processing to finish)
            cv2.imshow("Live Feed", frame)

            # Wait for the processing to finish
            future.result()

            # Check for key press (press 'q' to exit)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Release the VideoCapture object and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

