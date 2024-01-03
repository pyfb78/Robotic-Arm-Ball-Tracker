import cv2
import numpy as np

# Ball diameter in millimeters

sensor_length = 7
ball_diameter = 40

# Color range for ball detection in HSV
orange_lower = np.array([0, 120, 120], dtype=np.uint8)
orange_upper = np.array([20, 255, 255], dtype=np.uint8)

# Morphological kernel for noise removal
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

# List to store the tracked positions of the ball
history = []

# Inverse pixel size for distance calculation
pixel_size_inv = None


def track_ball(image):
    """
    Track the ball in the given image.
    :param image: Input image
    :return: Image with ball detection and trajectory
    """
    global pixel_size_inv

    # Convert image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the image to isolate the ball
    mask_ball = cv2.inRange(hsv_image, orange_lower, orange_upper)

    # Apply morphological operations to remove noise
    mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_OPEN, kernel, iterations=2)
    mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find contours of the ball
    contours, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Calculate contour area directly
        area = cv2.contourArea(contour)
        if area > 1000:
            # Calculate equivalent radius and center of the ball contour
            moments = cv2.moments(contour)
            center = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
            radius = int(np.sqrt(area / np.pi))

            # Draw a circle around the ball
            cv2.circle(image, center, radius, (0, 255, 0), 2)

            # Track the ball positions
            history.append(center)

            if len(history) >= 3:
                # Perform polynomial interpolation for trajectory prediction
                pos_list_x = [pt[0] for pt in history]
                pos_list_y = [pt[1] for pt in history]
                coeffs = np.polyfit(pos_list_x, pos_list_y, deg=2)
                poly = np.poly1d(coeffs)

                # Calculate the x values for the entire image width
                x_values = np.arange(image.shape[1])
                y_values = poly(x_values)

                # Draw the actual parabolic line
                pts = np.column_stack((x_values, y_values)).astype(np.int32)
                cv2.polylines(image, [pts], isClosed=False, color=(0, 0, 255), thickness=2)

    # Check if the ball is out of the frame
    if len(contours) == 0 and len(history) > 0:
        history.clear()  # Reset the history if ball is not detected

    return image


def calculate_inverse_pixel_size(image):
    """
    Calculate the inverse pixel size of the image.
    :param image: Input image
    """
    global pixel_size_inv
    if pixel_size_inv is None and image.shape[1] > 0:
        pixel_size_inv = sensor_length / image.shape[1]


def main():
    # Initialize the VideoCapture object for the camera
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            break

        # Track the ball in the frame
        frame = track_ball(frame)

        # Calculate the inverse pixel size
        calculate_inverse_pixel_size(frame)

        # Display the frame with ball detection and trajectory
        cv2.imshow("Ball Detection", frame)

        # Check for key press (press 'q' to exit)
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the VideoCapture object and close windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
