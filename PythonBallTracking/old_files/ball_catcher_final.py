import cv2
import numpy as np
import time
import math
import serial
import serial.tools.list_ports


# Teensy USB serial microcontroller program id data:
VENDOR_ID = 5824
PRODUCT_ID = 1155
SERIAL_NUMBER = 14814510
SERIAL_BAUDRATE = 115200


# Serial packet flag bits
ARM_RESET_POSITION        = 0x80  # Reset to starting position
ARM_MAX_HEIGHT_POSITION   = 0x40  # Move steper 1 full height
ARM_FULL_FORWARD_POSITION = 0x20  # Move stepper 1 full forward
ARM_OPEN_PWMSERVO         = 0x10  # Open the PWMServo completely
ARM_CLOSE_PWMSERVO        = 0x08  # Close the PWMServo
ARM_ONLY_FLAG_VALID       = 0x04  # Use only flag value of Serial Packet
ARM_NOCHECK_MOVE          = 0x02  # Move without hardware checks
ARM_MOVE_ABSOLUTE         = 0x01  # Move absolute (default relative)


# At this distance the ARM needs to move fast
BALL_ACTIVE_DISTANCE0 = 1800  
BALL_ACTIVE_DISTANCE1 = 1400  
BALL_ACTIVE_DISTANCE2 = 1000  

ARM_MOVE_LARGE        = 500
ARM_MOVE_MEDIUM       = 300
ARM_MOVE_SMALL        = 100

# Some constants
# Camera picture resolution
pic_frame_xrange = 1920
pic_frame_yrange = 1080
pic_frame_xcenter = (pic_frame_xrange / 2)
pic_frame_ycenter = (pic_frame_yrange / 2)
pic_frame_area   = pic_frame_xrange * pic_frame_yrange

real_ball_diameter = 40 # in mm

far_vertical_displacement = 300  # in mm
far_hortizontal_displacement = 300 # in mm
far_measured_distance = 3150 # in mm

far_pixel_vertical = 165 # In pixels
far_pixel_horizontal = 165 # In pixels
far_ball_pixel_diameter = ((far_pixel_vertical / far_vertical_displacement) * real_ball_diameter)

near_vertical_displacement = 300  # in mm
near_hortizontal_displacement = 300 # in mm
near_measured_distance = 550 # in mm

near_pixel_vertical = 894 # In pixels
near_pixel_horizontal = 894 # In pixels
near_ball_pixel_diameter = ((near_pixel_vertical / near_vertical_displacement) * real_ball_diameter)

distance2ball_very_close = 49.0
distance2ball_very_far = (far_measured_distance + 1)

# State variables
previous_pixel_diameter = 0.0
previous_distance = distance2ball_very_far
previous_xdelta_pix = 0.0
previous_ydelta_pix = 0.0
previous_xtheta = 0.0
previous_ytheta = 0.0


steppe1_relative_move = 0 # Right Stepper
steppe2_relative_move = 0 # Base Stepper
steppe3_relative_move = 0 # Left Stepper


serial_port = getTeensyPort()
serial_port_handle = Serial(serial_port, SERIAL_BAUDRATE)
serial_port_handle.flush()


def getTeensyPort():
    for comport in serial.tools.list_ports.comports():
        vid = int(comport.vid)
        pid = int(comport.pid)
        sn  = int(comport.serial_number)
        if ((vid == VENDOR_ID) and (pid == PRODUCT_ID) and (sn == SERIAL_NUMBER)):
            return comport.device
    return ''


def printCOMPorts():
    for comport in serial.tools.list_ports.comports():
        vid = int(comport.vid)
        pid = int(comport.pid)
        sn  = int(comport.serial_number)
        print("COM Port:", vid, pid, sn)
    return
    

def serial_flush(sph):
    sph.flush()
    return


def serial_send_packet(sph, flag, n0, n1, n2):
    sph.write(bytearray('#', 'ascii'))
    sph.write(flag.to_bytes(2, byteorder='little', signed=True))
    sph.write(n0.to_bytes(4, byteorder='little', signed=True))
    sph.write(n1.to_bytes(4, byteorder='little', signed=True))
    sph.write(n2.to_bytes(4, byteorder='little', signed=True))
    sph.write(bytearray('$', 'ascii'))


def pwmservo_open_arms(sph):
    flag = ARM_OPEN_PWMSERVO | ARM_ONLY_FLAG_VALID
    serial_send_packet(sph, flag, 0, 0, 0)

    	
def pwmservo_close_arms(sph):
    flag = ARM_CLOSE_PWMSERVO | ARM_ONLY_FLAG_VALID
    serial_send_packet(sph, flag, 0, 0, 0)


def reset_arm_position(sph):
    flag = ARM_RESET_POSITION | ARM_ONLY_FLAG_VALID | ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, 0, 0, 0)


def arm_full_forward_position(sph):
    flag = ARM_FULL_FORWARD_POSITION | ARM_ONLY_FLAG_VALID | ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, 0, 0, 0)


def arm_max_height_position(sph):
    flag = ARM_MAX_HEIGHT_POSITION | ARM_ONLY_FLAG_VALID | ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, 0, 0, 0)


def arm_move_relative(sph, r, b, l):
    flag = 0 
    serial_send_packet(sph, flag, r, b, l)


def arm_move_nocheck_relative(sph, r, b, l):
    flag = ARM_NOCHECK_MOVE 
    serial_send_packet(sph, flag, r, b, l)


def arm_move_absolute(sph, r, b, l):
    flag = ARM_MOVE_ABSOLUTE 
    serial_send_packet(sph, flag, r, b, l)


def arm_move_nocheck_absolute(sph, r, b, l):
    flag = ARM_NOCHECK_MOVE | ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, r, b, l)


def move_camera_position(sph, pos):
    send_serial_packet(sph, pos)
    return


#
# Calculates the relative steps each stepper needs to make
#
def move_steppers_to_pic_center(sph, dis, xfac, yfac):
    step_factor = 10000.0 / dis
    xsteps = step_factor * xfac
    ysteps = step_factor * yfac
    steppe2_relative_move = int(xsteps)
    steppe3_relative_move = int(ysteps)
    if ((dis <= BALL_ACTIVE_DISTANCE0) && (dis > BALL_ACTIVE_DISTANCE1)):
        if (xfac > 0.2):
            steppe1_relative_move = -ARM_MOVE_SMALL # Move back
        if (xfac < -0.2):
            steppe1_relative_move = ARM_MOVE_SMALL  # Move forward
        arm_move_relative(sph, steppe1_relative_move, steppe2_relative_moveb, steppe3_relative_movel)
        return
    if ((dis <= BALL_ACTIVE_DISTANCE1) && (dis > BALL_ACTIVE_DISTANCE2)):
        if (xfac > 0.2):
            steppe1_relative_move = -ARM_MOVE_MEDIUM # Move back
        if (xfac < -0.2):
            steppe1_relative_move = ARM_MOVE_MEDIUM  # Move forward
        arm_move_relative(sph, steppe1_relative_move, steppe2_relative_moveb, steppe3_relative_movel)
        return
    if ((dis <= BALL_ACTIVE_DISTANCE1) && (dis > BALL_ACTIVE_DISTANCE2)):
        if (xfac > 0.2):
            steppe1_relative_move = -ARM_MOVE_LARGE # Move back
        if (xfac < -0.2):
            steppe1_relative_move = ARM_MOVE_LARGE  # Move forward
        arm_move_relative(sph, steppe1_relative_move, steppe2_relative_moveb, steppe3_relative_movel)
    return


# It returns the value in radians
def get_camera_rotation_angle(move_pixels, pixel_dia, ball_dist):
    delta = (real_ball_diameter / pixel_dia) * move_pixels
    theta = delta / ball_dist
    return theta


def calculate_distance_using_magnification(pixel_diameter):
    if pixel_diameter < far_ball_pixel_diameter:
        return distance2ball_very_far

    delta_pix = near_ball_pixel_diameter - far_ball_pixel_diameter
    delta_dist = far_measured_distance - near_measured_distance

    x = pixel_diameter - far_ball_pixel_diameter
    d = far_measured_distance - ((delta_dist / delta_pix) * x)
    return d


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

    # Initialize the serial port
    serial_port = getTeensyPort()
    print("Teensy Serial Port = ", serial_port)
    serial_port_handle = serial.Serial(serial_port, SERIAL_BAUDRATE)
    
    # Reset the position of robotic ARM
    reset_arm_position(serial_port_handle)

    # Define the upper and lower HSV color thresholds for the ball
    # We are using an orange ball for this project
    # Threshold the image to isolate the ball
    orange_lower = np.array([0, 100, 100])
    orange_upper = np.array([30, 255, 255])

    # Create a VideoCapture object for the camera
    # We use the camera 1 of the laptop
    # Note camera 0 is the builtin camera of the laptop
    cap = cv2.VideoCapture(1)

    # Start capturing the frames to track the ball
    while True:
        # Read the current frame from the camera
        ret, frame = cap.read()

        if not ret:
            # there is no frame, wait and continue
            time.sleep(2.0)
            continue

        # Move the ARM to max height position
        arm_max_height_position(serial_port_handle)

        # blur the image so that our ball will be more visible
        # using the kernel size of (11, 11) as it is good for this project
        blurred_frame = cv2.GaussianBlur(frame, (11, 11), 0)

        # Convert the frame to HSV color space the blurred frame
        frame_hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
        
        # now create a mask to filter out our orange ball
        mask_ball = cv2.inRange(frame_hsv, orange_lower, orange_upper)

        # Apply morphological operations to remove noise
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        # masking to focus on the ball colors
        #mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_OPEN, kernel_open, iterations=2)
        #mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        mask_ball = cv2.erode(mask_ball, None, iterations=2)
        mask_ball = cv2.dilate(mask_ball, None, iterations=2)

        # Find contours of the ball
        contours, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size and circularity
        min_area = 100  # Adjusted minimum area threshold
        circularity_threshold = 0.5  # Adjusted circularity threshold
        filtered_contours = filter_contours(contours, min_area, circularity_threshold)
        center = None

        # check if there is atleast one contour
        if len(filtered_contours) > 0:
            # Find the contour with the largest area (the ball)
            ball_contour = max(filtered_contours, key=cv2.contourArea)

            # Calculate the center, radius and diameter of the ball
            (x, y), radius = cv2.minEnclosingCircle(ball_boundary)
            diameter = radius * 2

            x_pix_delta = x - pic_frame_xcenter
            y_pix_delta = y - pic_frame_ycenter

            M = cv2.moments(ball_contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Calculate and print the distance to the ball
            distance = calculate_distance_using_magnification(diameter)
            print("Distance to the ball: {:.6f} mm".format(distance))

            x_factor = x_pix_delta / pic_frame_xrange
            y_factor = y_pix_delta / pic_frame_yrange

            move_steppers_to_pic_center(serial_port_handle, distance, x_factor, y_factor)

            # Save previous state of the ball

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

