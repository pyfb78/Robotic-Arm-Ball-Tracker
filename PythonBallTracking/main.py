# Author: Pavan Yeddanapudi
#
# Ball tracker Python code
# This code works independently.
# It tracks the ball and moves the arm in that direction
# This code runs on the laptop connected to the ARM camera

from collections import deque
import cv2
import numpy as np
import time
import math
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt


# Teensy USB serial microcontroller program id data and baudrate
VENDOR_ID = 5824
PRODUCT_ID = 1155
SERIAL_NUMBER = 14814510
SERIAL_BAUDRATE = 115200


# Serial packet flag bits
# These bits are sent as flags in the serial packet to 
# Teensy board controling the ARM
ARM_RESET_POSITION        = 0x80  # Reset to starting position
ARM_MAX_HEIGHT_POSITION   = 0x40  # Move steper 1 full height
ARM_FULL_FORWARD_POSITION = 0x20  # Move stepper 1 full forward
ARM_OPEN_SERVO_JAWS       = 0x10  # Open the Servo completely
ARM_CLOSE_SERVO_JAWS      = 0x08  # Close the Servo
ARM_ONLY_FLAG_VALID       = 0x04  # Use only flag value of Serial Packet
ARM_NOCHECK_MOVE          = 0x02  # Move without hardware checks
ARM_MOVE_ABSOLUTE         = 0x01  # Move absolute (default relative)



# Some constants
# Camera picture resolution
pic_frame_xrange = 1920
pic_frame_yrange = 1080
pic_frame_xcenter = (pic_frame_xrange / 2)
pic_frame_ycenter = (pic_frame_yrange / 2)


# The real diameter of the ball used for calculating the
# distance based on the focal length of the camera
real_ball_diameter = 40.0 # in mm


# Steppers absolute position
# This the co-ordinate system of the steppers in the Python code
# the actual stepper postions in the hardware can be different
stepper1_absolute_position = 0 # Right Stepper
stepper2_absolute_position = 0 # Base Stepper
stepper3_absolute_position = 0 # Left Stepper


# This is used for delay to complete the previous ARM move
arm_command_ready_time = 0.0     # Time when the next command can be sent


# Serial packet number used for debugging
serial_packet_number = 0


#
# The Teensy port node
#
def getTeensyPort():
    return '/dev/cu.usbmodem148145101'


#
# Initialize the serial port
# We initialize the serial port first to send commands to the ARM
#
serial_port = getTeensyPort()
serial_port_handle = serial.Serial(serial_port, SERIAL_BAUDRATE)
serial_port_handle.flush()


#
# This function prints all the serial ports of the laptop
#
def printCOMPorts():
    for comport in serial.tools.list_ports.comports():
        vid = int(comport.vid)
        pid = int(comport.pid)
        sn  = int(comport.serial_number)
        print("COM Port:", vid, pid, sn)
    return


#
# this function flushes the data on serial port 'sph'
#
def serial_flush(sph):
    sph.flush()
    return


#
# this function increments the global serial packet number
#
def increment_and_serial_packet_number():
    global serial_packet_number
    serial_packet_number += 1
    print("Sent Packet = ", serial_packet_number)


#
# this function sends the packet to Teensy Robotic ARM
# all packets are of same size
# packet starts with a charecter '#' and ends with '$'
#
def serial_send_packet(sph, flag, n0, n1, n2, n3, n4, n5):
    sph.write(bytearray('#', 'ascii'))
    sph.write(flag.to_bytes(2, byteorder='little', signed=True))
    sph.write(n0.to_bytes(4, byteorder='little', signed=True))
    sph.write(n1.to_bytes(4, byteorder='little', signed=True))
    sph.write(n2.to_bytes(4, byteorder='little', signed=True))
    sph.write(n3.to_bytes(4, byteorder='little', signed=True))
    sph.write(n4.to_bytes(4, byteorder='little', signed=True))
    sph.write(n5.to_bytes(4, byteorder='little', signed=True))
    sph.write(bytearray('$', 'ascii'))
    increment_and_serial_packet_number()
    #print("a = ", n0, ", b = ", n1, ", c = ", n2)
    #print("x1 = ", n3, ", y1 = ", n4, ", z1 = ", n5)
    return


#
# This functions sends a serial packet to open the servo jaws
#
def servo_open_jaws(sph):
    flag = ARM_OPEN_SERVO_JAWS | ARM_ONLY_FLAG_VALID
    serial_send_packet(sph, flag, 0, 0, 0, 0, 0, 0)
    time.sleep(3.0)
    return


#
# This functions sends a serial packet to lose the servo jaws
#
def servo_close_jaws(sph):
    flag = ARM_CLOSE_SERVO_JAWS| ARM_ONLY_FLAG_VALID
    serial_send_packet(sph, flag, 0, 0, 0, 0, 0, 0)
    time.sleep(3.0)
    return


#
# this function moves the ARM to (r, b, l) position assuming the origin is at
# (p1, p2, p3). Both the oigin and the destination are send as a packet on the serial port
#
def arm_move_absolute(sph, p1, p2, p3, r, b, l):
    flag = ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, p1, p2, p3, r, b, l)
    return


#
# this function moves the ARM to (r, b, l) position assuming the origin is at
# (p1, p2, p3). Both the oigin and the destination are send as a packet on the serial port
# an extra flag is set to inform the ARM not to check the safety conditions
#
def arm_move_nocheck_absolute(sph, p1, p2, p3, r, b, l):
    flag = ARM_NOCHECK_MOVE | ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, p1, p2, p3, r, b, l)
    return


#
# Update ARM absolute position after the move
#
def update_absolute_move_position(abs_s1, abs_s2, abs_s3):
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position
    stepper1_absolute_position = abs_s1 # Right Stepper
    stepper2_absolute_position = abs_s2 # Base Stepper
    stepper3_absolute_position = abs_s3 # Left Stepper
    return



#
# Calculates the relative steps each stepper needs to make
#
def move_steppers_to_pic_center(sph, dis, xfac, yfac):
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position
    
    # these constant values are calculated based on the hardware
    stepper1_factor = 700000.0 / dis
    stepper2_factor = 700000.0 / dis
    stepper3_factor = 1000000.0 / dis
    xsteps = stepper2_factor * xfac
    ysteps = -stepper3_factor * yfac
    r = stepper1_absolute_position
    b = stepper2_absolute_position + int(xsteps)
    l = stepper3_absolute_position + int(ysteps)

    # Send the origin and the final position to the ARM
    arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
    time.sleep(abs((max(xsteps, ysteps)))/500.0) # delay so that the move completes
    update_absolute_move_position(r, b, l) # update the latest postion of the ARM
    return


#
# Calculates the relative steps each stepper needs to make
#
def move_steppers_to_pic_center_pix(sph, dis, xpix, ypix):
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position
    
    # these constant values are calculated based on the hardware
    stepper1_factor = 700000.0 / dis
    stepper2_factor = 700000.0 / dis
    stepper3_factor = 1000000.0 / dis
    xsteps = (stepper2_factor * xpix) / pic_frame_xrange
    ysteps = (-stepper3_factor * ypix) / pic_frame_yrange
    r = stepper1_absolute_position
    b = stepper2_absolute_position + int(xsteps)
    l = stepper3_absolute_position + int(ysteps)

    print("stepper2_absolute_position = ", stepper2_absolute_position)
    if (b < 0): 
        print("LEFT = ", b)
    else:
        print("RIGHT = ", b)

    # Send the origin and the final position to the ARM
    arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
    arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
    time.sleep(abs((max(xsteps, ysteps)))/1000.0) # delay so that the move completes
    update_absolute_move_position(r, b, l) # update the latest postion of the ARM
    return


#
# Returns the distance in mm
# This cde uses focal length
#
def calculate_distance(pixel_diameter, focal_length_mm, real_diameter):
    # Calculate the distance to an object using the formula: distance = (real_diameter * focal_length_mm) / pixel_diameter
    # :param pixel_diameter: Diameter of the object in pixels
    # :param focal_length_mm: Focal length of the camera in millimeters
    # :param real_diameter: Actual diameter of the object in millimeters
    # :return: Distance to the object in millimeters
    return round((real_diameter * focal_length_mm) / pixel_diameter, 6)


#
# Returns the potential ball contours in the picture frame
#
def filter_contours(contours, min_area, circularity_threshold):
    # Filter contours based on area and circularity
    # :param contours: List of contours to filter
    # :param min_area: Minimum area threshold
    # :param circularity_threshold: Circularity threshold
    # :return: Filtered contours
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


#
# This is the main function of ball tracking python code
#
def main():
    global real_ball_diameter
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position
    global serial_port_handle

    # Load camera calibration parameters
    calibration_file = "camera_calibration.npz"
    calibration_data = np.load(calibration_file)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

    # Get the focal length in millimeters from the calibration data
    focal_length_mm = calibration_data["focal_length_mm"][0]
    print("focal_length_mm = ", focal_length_mm)

    print("Teensy Serial Port = ", serial_port)

    # Create a VideoCapture object for the camera
    cap = cv2.VideoCapture(0)

    ball_active_time = time.time()

    while_cnt = 0
    # Start capturing the frames to track the ball

    while True:

        # Read the current frame from the camera
        ret, camera_frame = cap.read()

        if not ret:
            # there is no frame, wait and continue
            time.sleep(0.1)
            continue
        # Found a ball hence note the time
        ball_active_time = time.time()

        # We get inverted frame from the camera and it needs to be rotated
        # Rotate the image both vertically and horizontally
        frame = cv2.flip(camera_frame, -1)

        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Convert the frame to HSV color space
        img_hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

        # Threshold the image to isolate the ball

        # orange_lower = np.array([0, 100, 100])
        # orange_upper = np.array([30, 255, 255])
        # Values for identifying a green ball
        green_lower = np.array((40, 40, 40))
        green_upper = np.array((90, 255, 255))

        mask_ball = cv2.inRange(img_hsv, green_lower, green_upper)

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
        center = None

        # check if there is atleast one contour
        if len(filtered_contours) > 0:
            while_cnt += while_cnt + 1

            # Find the contour with the largest area (the ball)
            ball_contour = max(filtered_contours, key=cv2.contourArea)
            epsilon = 0.001 * cv2.arcLength(ball_contour, True)
            ball_boundary = cv2.approxPolyDP(ball_contour, epsilon, True)

            if len(ball_boundary) > 2:

                # Calculate the center, radius and diameter of the ball
                (x, y), radius = cv2.minEnclosingCircle(ball_boundary)
                diameter = radius * 2

                x_pix_delta = x - pic_frame_xcenter
                y_pix_delta = pic_frame_ycenter - y

                print("Ball x = ", x)
                print("x_pix_delta = ", x_pix_delta)

                M = cv2.moments(ball_contour)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # draw the circle and centroid on the frame,
                # then update the list of tracked points

                # Uncomment this code if you want a perfect circle around the ball
                # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # Draw the boundary around the ball as a polygon
                cv2.drawContours(frame, [ball_boundary], -1, (0, 255, 0), 2)
                # Calculate and print the distance to the ball
                distance = calculate_distance(diameter, focal_length_mm, real_ball_diameter)
                print("Distance to the ball: {:.6f} mm".format(distance))

                #x_factor = x_pix_delta / pic_frame_xrange
                #y_factor = y_pix_delta / pic_frame_yrange

                #move_steppers_to_pic_center(serial_port_handle, distance, x_factor, y_factor)

                # Move the ARM to the correct postion based on the frame
                move_steppers_to_pic_center_pix(serial_port_handle, distance, x_pix_delta, y_pix_delta)

            # Display the frame
            cv2.imshow("Live Feed", frame)
            if cv2.waitKey(1) == ord('q'):
                return
        else:
            curr_time = time.time()
            #if (curr_time - ball_active_time) > 10.0: # No Ball found for 10 sec
                #arm_ready = False
            # Display the original frame if the ball is not detected
            cv2.imshow("Live Feed", frame)
            if cv2.waitKey(1) == ord('q'):
                return

    # Release the VideoCapture object and close windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


