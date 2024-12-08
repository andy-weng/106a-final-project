#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import numpy as np
import time
#tables rgb: [59 34 18], [ 3  7 16], [101 121 162], [ 40 110 212]
#cup's rgb: [ 36  92 213], [ 92 110 146], [  1  39 161], [  0  40 155], [  0  74 164], [  0  64 183], [  0  56 183], [ 35 112 218], [  0  51 159], [  0  51 161],  [  0  65 182]

low_red = np.array([0, 0, 100])
high_red = np.array([20, 20, 190])

# Orange obstacle detection thresholds (in HSV)
low_orange = np.array([112, 148, 146])    # Lower orange boundary
high_orange = np.array([116, 255, 220])  # Upper orange boundary

# PID constants for left/right movement
Kp_lr = -0.5
Ki_lr = 0.01 
Kd_lr = 0.00

# PID constants for forward/backward movement
Kp_fb = 0.001  # Smaller value for smoother approach
Ki_fb = 0.0001
Kd_fb = 0.00

# Area thresholds
SAFE_LANDING_AREA = 40000  # Area threshold for landing
APPROACH_AREA = 20000  # Area threshold to slow down
FORWARD_SPEED = 15

# Add this function before tello_video_stream()
def mouse_callback(event, x, y, flags, param):
    """
    Mouse callback function to print RGB values when clicking on the image
    """
    if event == cv2.EVENT_LBUTTONDOWN:  # Left click
        frame = param
        rgb_values = frame[y, x]
        print(f"RGB values at position ({x}, {y}): {rgb_values}")


def detect_orange_obstacles(frame_rgb):
    """
    Detect orange cups as obstacles using HSV color space
    """
    # Convert to HSV for better color detection
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    
    # Create mask for orange objects
    orange_mask = cv2.inRange(hsv, low_orange, high_orange)
    
    # Enhanced noise removal
    kernel = np.ones((5,5), np.uint8)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    orange_mask = cv2.dilate(orange_mask, kernel, iterations=1)
    
    # Find contours with better filtering
    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    obstacles = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Increased minimum area threshold
            x, y, w, h = cv2.boundingRect(contour)
            obstacles.append((x + w//2, y + h//2))
            cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 165, 255), 2)
    
    return obstacles

def modify_speed_based_on_obstacles(object_x, obstacles, fb_control, safety_margin=100):
    """
    Modify forward/backward speed based on proximity to obstacles.
    """
    if not obstacles:
        return fb_control

    closest_distance = float('inf')
    for obs_x, _ in obstacles:
        distance = abs(object_x - obs_x)
        if distance < closest_distance:
            closest_distance = distance

    if closest_distance < safety_margin:
        fb_control = max(1, fb_control // 2)
    
    return fb_control

def tello_video_stream():
    # Initialize the ROS node
    rospy.init_node('tello_video_stream', anonymous=True)

    # Set up the publisher for video stream
    image_pub = rospy.Publisher('/tello/image_raw', Image, queue_size=10)
    mask_pub = rospy.Publisher('/tello/mask', Image, queue_size=10)

    # Create a CVBridge to convert OpenCV images to ROS Image messages
    bridge = CvBridge()

    # Initialize Tello drone connection
    tello = Tello()
    tello.connect()
    rospy.loginfo(f"Tello Battery: {tello.get_battery()}%")

    # Start the video stream
    tello.streamon()
    
    # Initialize PID variables
    prev_error_lr = 0
    integral_lr = 0
    prev_error_fb = 0
    integral_fb = 0

    # Initialize drone
    tello.send_command_with_return("command")
    # tello.send_command_with_return("takeoff")
    time.sleep(2)

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        # Capture the frame from Tello's camera
        frame = tello.get_frame_read().frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(frame_rgb, low_red, high_red)
        result = cv2.bitwise_and(frame_rgb, frame_rgb, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_with_bbox = frame_rgb.copy()
        cv2.namedWindow('Video Feed')
        cv2.setMouseCallback('Video Feed', mouse_callback, frame_rgb)
        
        # Display the frame
        cv2.imshow('Video Feed', frame_rgb)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Filter by area to ignore noise
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame_with_bbox, (x, y), (x + w, y + h), (0, 255, 0), 2)
                object_x, object_y = x + w // 2, y + h // 2
                current_area = w * h
            else:
                object_x, object_y = None, None
                current_area = 0
        else:
            object_x, object_y = None, None
            current_area = 0

         # Detect orange obstacles
        obstacles = detect_orange_obstacles(frame_with_bbox)

        # Publish image with bounding box
        try:
            ros_image = bridge.cv2_to_imgmsg(frame_with_bbox, encoding="rgb8")
            image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

        # Publish mask
        try:
            mask_image = bridge.cv2_to_imgmsg(result, encoding="rgb8")
            mask_pub.publish(mask_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

        # PID Control
        frame_center_x = frame_rgb.shape[1] // 2  # Center of the frame
        
        if object_x is not None:
            # Left/Right PID Control
            error_lr = frame_center_x - object_x
            integral_lr += error_lr
            derivative_lr = error_lr - prev_error_lr
            lr_control = int(np.clip((Kp_lr * error_lr + Ki_lr * integral_lr + Kd_lr * derivative_lr), -20, 20))
            
            if current_area >= SAFE_LANDING_AREA:
                rospy.loginfo("Safe landing distance reached - Landing")
                tello.land()
                break
            elif current_area >= APPROACH_AREA:
                fb_control = 5
            else:
                fb_control = FORWARD_SPEED

            # Modify speed based on obstacles
            fb_control = modify_speed_based_on_obstacles(object_x, obstacles, fb_control)

            rospy.loginfo(f"Object detected at X: {object_x}, Y: {object_y}")
            rospy.loginfo(f"Current Area: {current_area}")
            rospy.loginfo(f"Control signals - LR: {lr_control}, FB: {fb_control}")
            rospy.loginfo(f"Detected Obstacles: {len(obstacles)}")
            
            # Send control signals to Tello
            # tello.send_rc_control(lr_control, fb_control, 0, 0)
            
            # Update previous errors
            prev_error_lr = error_lr
            
        else:
            # Stop movement if no object is detected
            tello.send_rc_control(0, 0, 0, 0)
            rospy.loginfo("Object not detected!")
            # Reset PID variables
            integral_lr = 0
            integral_fb = 0
            prev_error_lr = 0
            prev_error_fb = 0

        rate.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            tello.land()
            break

    tello.streamoff()

if __name__ == '__main__':
    try:
        tello_video_stream()
    except rospy.ROSInterruptException:
        pass