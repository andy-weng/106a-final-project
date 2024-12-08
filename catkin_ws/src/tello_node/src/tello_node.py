#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import numpy as np
import time

low_red = np.array([0, 0, 100])
high_red = np.array([20, 20, 190])

Kp = -0.5
Ki = 0.01 
Kd = 0.00

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
    prev_error = 0

    # tello.takeoff()
    tello.send_command_with_return("command")
    tello.send_command_with_return("takeoff")
    time.sleep(2)

    rate = rospy.Rate(10) 
    integral= 0

    while not rospy.is_shutdown():
        # Capture the frame from Tello's camera
        frame = tello.get_frame_read().frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(frame_rgb, low_red, high_red)
        result = cv2.bitwise_and(frame_rgb, frame_rgb, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_with_bbox = frame_rgb.copy()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Filter by area to ignore noise
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame_with_bbox, (x, y), (x + w, y + h), (0, 255, 0), 2)
                object_x, object_y = x + w // 2, y + h // 2
            else:
                object_x, object_y = None, None
        else:
            object_x, object_y = None, None

        ## Publish image with bounding box
        try:
            ros_image = bridge.cv2_to_imgmsg(frame_with_bbox, encoding="rgb8")
            image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

        ## Publish mask
        try:
            mask_image = bridge.cv2_to_imgmsg(result, encoding="rgb8")
            mask_pub.publish(mask_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

        ## PID Control 
        ## TO DO: Implement this in another node
        frame_center_x = frame_rgb.shape[1] // 2  # Center of the frame
        if object_x is not None:
            error = frame_center_x - object_x  # Error (difference from center)
            integral += error
            derivative = error - prev_error

            # Calculate PID output
            control_signal = Kp * error
            control_signal = int(np.clip(control_signal, -5, 5))
            rospy.loginfo(f"Object detected at X: {object_x}, Y: {object_y} with control signal: {control_signal}")
            # Send control signal to Tello
            tello.send_rc_control(int(control_signal), 0, 0, 0)
            prev_error = error
        else:
            # Stop movement if no object is detected
            #tello.send_rc_control(0, 0, 0, 0)
            print(f"Object not detected!")
            error, integral, derivative, prev_error = 0, 0, 0, 0

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
