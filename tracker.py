"""
A tracker class for controlling the Tello and some sample code for showing how
it works. you can test it using your webcam or a video file to make sure it works.

it computes a vector of the ball's direction from the center of the
screen. The axes are shown below (assuming a frame width and height of 600x400):
+y                 (0,200)


Y  (-300, 0)        (0,0)               (300,0)


-Y                 (0,-200)
-X                    X                    +X

Based on the tutorial:
https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

Usage:
for existing video:
python tracker.py --video ball_tracking_example.mp4
For live feed:
python tracking.py

@author Leonie Buckley and Jonathan Byrne
@copyright 2018 see license file for details
"""

# import the necessary packages
import argparse
import time
import cv2
import imutils
from imutils.video import VideoStream
import tellopy
import av
import numpy


def main():
    # Set up tello streaming
    drone = tellopy.Tello()
    drone.log.set_level(2)
    drone.connect()
    drone.start_video()

    # Open the video stream from Tello
    # @JEFF THIS IS WHERE YOU'LL NEED TO CHANGE THE FORMAT TO FEED INTO SLAM
    retry = 3
    container = None
    while retry > 0:
        try:
            container = av.open(drone.get_video_stream())
            break
        except av.AVError as e:
            print(f"Failed to open video stream, retries left: {retry}")
            retry -= 1
            time.sleep(1)

    if not container:
        print("Failed to open video stream from Tello drone.")
        return

    # Define the lower and upper boundaries of the "red" ball in HSV color space
    red_lower = (0, 100, 50)  # Lower bounds: Hue ~0, high saturation, low brightness
    red_upper = (10, 255, 150)  # Upper bounds: Slightly higher hue, full saturation, and medium brightness


    video_st = None

    # Initialize the tracker
    frame = get_frame(container, video_st)
    height, width = frame.shape[0], frame.shape[1]
    tracker = Tracker(height, width, red_lower, red_upper)

    try:
        # Process the video stream
        while True:
            frame = get_frame(container, video_st)
            if frame is None:
                break
            tracker.track(frame)
            frame = tracker.draw_arrows(frame)
            show(frame)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Gracefully stop the drone video stream and close OpenCV windows
        drone.quit()
        cv2.destroyAllWindows()



def get_frame(vid_stream, stream):
    """grab the current video frame"""
    try:
        for packet in vid_stream.demux((stream,)):
            for frame in packet.decode():
                # Convert frame to cv2-compatible image
                image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                # Resize for consistent processing
                image = imutils.resize(image, width=600)
                return image
    except av.AVError as e:
        print(f"Error retrieving frame: {e}")
    return None


def show(frame):
    """show the frame to cv2 window"""
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        exit()


class Tracker:
    """
    A basic color tracker, it will look for colors in a range and
    create an x and y offset valuefrom the midpoint
    """

    def __init__(self, height, width, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.midx = int(width / 2)
        self.midy = int(height / 2)
        self.xoffset = 0
        self.yoffset = 0

    def draw_arrows(self, frame):
        """Show the direction vector output in the cv2 window"""
        #cv2.putText(frame,"Color:", (0, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, thickness=2)
        cv2.arrowedLine(frame, (self.midx, self.midy),
                        (self.midx + self.xoffset, self.midy - self.yoffset),
                        (0, 0, 255), 5)
        return frame

    def track(self, frame):
        """Simple HSV color space tracking"""
        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                self.xoffset = int(center[0] - self.midx)
                self.yoffset = int(self.midy - center[1])
            else:
                self.xoffset = 0
                self.yoffset = 0
        else:
            self.xoffset = 0
            self.yoffset = 0
        return self.xoffset, self.yoffset

if __name__ == '__main__':
    main()
