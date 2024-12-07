#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
import base64
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
import time

class CameraControl:
    def __init__(self):
        # cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.image = None

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        try:
            # Convert the compressed image to a CV2 image
            np_arr = numpy.frombuffer(msg.data, numpy.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Encode the image as a base64 string in UTF-8
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.image = base64.b64encode(buffer).decode('utf-8')

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def capture_image(self, timeout=10):
        """
        Returns the current captured image as a base64 encoded string.
        
        :param timeout: Maximum time to wait for an image (in seconds).
        :return: Base64 encoded image string or None if no image is available.
        """
        start_time = time.time()
        while self.image is None:
            rospy.logwarn("No image captured yet. Retrying...")
            if time.time() - start_time > timeout:
                rospy.logerr("Image capture timed out.")
                return None
            time.sleep(0.5)
        return self.image
