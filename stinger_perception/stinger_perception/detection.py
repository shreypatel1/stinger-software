'''
Camera specs: 
width: 1280
height: 720
30fps

ABS
Length Approx.6cm 2.36in
Pixel 1million pixels
Photosensitive chip: OV9726(1/6.5“)
Field of view: 63°No Distortion
Output:USB2.0
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from stinger_msgs.msg import Gate

class Detection(Node):
    def __init__(self):
        super().__init__("Detection_Node")

        self.image_width = 1280

        self.image_sub = self.create_subscription(Image, '/stinger/camera_0/image_raw', self.image_callback, 10)
        self.gate_pos_pub = self.create_publisher(Gate, '/stinger/gate_location', 10)
        self.bridge = CvBridge()
        self.hsv = np.array([])
        self.get_logger().info("Perception node initialized! Let there be light. You can see now.")

    def image_callback(self, msg):
        """Process the camera feed to detect red, green, and yellow buoys."""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # Opencv wants BGR, but ROS defaults RGB
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Separating Hue, Saturation, & Value isolates color regardless of lighting
        self.gate_detection_cv()

    def gate_detection_cv(self):
        """Publish the horizontal location of the gate"""
        
        # TODO: 6.1.a Understanding HSV
        # Define color ranges for red and green in HSV
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        if len(self.hsv) == 0:
            self.get_logger().info("Waitng for frame")
            return

        # TODO: 6.1.b Masking 
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        # Find contours for each color
        red_buoy_list = self.find_circles(red_mask)
        green_buoy_list = self.find_circles(green_mask)

        self.get_logger().info(f"Red buoys: {red_buoy_list}")
        self.get_logger().info(f"Green buoys: {green_buoy_list}")

        msg = Gate()
        if len(red_buoy_list) > 0:
            msg.red_x = float(red_buoy_list[0][0])
        else:
            msg.red_x = -1.0

        if len(green_buoy_list) > 0:
            msg.green_x = float(green_buoy_list[0][0])
        else:
            msg.green_x = -1.0
        
        self.gate_pos_pub.publish(msg)

    def find_circles(self, mask):
        """Find circular contours in a binary mask."""

        # TODO: 6.1.c Contours
        ### STUDENT CODE HERE

        ### END STUDENT CODE
        detected = []

        # TODO: 6.1.d Understanding and tune pixel radius
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            ### STUDENT CODE HERE

            ### END STUDENT CODE

        detected_sorted = sorted(detected, key=lambda x: x[2], reverse=True)
        return detected_sorted

rclpy.init()
detection_node = Detection()
rclpy.spin(detection_node)
detection_node.destroy_node()
rclpy.shutdown()