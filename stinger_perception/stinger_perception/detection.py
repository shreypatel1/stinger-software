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
    
    def find_circles(self, mask):
        """Find circular contours in a binary mask."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if 20 < radius:  # Filter small objects (radius greater than 40 pixels rn, tune as you go)
                detected.append((int(x), int(y), int(radius)))

        detected_sorted = sorted(detected, key=lambda x: x[2], reverse=True)
        return detected_sorted

    def gate_detection_cv(self):
        """Publish the horizontal location of the gate"""
        # Define color ranges for red, green, and yellow in HSV
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        green_lower = np.array([60, 80, 80])
        green_upper = np.array([80, 255, 255])

        if len(self.hsv) == 0:
            self.get_logger().info("Waitng for frame")
            return

        # Create masks
        red_mask = cv2.inRange(self.hsv, red_lower, red_upper)
        green_mask = cv2.inRange(self.hsv, green_lower, green_upper)

        # Find contours for each color
        red_buoy_list = self.find_circles(red_mask)
        green_buoy_list = self.find_circles(green_mask)

        self.get_logger().info("I AM HERE!!!!!!")
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

rclpy.init()
detection_node = Detection()
rclpy.spin(detection_node)
detection_node.destroy_node()
rclpy.shutdown()