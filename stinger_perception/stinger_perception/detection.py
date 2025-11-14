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

        self.image_width = 480

        self.image_sub = self.create_subscription(Image, '/stinger/camera_0/image_raw', self.image_callback, 10)
        self.gate_pos_pub = self.create_publisher(Gate, '/stinger/gate_location', 10)
        self.detection_img_pub = self.create_publisher(Image, '/stinger/detection_image', 10)
        self.bridge = CvBridge()
        self.hsv = None
        self.frame = None
        self.get_logger().info("Perception node initialized! Let there be light. You can see now.")

        # # TODO: 6.1.a Understanding HSV
        # # True or False, as my Value approaches 0, the color becomes darker.
        # self.question_1 = True
        # # True or False, as my Saturation increases, the color becomes whiter.
        # self.question_2 = False
        # # [0, 255), what hue value might cyan be.
        # self.question_3 = 90
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        self.red_lower = np.array([0, 100, 100], dtype=np.uint8)
        self.red_upper = np.array([10, 255, 255], dtype=np.uint8)
        self.red_lower2 = np.array([170, 100, 100], dtype=np.uint8)
        self.red_upper2 = np.array([179, 255, 255], dtype=np.uint8)
        self.green_lower = np.array([50, 50, 50], dtype=np.uint8)
        self.green_upper = np.array([75, 255, 255], dtype=np.uint8)

    def image_callback(self, msg):
        """Process the camera feed to detect red, green, and yellow buoys."""
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # Opencv wants BGR, but ROS defaults RGB
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) # Separating Hue, Saturation, & Value isolates color regardless of lighting
        self.gate_detection_cv()

    def gate_detection_cv(self):
        """Publish the horizontal location of the gate"""

        if len(self.hsv) == 0:
            self.get_logger().info("Waitng for frame")
            return

        # Init to zeros
        # red_mask = np.zeros_like(self.hsv)
        # green_mask = np.zeros_like(self.hsv)

        # TODO: 6.1.b Masking 
        ### STUDENT CODE HERE
        red_mask1 = cv2.inRange(self.hsv, self.red_lower, self.red_upper)
        red_mask2 = cv2.inRange(self.hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(self.hsv, self.green_lower, self.green_upper)
        ### END STUDENT CODE
        
        ### DEV
        # cv2.imshow("Red_mask", red_mask)
        # cv2.imshow("Green_mask", green_mask)
        # cv2.waitKey(1)

        # Find contours for each color
        red_buoy_list = self.find_circles(red_mask)
        green_buoy_list = self.find_circles(green_mask)

        # self.get_logger().info(f"Red buoys: {red_buoy_list}")
        # self.get_logger().info(f"Green buoys: {green_buoy_list}")

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
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ### END STUDENT CODE
        detected = []

        # TODO: 6.1.d Understanding and tuning pixel radius
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            # cv2.circle(self.frame, (int(x), int(y)), int(radius), (255, 0, 0), 3)
            ### STUDENT CODE HERE
            if radius >= 20: # Minimum radius threshold
                cv2.circle(self.frame, (int(x), int(y)), int(radius), (255, 0, 0), 3)
                detected.append((int(x), int(y), radius))
            ### END STUDENT CODE
        # cv2.imshow("original_frame", self.frame)
        self.detection_img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, 'bgr8'))
        detected_sorted = sorted(detected, key=lambda x: x[2], reverse=True)
        return detected_sorted


def main(args=None):
    rclpy.init()
    detection_node = Detection()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()