'''
This node takes in camera image and publish the 'detected object'
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/vision/detected_object', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY) # replace threshold number
        edges = cv2.Canny(thresh, 100, 200)

        # if edges ... 
        # thinking a green triangle
        detected_object = "triangle"
        self.publisher.publish(String(data=detected_object))
        # else: publish('looking for marker...')