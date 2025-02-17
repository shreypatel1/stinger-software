'''
This node takes in camera image and identifies the pair of gates and the goal.

Search for TODO to complete this node.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Subscribers & publishers 
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.waypoint_pub = self.create_publisher(Point, '/gate_waypoint', 10)
        self.goal_pub = self.create_publisher(Bool, '/goal_found', 10)

        self.bridge = CvBridge()
        self.lidar_data = None
        self.get_logger().info("Vision node initialized.")

    def image_callback(self, msg):
        """Process the camera feed to detect red and green buoys."""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges in HSV
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        green_lower = np.array([40, 40, 40])
        green_upper = np.array([80, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        # TODO: Create masks
        red_mask = None
        green_mask = None
        yellow_mask = None

        # TODO: Write 'find_circles' to recognize color contour
        # Find contours for both colors
        red_buoy = self.find_circles(red_mask)
        green_buoy = self.find_circles(green_mask)

        # If both red & green buoys are detected, and the tug is right in front of the gate
        # Find the midpoint
        if red_buoy and green_buoy and self.lidar_data:
            self.publish_waypoint(red_buoy[0][0], green_buoy[0][0])
            
        # Check if the goal (yellow buoy) is in front
        if cv2.countNonZero(yellow_mask) > 50:  # If significant yellow pixels are detected
            self.get_logger().info("Goal Found!")
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_pub.publish(goal_msg)

    def lidar_callback(self, msg):
        """Store the latest LiDAR scan data."""
        self.lidar_data = msg.ranges

    def find_circles(self, mask):
        """TODO: Find circular contours in a binary mask."""
        detected = None # location in the formata: (int(x), int(y))
        return detected

    def publish_waypoint(self, red_x, green_x):
        """Check if the tug is approximately centered between the buoys."""
        mid_x = (red_x + green_x) / 2

        # Use LiDAR to measure the distance to each buoy
        red_distance = self.estimate_distance(red_x)
        green_distance = self.estimate_distance(green_x)

        if red_distance > 0 and green_distance > 0:
            error_margin = 0.2  # Adjust tolerance in distance measurement
            if abs(red_distance - green_distance) <= error_margin:
                # TODO: publish the waypoint
                pass

    def estimate_distance(self, x_position):
        """Estimate the distance of the detected object using LiDAR data."""
        if self.lidar_data:
            angle_index = int((x_position / 640) * len(self.lidar_data))  # Map pixel position to LiDAR index
            return self.lidar_data[angle_index]
        return -1.0  # No valid LiDAR reading

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
