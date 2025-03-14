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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
from sensor_msgs.msg import Imu
import tf_transformations

class State(Enum):
    Searching = 0
    Approaching = 1
    #TODO Define your states

class GateTask(Node):
    def __init__(self):
        super().__init__("Gate_Task")

        self.image_width = 1280
        self.hfov = 1.09956
        self.angular_correction_factor = 1.0

        self.previous_state = None
        self.state = State.Searching

        self.timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.pre_push_time = None # time difference from last push and now calls correct orientation state
        self.gate_offset = 0.0 # angle offset
        self.starting_quat = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_autonomy', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.imu_result = Imu()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.hsv = np.array([])
        self.get_logger().info("Vision node initialized! Let there be light. You can see now.")

    def image_callback(self, msg):
        """Process the camera feed to detect red, green, and yellow buoys."""
        # TODO convert to cv2 format and hsv
    
    def imu_callback(self, msg):
        self.imu_result = msg
    
    def find_circles(self, mask):
        """Find circular contours in a binary mask."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if 40 < radius:  # Filter small objects
                detected.append((int(x), int(y), int(radius)))

        detected_sorted = sorted(detected, key=lambda x: x[2], reverse=True)
        return detected_sorted

    def gate_detection_cv(self):
        # TODO Define color ranges for red, green, and yellow in HSV

        if len(self.hsv) == 0:
            self.get_logger().info("Waitng for frame")
            return [], []

        # TODO Create masks


        # Find contours for each color
        red_buoy = self.find_circles(red_mask)
        green_buoy = self.find_circles(green_mask)

        return red_buoy, green_buoy
    
    def calculate_gate_fov_bound(self, left_gate_x, right_gate_x):
        distance = 1
        if left_gate_x is not None and right_gate_x is not None:
            distance = (right_gate_x - left_gate_x) / self.image_width
        return distance
    
    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        # TODO state machine
        if cmd_vel is None:
            return
        self.cmd_vel_pub.publish(cmd_vel)

    def search(self):
        cmd_vel = Twist()
        left_gate_location, right_gate_location = self.gate_detection_cv()

        self.get_logger().info(f"{left_gate_location}")
        self.get_logger().info(f"{right_gate_location}")

        if len(left_gate_location)==0 or len(right_gate_location)==0:
            return
        
        deg_per_pixel = self.hfov / self.image_width

        left_gate_x = left_gate_location[0][0]
        right_gate_x = right_gate_location[0][0]

        mid_x = (right_gate_x + left_gate_x) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x

        # If we are far too left, want to turn right, this will be a negative value
        # If we are far too right, want to turn left, this will be a positive value
        turn_angle = diff_mid * deg_per_pixel

        self.get_logger().info(f"{turn_angle}")

        # Command to go in that direction
        cmd_vel.angular.z = self.angular_correction_factor * turn_angle

        if abs(turn_angle) < 0.1:
            self.state = State.Approaching
        return cmd_vel
    
    def approach(self):
        cmd_vel = Twist()
        left_gate_location, right_gate_location = self.gate_detection_cv()

        if len(left_gate_location)==0 or len(right_gate_location)==0:
            return
        
        deg_per_pixel = self.hfov / self.image_width
        left_gate_x = left_gate_location[0][0]
        right_gate_x = right_gate_location[0][0]

        mid_x = (right_gate_x + left_gate_x) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x
        turn_angle = diff_mid * deg_per_pixel

        #TODO, THIS MIGHT BE CURSED
        self.gate_offset = turn_angle

        cmd_vel.angular.z = self.angular_correction_factor * turn_angle
        cmd_vel.linear.x = 0.5
        gate_fov_bound = self.calculate_gate_fov_bound(left_gate_x, right_gate_x)
        
        self.get_logger().info(f"gate_fov_bound: {gate_fov_bound}")

        # The idea here, is that the closer we get to the gates, they will move closer towards the bounds of our FOV
        if gate_fov_bound > 0.7:
            self.state = State.Push
            self.pre_push_time = self.get_clock().now()
        return cmd_vel
    
    def pass_through(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        if (self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9 > 2:
            self.state = State.PassedThrough
        self.get_logger().info(f"{(self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9}")
        return cmd_vel
    
    def complete(self):
        self.get_logger().info("Gate successfully passed!")
        self.destroy_node()

rclpy.init()
gate_task = GateTask()
rclpy.spin(gate_task)
gate_task.destroy_node()
rclpy.shutdown()