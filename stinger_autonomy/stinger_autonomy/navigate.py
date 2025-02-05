'''
This node sends thrust commands to the motors, avoid obstacles, and navigate to the goal position via waypointing

The tug starts at global position (0m,0m), with an orientation aligned with x-axis. 
The goal position is (4m,4m). 
The tug will need to utilize camera to find the pair of green gate to pass through to reach the goal position. 
The mid point of the pair of green gate is (2m, 4m). 
In this case, the tug following this trajectory is not allowed (0m,0m) -> (4m, 0m) -> (4m, 4m). 
It has to go through the pair of green gate. 
In front of the green gate, there is a red colored obstacle located at (1m, 4m). 
The tug needs to use the lidar to avoid the obstacle to get to the green gate. 
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Subscribers
        self.state_subscription = self.create_subscription(Odometry, '/localization/odometry', self.state_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Declare and get motor topics from params
        self.declare_parameter('motor_topic_left', 'thrusters/left/thrust')
        self.declare_parameter('motor_topic_right', 'thrusters/right/thrust')

        self.motor_topic_left = self.get_parameter('motor_topic_left').get_parameter_value().string_value
        self.motor_topic_right = self.get_parameter('motor_topic_right').get_parameter_value().string_value

        # Publishers for motor commands
        self.port_motor_publisher = self.create_publisher(Float64, self.motor_topic_left, 10)
        self.stbd_motor_publisher = self.create_publisher(Float64, self.motor_topic_right, 10)

        # PID Controller gains
        self.kp_linear = 1.0
        self.kp_angular = 2.5

        # Target waypoint (green gate midpoint)
        self.goal_position = (4.0, 4.0)
        self.gate_position = (2.0, 4.0)
        self.obstacle_position = (1.0, 4.0)

        # State
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0  # yaw angle in radians
        self.obstacle_detected = False

    def state_callback(self, msg: Odometry):
        # Extract current position and orientation (yaw)
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        self.navigate_to_goal()

    def lidar_callback(self, msg: LaserScan):
        # Detect obstacles within a certain distance threshold
        min_distance = min(msg.ranges)
        if min_distance < 1.5:  # Obstacle within 1.5m
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def navigate_to_goal(self):
        if self.obstacle_detected:
            self.avoid_obstacle()
            return

        goal_x, goal_y = self.gate_position if self.is_before_gate() else self.goal_position
        current_x, current_y = self.current_position

        # Compute errors
        distance_error = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        target_angle = math.atan2(goal_y - current_y, goal_x - current_x)
        angular_error = target_angle - self.current_orientation

        # Normalize angular error to [-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # Compute control signals
        linear_thrust = self.kp_linear * distance_error
        angular_thrust = self.kp_angular * angular_error

        # Apply thrust based on under-actuated dynamics
        port_thrust = max(min(linear_thrust - angular_thrust, 100.0), -100.0)
        stbd_thrust = max(min(linear_thrust + angular_thrust, 100.0), -100.0)

        self.send_motor_commands(port_thrust, stbd_thrust)

    def avoid_obstacle(self):
        self.get_logger().info("Obstacle detected, taking avoidance action...")
        # Simple avoidance maneuver: turn starboard to avoid obstacle
        self.send_motor_commands(-50.0, 50.0)  # Rotate right in place

    def send_motor_commands(self, port_thrust, stbd_thrust):
        self.port_motor_publisher.publish(Float64(data=port_thrust))
        self.stbd_motor_publisher.publish(Float64(data=stbd_thrust))

    def is_before_gate(self):
        # Check if the tug is before the gate based on x-coordinate
        return self.current_position[0] < self.gate_position[0]

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    # def state_callback(self, msg):
    #     position  = msg.pose.pose.position
    #     orientation = msg.pose.pose.orientation

    #     (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    #     if yaw<0:
    #         yaw = yaw % (-2*np.pi)
    #         yaw = 2*np.pi + yaw
    #     else:
    #         yaw = yaw % (+2*np.pi)
            
    #     self.heading = yaw
    #     self.x_loc_global = position.x
    #     self.y_loc_global = position.y
    #     self.full_odom = msg