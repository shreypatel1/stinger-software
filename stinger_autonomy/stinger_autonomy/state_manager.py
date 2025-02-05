''''
TODO: crappy code warning
'''


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        # State machine states
        self.state = "SEARCH_GATE"  # Initial state
        self.goal_position = (4.0, 4.0)

        # Subscribers
        self.create_subscription(Point, '/detected_gate', self.gate_callback, 10)
        self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)

        # Publisher for state information
        self.state_publisher = self.create_publisher(String, '/state', 10)

        self.gate_midpoint = None
        self.gate_crossed = False

    def gate_callback(self, msg: Point):
        # Update the detected gate midpoint position
        self.gate_midpoint = (msg.x, msg.y)
        if self.state == "SEARCH_GATE":
            self.get_logger().info("Gate detected, switching to NAVIGATE_TO_GATE")
            self.state = "NAVIGATE_TO_GATE"
            self.publish_state()

    def odom_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # State transitions
        if self.state == "NAVIGATE_TO_GATE" and self.gate_midpoint:
            gate_x, gate_y = self.gate_midpoint
            if abs(current_x - gate_x) < 0.5 and abs(current_y - gate_y) < 0.5:
                self.get_logger().info("Gate crossed, switching to REACH_GOAL")
                self.state = "REACH_GOAL"
                self.publish_state()

        if self.state == "REACH_GOAL":
            goal_x, goal_y = self.goal_position
            if abs(current_x - goal_x) < 0.5 and abs(current_y - goal_y) < 0.5:
                self.get_logger().info("Goal reached!")
                self.state = "IDLE"
                self.publish_state()

    def publish_state(self):
        # Publish the current state to the /state topic
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
