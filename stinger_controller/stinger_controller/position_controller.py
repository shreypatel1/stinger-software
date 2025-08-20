import rclpy
from rclpy.node import Node
from stinger_msgs.msg import Goal
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from stinger_controller.control_models.line_follower_mpc import LineFollowerMPC


class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')

        self.create_subscription(
            Goal,
            '/goal',
            self.goal_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.create_timer(0.1, self.update_callback)

        self.current_goal: Goal = Goal()
        self.current_odom = Odometry()

        self.control = LineFollowerMPC()
    
    def goal_callback(self, msg: Goal) -> None:
        self.current_goal = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg

    def update_callback(self) -> None:
        msg: TwistStamped = TwistStamped()
        if self.current_goal.mode == Goal.MODE_WAYPOINT:
            inputs = {
                'parametric_line': self.current_goal.line,
                'goal_pose': self.current_goal.goal_pose,
                'current_pose': self.current_odom.pose.pose
            }
            msg = self.control(inputs, self)
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)
