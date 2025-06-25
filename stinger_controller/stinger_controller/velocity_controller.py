import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from stinger_controller.PID import PID
from nav_msgs.msg import Odometry

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.cmd_accel_pub = self.create_publisher(
            Twist,
            '/cmd_accel',
            10
        )

        self.odom_sub = self.create_subscription(
          Odometry,
          '/odometry/filtered',
          self.odometry_callback,
          10
        )

        self.pid_linear = PID(20, 0, 0)
        self.pid_angular = PID(10, 0, 0)

        self.prev_time = self.get_clock().now()
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def odometry_callback(self, msg: Odometry):
        twist_msg: Twist = Twist()
        linear_err = 0.0
        angular_err = 0.0

        ### STUDENT CODE HERE

        ### END STUDENT CODE
        
        twist_msg.linear.x = self.pid_linear(linear_err, dt)
        twist_msg.angular.z = self.pid_angular(angular_err, dt)
        self.cmd_accel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    
