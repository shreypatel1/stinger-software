import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, Twist
from stinger_controller.control_models.PID import PID
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
            Accel,
            '/cmd_accel',
            10
        )

        self.odom_sub = self.create_subscription(
          Odometry,
          '/odometry/filtered',
          self.odometry_callback,
          10
        )

        self.pid_linear = PID(kp=1, ki=0, kd=0)
        self.pid_angular = PID(kp=1, ki=0, kd=0)

        self.prev_time = self.get_clock().now()
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def odometry_callback(self, msg: Odometry):
        accel_msg: Accel = Accel()
        
        v_linear = msg.twist.twist.linear.x
        v_angular = msg.twist.twist.angular.z

        linear_err = self.cmd_linear - v_linear
        linear_input = {
            'error': linear_err
        }
        angular_err = self.cmd_angular - v_angular
        angular_input = {
            'error': angular_err
        }
        accel_msg.linear.x = self.pid_linear(linear_input)
        accel_msg.angular.z = self.pid_angular(angular_input)
        self.cmd_accel_pub.publish(accel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    
