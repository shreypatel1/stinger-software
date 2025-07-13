import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, Wrench, Twist
from sensor_msgs.msg import Imu
from stinger_controller.control_models.control_base_class import CompositeControlFunction
from stinger_controller.control_models.PID import PID
from stinger_controller.control_models.simple_newtonian_model import SimpleNewtonianModel
from stinger_controller.control_models.simple_inertial_model import SimpleInertialModel
from stinger_controller.control_models.simple_linear_drag_model import SimpleLinearDragModel

class AccelerationController(Node):
    def __init__(self):
        super().__init__('acceleration_controller')

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Accel,
            '/cmd_accel',
            self.cmd_accel_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.cmd_wrench_pub = self.create_publisher(
            Wrench,
            '/cmd_wrench',
            10
        )
        cross_section_area = 2 * 0.081 * 0.135
        self.linear = CompositeControlFunction([
            # Feedback
            PID(kp=2.0, ki=0.0, kd=0.0),
            # Feedforward
            SimpleNewtonianModel(mass=5.0),
            SimpleLinearDragModel(C=0.82, area=cross_section_area)
        ])

        self.angular = CompositeControlFunction([
            # Feedback
            PID(kp=1.0, ki=0.0, kd=0.0),
            # Feeforward
            SimpleInertialModel(moment=3.6403125)
        ])
        
        self.prev_time = None
        self.prev_angular_velocity = None
        self.cmd_acceleration_linear = 0.0
        self.cmd_acceleration_angular = 0.0
        self.cmd_vel_linear = 0.0
    
    def cmd_accel_callback(self, msg: Accel) -> None:
        self.cmd_acceleration_linear = msg.linear.x
        self.cmd_acceleration_angular = msg.angular.z
    
    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_linear = msg.linear.x

    def imu_callback(self, msg: Imu) -> None:
        if self.prev_time is None or self.prev_angular_velocity is None:
            self.prev_time = self.get_clock().now()
            self.prev_angular_velocity = msg.angular_velocity.z
            return
        wrench_msg: Wrench = Wrench()
        a_linear = msg.linear_acceleration.x
        linear_error = self.cmd_acceleration_linear - a_linear
        linear_input = {
            'error': linear_error,
            'acceleration': self.cmd_acceleration_linear,
            'velocity': self.cmd_vel_linear
        }

        dt = self.get_clock().now() - self.prev_time
        a_angular = (msg.angular_velocity.z - self.prev_angular_velocity) / (dt.nanoseconds * 1e-9)
        angular_error = self.cmd_acceleration_angular - a_angular
        angular_input = {
            'error': angular_error,
            'alpha': self.cmd_acceleration_angular
        }

        wrench_msg.force.x = self.linear(linear_input)
        wrench_msg.torque.z = self.angular(angular_input)
        self.cmd_wrench_pub.publish(wrench_msg)


def main(args=None):
    rclpy.init()
    node = AccelerationController()
    rclpy.spin(node)
