import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

class ThrottleController(Node):
    def __init__(self):
        super().__init__('throttle_controller')
        self.mass = 5.0
        self.inertia_z = 3.6403125
        self.thruster_distance = 0.153

        self.create_subscription(
            Twist,
            '/cmd_accel',
            self.accel_callback,
            10
        )

        self.port_thruster_pub = self.create_publisher(
            Float64,
            '/stinger/thruster_port/cmd_thrust',
            10
        )

        self.stbd_thruster_pub = self.create_publisher(
            Float64,
            '/stinger/thruster_stbd/cmd_thrust',
            10
        )
    
    def accel_callback(self, msg: Twist):
        ax = msg.linear.x
        alpha_z = msg.angular.z

        m = self.mass
        Iz = self.inertia_z
        d = self.thruster_distance

        # Jacobian (J)
        J = np.array([
            [1/m,   1/m],
            [-d/Iz, d/Iz]
        ])

        J_inv = np.linalg.inv(J)
        acc = np.array([ax, alpha_z])
        motor_forces = J_inv @ acc

        max_thrust = 5.0
        u_port = np.sign(motor_forces[0]) * min(abs(motor_forces[0]), max_thrust)
        u_stbd = np.sign(motor_forces[1]) * min(abs(motor_forces[1]), max_thrust)

        self.port_thruster_pub.publish(Float64(data=u_port))
        self.stbd_thruster_pub.publish(Float64(data=u_stbd))


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleController()
    rclpy.spin(node)
        
