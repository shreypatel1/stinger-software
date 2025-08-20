import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np

class ThrottleController(Node):
    def __init__(self):
        super().__init__('throttle_controller')

        # Physical parameters
        self.thruster_distance = 0.153  # Distance from center to each thruster

        # ROS2 Subscriptions & Publishers
        self.create_subscription(
            WrenchStamped,
            '/cmd_wrench',
            self.wrench_callback,
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

    def wrench_callback(self, msg: WrenchStamped):
        fx = msg.wrench.force.x          # Desired force in X (surge)
        torque_z = msg.wrench.torque.z   # Desired yaw torque

        d = self.thruster_distance

        # Allocation matrix:
        # Each thruster contributes to surge and yaw:
        # port  = [1, -d]
        # stbd  = [1, +d]
        # So we invert that:
        # [Fx]       [1  1] [u_port]
        # [Mz]   =   [-d d] [u_stbd]
        #
        # Solve: u = J_inv @ wrench
        J = np.array([
            [1.0,  1.0],
            [-d,   d]
        ])

        J_inv = np.linalg.inv(J)
        wrench_vec = np.array([fx, torque_z])
        motor_forces = J_inv @ wrench_vec

        u_port = motor_forces[0]
        u_stbd = motor_forces[1]

        # Optional: saturate to maximum thruster output
        max_thrust = 30.0
        u_port = np.clip(u_port, -max_thrust, max_thrust)
        u_stbd = np.clip(u_stbd, -max_thrust, max_thrust)

        # Publish the thrust commands
        self.port_thruster_pub.publish(Float64(data=u_port))
        self.stbd_thruster_pub.publish(Float64(data=u_stbd))

def main(args=None):
    rclpy.init(args=args)
    node = ThrottleController()
    rclpy.spin(node)
