import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import math

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # TODO: 5.1.a Velocity Controller Setup
        ### STUDENT CODE HERE
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/cmd_wrench',
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered', # '/ground_truth/odometry' for dev
            self.odometry_callback,
            10
        )
        ### END STUDENT CODE

        # Mock Values, Tune These
        # self.Kp_surge = 1
        # self.Ki_surge = 0
        # self.Kd_surge = 0

        # self.Kp_yaw = 1
        # self.Ki_yaw = 0
        # self.Kd_yaw = 0
        
        # TODO: 5.1.g Controller Tuning
        ### STUDENT CODE HERE
        self.Kp_surge = 3
        self.Ki_surge = 4
        self.Kd_surge = 5
        self.Kp_yaw = 6
        self.Ki_yaw = 7
        self.Kd_yaw = 8
        ### END STUDENT CODE

        # Initialize variables - self added
        self.I_surge = 0
        self.I_yaw = 0

        # Low-pass derivative filter toggle
        self.use_derivative_filter = False  # SET True TO ENABLE FILTERING
        self.D_alpha = 0.8  # SMOOTHING FACTOR, 0 < ALPHA <= 1
        self.D_surge_prev = 0
        self.D_yaw_prev = 0

        self.cmd_vel = Twist()
        self.use_sim_time = self.get_clock().ros_time_is_active
        self.prev_time = None
        self.prev_error_surge = 0
        self.prev_error_yaw = 0

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def odometry_callback(self, msg: Odometry):
        if not self.prev_time:
            self.prev_time = Time.from_msg(msg.header.stamp) if self.use_sim_time else self.get_clock().now()
            return

        output_force = WrenchStamped()
        output_force.header = msg.header
        now = Time.from_msg(msg.header.stamp) if self.use_sim_time else self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        error_surge = 0
        # TODO: 5.1.b Error Calculation
        ### STUDENT CODE HERE
        # Extract quaternion
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        # Convert quaternion to yaw
        _, _, current_yaw = tf_transformations.euler_from_quaternion(quaternion)

        # Get world-frame velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Project onto boat's forward direction
        current_velocity = vx * math.cos(current_yaw) + vy * math.sin(current_yaw) # Dot product - velocity in boat's forward direction
        
        desired_velocity = self.cmd_vel.linear.x
        error_surge = desired_velocity - current_velocity
        ### END STUDENT CODE

        # P_surge = 0
        # TODO: 5.1.c Proportional Calculation
        ### STUDENT CODE HERE
        P_surge = self.Kp_surge * error_surge
        ### END STUDENT CODE
        
        # I_surge = 0
        # TODO: 5.1.d Integral Calculation
        ### STUDENT CODE HERE
        self.I_surge += self.Ki_surge * error_surge * dt
        # Anti-windup (clamp I_surge to reasonable bounds)
        self.I_surge = max(min(self.I_surge, 10.0), -10.0) # adjust bounds as needed
        ### END STUDENT CODE

        D_surge = 0
        # TODO: 5.1.e Derivative Calculation
        ### STUDENT CODE HERE
        raw_derivative_surge = 0.0 if dt == 0 else (error_surge - self.prev_error_surge) / dt
        if self.use_derivative_filter: # Low-pass filter on derivative term
            D_surge = self.Kd_surge * (self.D_alpha * raw_derivative_surge + (1 - self.D_alpha) * self.D_surge_prev)
            self.D_surge_prev = D_surge
        else:
            D_surge = self.Kd_surge * raw_derivative_surge
        self.prev_error_surge = error_surge
        ### END STUDENT CODE

        control_surge = P_surge + self.I_surge + D_surge
        output_force.wrench.force.x = control_surge

        # TODO: 5.1.f Yaw Control
        ### STUDENT CODE HERE
        desired_yaw = self.cmd_vel.angular.z
        error_yaw = desired_yaw - current_yaw
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        P_yaw = self.Kp_yaw * error_yaw

        self.I_yaw += self.Ki_yaw * error_yaw * dt
        # Anti-windup for yaw integral
        self.I_yaw = max(min(self.I_yaw, 10.0), -10.0) # adjust bounds as needed

        # Derivative calculation - Yaw
        D_yaw = 0
        raw_derivative_yaw = 0.0 if dt == 0 else (error_yaw - self.prev_error_yaw) / dt
        if self.use_derivative_filter: # Low-pass filter on derivative term
            D_yaw = self.Kd_yaw * (self.D_alpha * raw_derivative_yaw + (1 - self.D_alpha) * self.D_yaw_prev)
            self.D_yaw_prev = D_yaw
        else:
            D_yaw = self.Kd_yaw * raw_derivative_yaw
        self.prev_error_yaw = error_yaw

        control_yaw = P_yaw + self.I_yaw + D_yaw
        output_force.wrench.torque.z = control_yaw
        ### END STUDENT CODE

        self.wrench_pub.publish(output_force)

        # Ignore this line, only for autograder purposes
        return error_surge, P_surge, self.I_surge, D_surge, control_yaw, output_force


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt: # Handle Ctrl-C
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
