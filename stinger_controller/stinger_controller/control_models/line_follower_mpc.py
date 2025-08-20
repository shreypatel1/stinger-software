from stinger_controller.control_models.control_base_class import ControlBaseClass
from stinger_msgs.msg import ParametricLine

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
from scipy.optimize import minimize
import numpy as np
import tf_transformations

class LineFollowerMPC(ControlBaseClass):
    def __init__(self, constraints=None, horizon=10, dt=0.5, dist_weight=0.35, control_weight=0.15, traj_weight=0.2, goal_weight=0.3):
        self.horizon = horizon
        self.dt = dt
        self.constraints = constraints
        self.dist_weight = dist_weight
        self.control_weight = control_weight
        self.traj_weight = traj_weight
        self.goal_weight = goal_weight
        self.max_distance = 0.0

    def distance_function(self, x_current, parametric_line: ParametricLine) -> float:
        p0 = np.array([[parametric_line.b_x, parametric_line.b_y]])
        v = np.array([parametric_line.m_x, parametric_line.m_y])
        q = x_current[:2]
        
        w = q - p0
        t = np.dot(w, v)
        closest_point = p0 + t * v
        distance = np.linalg.norm(q - closest_point)

        distance = np.clip(distance / 2.0, 0.0, 1.0)
        return distance

    def control_cost_function(self, current_u) -> float:
        v = np.clip(current_u[0] / 0.3, -1.0, 1.0)
        w = np.clip(current_u[1] / 0.2, -1.0, 1.0)
        return v ** 2 + w ** 2

    def traj_cost_function(self, x, ref_traj):
        theta = x[2]
        robot_heading = np.array([np.cos(theta), np.sin(theta)])
        dot = np.clip(np.dot(robot_heading, ref_traj), -1.0, 1.0)
        angle_error = np.arccos(dot)

        heading_cost = angle_error**2
        heading_cost = np.clip(heading_cost, 0.0, 1.0)
        return heading_cost

    def goal_cost_function(self, x_current, goal_pose: Pose) -> float:
        x_err = (x_current[0] - goal_pose.position.x) ** 2
        y_err = (x_current[1] - goal_pose.position.y) ** 2
        distance = x_err + y_err
        return distance / self.max_distance

    def propagate_state(self, x, u, dt):
        theta = x[2]
        v, w = u
        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dtheta = w
        x_next = x + np.array([dx, dy, dtheta]) * dt
        return x_next

    def cost_function(self, u_flat, x0, parametric_line: ParametricLine, goal_pose: Pose) -> float:
        cost = 0
        x_current = x0
        v = np.array([parametric_line.m_x, parametric_line.m_y])
        for i in range(self.horizon):
            current_u = [u_flat[i * 2], u_flat[i * 2 + 1]]
            dist_cost = self.dist_weight * self.distance_function(x_current, parametric_line)
            control_cost = self.control_weight * self.control_cost_function(current_u)
            traj_cost = self.traj_weight * self.traj_cost_function(x_current, v)
            # Puts more emphasis on reaching the goal in the future
            # goal_weight_i = self.goal_weight * (i + 1) / self.horizon
            # goal_cost = goal_weight_i * self.goal_cost_function(x_current, goal_pose)
            goal_cost = self.goal_weight * self.goal_cost_function(x_current, goal_pose)
            cost = cost + dist_cost + control_cost + traj_cost + goal_cost

            x_current = self.propagate_state(x_current, current_u, self.dt)

        return cost

    def __call__(self, input: dict, super) -> TwistStamped:
        parametric_line: ParametricLine = input['parametric_line']
        goal_pose: Pose = input['goal_pose']
        pose: Pose = input['current_pose']
        
        self.max_distance = ((goal_pose.position.x - pose.position.x) ** 2 + (goal_pose.position.y - pose.position.y) ** 2) + 5.0
        # Avoid division by 0 issues
        self.max_distance = max(self.max_distance, 1e-9)
        # Normalize the vector
        norm = np.linalg.norm([parametric_line.m_x, parametric_line.m_y])
        parametric_line.m_x = parametric_line.m_x / norm
        parametric_line.m_y = parametric_line.m_y / norm
        quat = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                ]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        # Initial State in global frame
        x0 = np.array([
            pose.position.x,
            pose.position.y,
            yaw
        ])
        
        # Initial guess
        # u = [v, w]
        u0 = np.array([0.3, 0.0] * self.horizon)
        # bound input to +- 0.3 m/s and +- 0.2 radians/s
        bounds = [(-0.3, 0.3), (-0.2, 0.2)] * self.horizon
        res = minimize(
            self.cost_function,
            u0.flatten(),
            args=(x0, parametric_line, goal_pose),
            bounds=bounds
        )
        u_opt = res.x.reshape((self.horizon, 2))
        v, w = u_opt[0]  # take first control input

        msg: TwistStamped = TwistStamped()
        msg.header.stamp = super.get_clock().now().to_msg()
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        return msg
