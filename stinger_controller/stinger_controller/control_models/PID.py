from stinger_controller.control_models.control_base_class import ControlBaseClass
import time

class PID(ControlBaseClass):
    def __init__(self,
                 kp: float, ki: float, kd: float,
                 windup_max: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_err = 0.0
        self.integral = 0.0
        self.previous_time = None
        # max magnitude to cap the integral
        self.windup_max = windup_max

    def __call__(self, input: dict) -> float:
        # First loop
        if self.previous_time is None:
            self.previous_time = time.time()
            return 0.0
        
        dt = time.time() - self.previous_time
        self.previous_time = time.time()
        P = 0
        I = 0
        D = 0
        error = input['desired_control'] - input['actual_control']

        ### STUDENT CODE HERE

        ### END STUDENT CODE

        ### STUDENT CODE HERE

        ### END STUDENT CODE

        # Integral Term
        self.integral += error * dt
        I = self.ki * self.integral

        # Clamp the integral term so it doesn't grow too large
        if self.windup_max is not None:
            self.integral = max(-self.windup_max, min(self.integral, self.windup_max))

        # compute the output feedback
        control = P + I + D

        self.prev_err = error

        return control