from rclpy.time import Duration

class PID:
    def __init__(self,
                 kp: float, ki: float, kd: float,
                 windup_max: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_err = 0.0
        self.integral = 0.0

        # max magnitude to cap the integral
        self.windup_max = windup_max

    def __call__(self, err: float, dt: Duration) -> float:
        dt = dt.nanoseconds * 1e-9

        # Check for valid dt
        if dt <= 0:
            print("invalid dt")
            return 0.0
        
        P = 0
        I = 0
        D = 0

        ### STUDENT CODE HERE

        ### END STUDENT CODE

        ### STUDENT CODE HERE

        ### END STUDENT CODE

        ### STUDENT CODE HERE

        ### END STUDENT CODE

        # Clamp the integral term so it doesn't grow too large
        if self.windup_max is not None:
            self.integral = max(-self.windup_max, min(self.integral, self.windup_max))

        # compute the output feedback
        fb = P + I + D

        u = fb

        self.prev_err = err

        return u