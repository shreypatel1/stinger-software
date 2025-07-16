from stinger_controller.control_models.control_base_class import ControlBaseClass

class ForceDisturbanceObserver(ControlBaseClass):
    def __init__(self,
                 acceleration_to_force_model: ControlBaseClass,
                 force_to_acceleration_model: ControlBaseClass,
                 gamma: float = 0.05,
                 measured_control_alpha: float = 0.95,
                 disturbance_alpha: float = 0.95) -> None:
        self.gamma = gamma
        self.acceleration_to_force_model = acceleration_to_force_model
        self.estimated_disturbance = 0.0
        self.force_to_acceleration_model = force_to_acceleration_model
        self.measured_acceleration_filtered = 0.0
        self.disturbance_force_filtered = 0.0
        self.measured_acceleration_alpha = measured_control_alpha
        self.disturbance_force_alpha = disturbance_alpha

    def __call__(self, input: dict) -> float:
        # This represents our expected net force/ actual acceleration we are achieving
        expected_control = input['previous_control_command'] - self.estimated_disturbance
        # Transform the expected control (with units of ouput control) into units of input control
        # also referred to as measured/tracking control
        expected_control_transformed = self.force_to_acceleration_model({'control': expected_control})
        self.measured_acceleration_filtered = self.low_pass_filter(
            self.measured_acceleration_filtered,
            input['actual_control'],
            self.measured_acceleration_alpha 
        )
        error = expected_control_transformed - self.measured_acceleration_filtered
        # Update disturbance estimate
        self.estimated_disturbance += self.gamma * self.acceleration_to_force_model({'desired_control': error})

        # Optionally filter the estimated disturbance too
        self.disturbance_force_filtered = self.low_pass_filter(
            self.disturbance_force_filtered,
            self.estimated_disturbance,
            self.disturbance_force_alpha
        )

        # Output control with filtered disturbance (or raw, if preferred)
        control_output = self.acceleration_to_force_model(input) + self.disturbance_force_filtered
        return control_output

    def low_pass_filter(self, filtered: float, raw: float, alpha: float) -> float:
        return alpha * filtered + (1 - alpha) * raw