from stinger_controller.control_models.control_base_class import ControlBaseClass

class SimpleInverseNewtonianModel(ControlBaseClass):
    def __init__(self, mass: float):
        self.mass = mass
    
    def __call__(self, input: dict) -> float:
        # a = F/m
        return input['control'] / self.mass