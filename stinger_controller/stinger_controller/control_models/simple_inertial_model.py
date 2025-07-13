from stinger_controller.control_models.control_base_class import ControlBaseClass

class SimpleInertialModel(ControlBaseClass):
    def __init__(self, moment: float):
        self.moment = moment
    
    def __call__(self, input: dict) -> float:
        # T = Ia
        return input['alpha'] * self.moment