from stinger_controller.control_models.control_base_class import ControlBaseClass
import numpy as np

WATER_DENSITY = 1000.0

class SimpleLinearDragModel(ControlBaseClass):
    def __init__(self, C: float, area: float):
        # Drag coefficient
        self.C = C
        self.area = area
    
    def __call__(self, input: dict) -> float:
        desired_velocity = input['velocity']
        # Linear Drag: F = 1/2 CpAv^2            
        f_drag_abs = 1/2 * self.C * WATER_DENSITY * self.area * desired_velocity ** 2  # noqa: E501
        f_drag = np.sign(desired_velocity) * f_drag_abs
        return f_drag