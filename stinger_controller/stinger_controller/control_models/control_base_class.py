from abc import ABC, abstractmethod

class ControlBaseClass(ABC):
    @abstractmethod
    def __call__(self, input: dict) -> float:
        pass

class CompositeControlFunction(ControlBaseClass):
    def __init__(self, components: list[ControlBaseClass]):
        self.components = components

    def __call__(self, input_data: dict) -> any:
        total = 0.0
        for component in self.components:
            total += component(input_data)
        return total