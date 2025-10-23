from abc import ABC, abstractmethod

class Section(ABC):
    def __init__(self, name, window):
        self.name = name
        self.window = window
    
    @abstractmethod
    def draw(self):
        pass