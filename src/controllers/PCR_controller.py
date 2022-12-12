import numpy as np

class PCRController:
    def __init__(self):
        self.costmap = None
        self.scale = 0
        self.end_point = np.array([0.0])
        self.links = []

    def update_end_point(self, endpoint):
        raise NotImplementedError()

    def enable_log(self, filename):
        raise NotImplementedError()

    def disable_log(self):
        raise NotImplementedError()

    def draw(self):
        raise NotImplementedError()