from .aurora_api import AuroraAPI
from .motor_api import MotorController

class PCR_API:
    def __init__(self):
        self.controller = MotorController()
        self.aurora = AuroraAPI()

    def enable(self):
        self.controller.enable()
        self.aurora.start_tracking()

    def disable(self):
        self.controller.disable()
        self.aurora.stop_tracking()

    def set_velocity(self, velocities):
        self.controller.set_velocity(velocities)
    
    def get_data(self):
        aurora_data = self.aurora.get_sensor_data()
        motor_data = self.controller.get_sensor_data()

        return aurora_data, motor_data


if __name__=='__main__':
    import time 

    api = PCR_API()
    api.enable()

    api.set_velocity([1, 1, 0, 0])

    for i in range(10):
        print(api.get_data())
        time.sleep(1)

    api.disable()