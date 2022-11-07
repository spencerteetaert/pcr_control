import time 
import sys
sys.path.append('.')

print(sys.path)

from src.api.single_motor_api import MotorController

velocity = 1
dt = 3

api = MotorController()
api.enable()

print("Starting trajectory")
api.set_velocity([velocity, -velocity, 0, 0])
time.sleep(dt)

api.set_velocity([0, 0, 0, 0])
time.sleep(1)

api.set_velocity([-velocity, velocity, 0, 0])
time.sleep(dt)

api.set_velocity([0, 0, 0, 0])
print("Finished Trajectory")
api.disable()