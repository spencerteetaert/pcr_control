import sys
sys.path.append('.')
import time

import yaml
import cv2

from src.data_generation.generate_movement import get_random_end_point
from src.controllers.cc_controller import CC_Model
from src.trajectory_planner import TrajectoryPlanner
from src.api.motor_api import MotorController
from src.api.aurora_api import AuroraAPI

if __name__=='__main__':
    # Initialize required objects
    config = yaml.safe_load(open("configs/config1.yaml", 'r'))
    model = CC_Model(config['controller_params'])

    controller = MotorController()
    aurora = AuroraAPI(verbose=False)

    controller.enable()
    aurora.start_tracking()

    # Update endpoint
    print("POSITION", aurora.get_position()[:2])
    print("End point updated:", model.update_end_point(aurora.get_position()[:2]))
    print("Controller position updated.", model.end_point)
    # input("1: Press enter to continue")

    time.sleep(3)

    # input("1.5: Press enter to continue")
    planner = TrajectoryPlanner(model, config['trajectory_params'])

    # input("1.6: Press enter to continue")

    dt = 0.001
    qs = planner.gen_trajectory(get_random_end_point(model), dt=dt, add_noise=False, verbose=False)

    print("Trajectory generated.")
    # input("2: Press enter to continue")

    # aurora.enable_log()
    # controller.enable_log()

    start_time = time.time()
    i = 0
    while time.time() - start_time < len(qs[0]) * dt:
        model.update_qs([qs[0][i], qs[2][i]])
        img = model.draw()
        cv2.imshow('Controller', img)

        i += 1
        time.sleep(max(0, i*dt - (time.time() - start_time)))

    # aurora.disable_log()
    # controller.disable_log()
        

    aurora.stop_tracking()
    controller.disable()

    