import sys
sys.path.append('.')
import time

import yaml

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
    aurora = AuroraAPI(verbose=True)

    controller.enable()
    aurora.start_tracking()

    # Update endpoint
    print("POSITION", aurora.get_position()[:2])
    print("End point updated:", model.update_end_point(aurora.get_position()[:2]))
    print("Controller position updated.", model.end_point)
    input("Press enter to continue")
    planner = TrajectoryPlanner(model, config['trajectory_params'])

    dt = 0.001
    qs = planner.gen_trajectory(get_random_end_point(model), dt=dt, add_noise=True, verbose=True)

    print("Trajectory generated.")
    input("Press enter to continue")

    # aurora.enable_log()
    # controller.enable_log()

    start_time = time.time()
    i = 0
    while time.time() - start_time < len(qs[0]) * dt:
        print(qs[0][i], i*dt - (time.time() - start_time))

        i += 1
        time.sleep(max(0, i*dt - (time.time() - start_time)))

    # aurora.disable_log()
    # controller.disable_log()
        

    aurora.stop_tracking()
    controller.disable()

    