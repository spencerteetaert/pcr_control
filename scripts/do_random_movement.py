import sys
sys.path.append('.')

import yaml

from src.data_generation.generate_movement import do_movement
from src.controllers.cc_controller import CC_Model
from src.trajectory_planner import TrajectoryPlanner
from src.api.motor_api import MotorController
from src.api.aurora_api import AuroraAPI

if __name__=='__main__':
    config = yaml.safe_load(open("configs/config1.yaml", 'r'))
    controller = CC_Model(config['controller_params'])

    controller.update_end_point([0.25, 0.45])

    planner = TrajectoryPlanner(controller, config['trajectory_params'])

    planner.gen_trajectory([0.10, 0.30], add_noise=False)

    # controller = MotorController()
    # aurora = AuroraAPI(verbose=True)

    # controller.enable()
    # aurora.start_tracking()  

    # do_movement('data/DEL/', None, aurora, controller)

    # aurora.stop_tracking()
    # controller.disable()

    