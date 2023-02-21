import sys
sys.path.append('.')
import time

import yaml
import cv2

from src.controllers.closed_loop_controller import Closed_Loop_Controller
from src.api.motor_api import MotorController
from src.api.aurora_api import AuroraAPI

if __name__=='__main__':
    # Initialize required objects
    config = yaml.safe_load(open("configs/config1.yaml", 'r'))
    model = Closed_Loop_Controller(config['controller_params'])

    controller = MotorController(type='vel')
    aurora = AuroraAPI(verbose=False)

    controller.enable()
    aurora.start_tracking()

    # Update endpoint
    print("POSITION", aurora.get_position()[:2])
    print("End point updated:", model.update_end_point(aurora.get_position()[:2]))
    print("Controller position updated.", model.end_point)
    input("1: Press enter to continue")

    # aurora.enable_log()
    # controller.enable_log()

    start_time = time.time()
    i = 0
    while True:
        model.update_end_point(aurora.get_position()[:2])

        u = model.get_command()

        controller.set_ref(u)

        img = model.draw()
        cv2.imshow('Controller', img)

        i += 1
        time.sleep(max(0, i*dt - (time.time() - start_time)))

    # aurora.disable_log()
    # controller.disable_log()
        

    aurora.stop_tracking()
    controller.disable()

    