import sys
sys.path.append('.')
import yaml 
import time
from enum import Enum

import numpy as np
from pynput import keyboard

from src.controllers.PCR_controller import PCRController, State
from src.api.aurora_api import AuroraAPI
from src.api.motor_api import MotorController
from src.controllers.closed_loop_controller import Closed_Loop_Controller
from src.controllers.cc_diff_controller import CC_Diff_Controller
from src.controllers.learned_controller import Learned_Controller
from src.controllers.cc_controller import CC_Controller

class ControllerTypes(Enum):
    CC_DIFF = 0 
    PID = 1 
    LEARNED = 2 
    CC = 3

control_space = 'task'
control_type = 'vel' # vel for task space control, either for joint space
controller_type = ControllerTypes.CC

ref = [0, 0]
ref_trajs = [
    "data/test_trajs/square.npy",
    "data/test_trajs/circle.npy",
    "data/test_trajs/zigzag.npy",
    "data/test_trajs/hline.npy",
    "data/test_trajs/vline.npy",
]




if control_space == 'task':
    step = 0.05 # m 
elif control_space == 'joint':
    if control_type == 'pos':
        step = 0.1 # rad 
    elif control_type == 'vel':
        step = 0.005 # rad / s 

config = yaml.safe_load(open("configs/config1.yaml", 'r'))
# log_dir = f'logs/{time.time()}'
log_dir = f'logs/video2_CC'

if controller_type == ControllerTypes.PID:
    controller = PCRController(MotorController(type=control_type), AuroraAPI(verbose=False), Closed_Loop_Controller(config['controller_params'], real_time=True), debug=True, log_dir=log_dir)
elif controller_type == ControllerTypes.CC_DIFF:
    controller = PCRController(MotorController(type=control_type), AuroraAPI(verbose=False), CC_Diff_Controller(config['controller_params'], real_time=True), debug=True, log_dir=log_dir)
elif controller_type == ControllerTypes.LEARNED:
    model_path = "/home/jimmy/spencer_thesis/pcr_control/experiments/feedback_horizon_search2/horizon_30/models/best_val"
    learned_controller = Learned_Controller(model_path, real_time=True)
    controller = PCRController(MotorController(type=control_type, learning_model=learned_controller), AuroraAPI(verbose=False), learned_controller, debug=True, log_dir=log_dir)
elif controller_type == ControllerTypes.CC:
    controller = PCRController(MotorController(type=control_type), AuroraAPI(verbose=False), CC_Controller(config, real_time=True), debug=True, log_dir=log_dir)
else:
    raise ValueError("Please select a valid controller type.")

temp_flag = False

def on_press(key):
    global ref, controller, control_space, temp_flag

    # State responses 
    if controller.state == State.RECOVER:
        return 
    if controller.state == State.READY:
        # If ready, reset ref position. This is for post recovery 
        if control_space == 'joint':
            if control_type == 'pos':
                ref[0] = controller.motor_api.mtr_data.mtr1.position.value
                ref[1] = controller.motor_api.mtr_data.mtr2.position.value
            elif control_type == 'vel':
                ref = [0, 0]
        elif control_space == 'task':
            ref = controller.end_point
    
    # Toggles 
    if key == keyboard.KeyCode.from_char('e'):
        # Force recovery bahavior 
        controller.set_mode('man_joint' if control_space == 'joint' else 'man_task', ref)
        controller._start_recovery()
        temp_flag = True
        return 
    elif key == keyboard.KeyCode.from_char('r'):
        # Enter random data generation mode 
        controller.set_mode('ran')
        return 
    elif key == keyboard.KeyCode.from_char('m'):
        # Toggles between joint and task space control 
        control_space = 'joint' if control_space == 'task' else 'task'
        return 
    elif key == keyboard.KeyCode.from_char('t'):
        if controller.controller.__class__.__name__ == "CC_Diff_Controller" and temp_flag == False:
            # TODO: Fix. This shouldn't be required. 
            controller.set_mode('man_joint' if control_space == 'joint' else 'man_task', ref)
            controller._start_recovery()
            temp_flag = True
            return 
        # Runs test trajectories 
        temp_flag = False
        ref_traj = np.load(ref_trajs.pop(0))
        controller.set_mode('ref', ref_traj)
        return 
    elif key == keyboard.KeyCode.from_char('p'):
        controller.set_mode('trial', [0.15, 0.2])
        return 

    # Movement 
    elif key == keyboard.KeyCode.from_char('a'):
        ref[0] -= step
    elif key == keyboard.KeyCode.from_char('d' if control_space == 'task' else 'q'):
        ref[0] += step
    elif key == keyboard.KeyCode.from_char('w'):
        ref[1] += step
    elif key == keyboard.KeyCode.from_char('s'):
        ref[1] -= step
    else:
        if abs(ref[0]) > 0.03:
            ref[0] += 0.03 * ((ref[0] < 0) - 0.5)*2
        else:
            ref[0] = 0
        if abs(ref[1]) > 0.03:
            ref[1] += 0.03 * ((ref[1] < 0) - 0.5)*2
        else:
            ref[1] = 0
    controller.set_mode('man_joint' if control_space == 'joint' else 'man_task', ref)

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

if __name__=='__main__':
    controller.enable()

    if control_space == 'joint':
        if control_type == 'pos':
            ref[0] = controller.motor_api.mtr_data.mtr1.position.value
            ref[1] = controller.motor_api.mtr_data.mtr2.position.value
    elif control_space == 'task':
        ref = controller.end_point

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    print("Manual control ready")
    listener.join()

    controller.disable()
