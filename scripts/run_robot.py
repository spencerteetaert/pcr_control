import sys
sys.path.append('.')
import yaml 
import time

from pynput import keyboard

from src.controllers.PCR_controller import PCRController, State
from src.api.aurora_api import AuroraAPI
from src.api.motor_api import MotorController
from src.controllers.closed_loop_controller import Closed_Loop_Controller

control_space = 'joint'
control_type = 'vel' # vel for task space control, either for joint space

ref = [0, 0]

if control_space == 'task':
    step = 0.05 # m 
elif control_space == 'joint':
    if control_type == 'pos':
        step = 0.1 # rad 
    elif control_type == 'vel':
        step = 0.005 # rad / s 

config = yaml.safe_load(open("configs/config1.yaml", 'r'))
controller = PCRController(MotorController(type=control_type), AuroraAPI(verbose=False), Closed_Loop_Controller(config['controller_params'], real_time=True), debug=True)

def on_press(key):
    global ref, controller, control_space

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
        controller._start_recovery()
        return 
    elif key == keyboard.KeyCode.from_char('r'):
        # Enter random data generation mode 
        controller.set_mode('ran')
        return 
    elif key == keyboard.KeyCode.from_char('m'):
        # Toggles between joint and task space control 
        control_space = 'joint' if control_space == 'task' else 'task'
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
