import sys
sys.path.append('.')
import yaml 
import time

from pynput import keyboard

from src.controllers.PCR_controller import PCRController, State
from src.api.aurora_api import AuroraAPI
from src.api.motor_api import MotorController
from src.controllers.closed_loop_controller import Closed_Loop_Controller


config = yaml.safe_load(open("configs/config1.yaml", 'r'))
controller = PCRController(MotorController(type='pos', auto_tension = False), AuroraAPI(verbose=False), Closed_Loop_Controller(config['controller_params'], real_time=True), debug=True)

controller.enable()

ref = [0, 0]
step = 0.5

i1, i2 = 0, 1

def on_press(key):
    global ref, controller

    if controller.state == State.READY:
        # If ready, reset ref position. This is for post recovery 
        ref[0] = controller.motor_api.mtr_data.mtr1.position.value
        ref[1] = controller.motor_api.mtr_data.mtr2.position.value
    if controller.state == State.RECOVER:
        return 

    if key == keyboard.KeyCode.from_char('e'):
        # Force recovery bahavior 
        controller._start_recovery()
        return 

    if key == keyboard.KeyCode('r'):
        # Enter random data generation mode 
        controller.set_mode('ran')
        return 

    if key == keyboard.KeyCode.from_char('t'):
        controller.controller.auto_tension = not controller.controller.auto_tension
        print("Auto tension:", controller.controller.auto_tension)
    elif key == keyboard.KeyCode.from_char('a'):
        ref[i1] -= step
    elif key == keyboard.KeyCode.from_char('q'):
        ref[i1] += step
    elif key == keyboard.KeyCode.from_char('w'):
        ref[i2] += step
    elif key == keyboard.KeyCode.from_char('s'):
        ref[i2] -= step

    else:
        if abs(ref[0]) > 0.03:
            ref[0] += 0.03 * ((ref[0] < 0) - 0.5)*2
        else:
            ref[0] = 0
        if abs(ref[1]) > 0.03:
            ref[1] += 0.03 * ((ref[1] < 0) - 0.5)*2
        else:
            ref[1] = 0

    controller.set_mode('man', ref)

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

ref[0] = controller.motor_api.mtr_data.mtr1.position.value
ref[1] = controller.motor_api.mtr_data.mtr2.position.value

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

print("Manual control ready")
listener.join()

controller.disable()
