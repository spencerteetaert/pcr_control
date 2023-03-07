import time 
import sys
import threading
sys.path.append('.')

from pynput import keyboard
import matplotlib.pyplot as plt

from src.api.motor_api import MotorController
from blmc.pid import PID

CONTROL_TYPE = 'vel'

api = MotorController(type = CONTROL_TYPE)
time.sleep(2)
api.enable()
time.sleep(1)
print("API enabled.")
ref = [0, 0]

if CONTROL_TYPE == 'pos':
    step = 0.1
else:
    step = 0.005

i1, i2 = 0, 1
FLAG = time.time() 

def on_press(key):
    global ref, api, i1, i2, FLAG

    if key == keyboard.KeyCode.from_char('a'):
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

    api.set_ref(ref)

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

if CONTROL_TYPE == 'pos':
    ref[0] = api.mtr_data.mtr1.position.value
    ref[1] = api.mtr_data.mtr2.position.value

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

# api.enable_log('data/DELETE')
print("Manual control ready")
listener.join()
# api.disable_log()

api.disable()
