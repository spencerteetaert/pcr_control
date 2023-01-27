import time 
import sys
import threading
sys.path.append('.')

from pynput import keyboard

from src.api.motor_api import MotorController
from blmc.pid import PID

velocity = 1
dt = 3

api = MotorController(False)
time.sleep(2)
api.enable()
print("API enabled.")
speed = [0, 0, 0, 0]

i1, i2 = 0,1
FLAG = time.time() 

def on_press(key):
    global speed, api, i1, i2, FLAG

    override = False
    if key == keyboard.KeyCode.from_char('t'):
        api.auto_tension = not api.auto_tension
        print("Auto tension:", api.auto_tension)
    elif key == keyboard.KeyCode.from_char('a'):
        override = True
        speed[i1] += 0.005
    elif key == keyboard.KeyCode.from_char('q'):
        override = True
        speed[i1] -= 0.005
    elif key == keyboard.KeyCode.from_char('w'):
        override = True
        speed[i2] -= 0.005
    elif key == keyboard.KeyCode.from_char('s'):
        override = True
        speed[i2] += 0.005
        print("TIME:", time.time()- FLAG)

        FLAG = time.time() 

    # elif key == keyboard.Key.up:
    #     i1 += 2
    #     i2 += 2
    #     i1 = i1 % 4
    #     i2 = i2 % 4
    # elif key == keyboard.Key.left:
    #     speed[i1] += 0.005
    #     speed[i2] += 0.005
    # elif key == keyboard.Key.right:
    #     speed[i2] -= 0.005
    #     speed[i1] -= 0.005
    else:
        override = True
        if abs(speed[0]) > 0.03:
            speed[0] += 0.03 * ((speed[0] < 0) - 0.5)*2
        else:
            speed[0] = 0
        if abs(speed[1]) > 0.03:
            speed[1] += 0.03 * ((speed[1] < 0) - 0.5)*2
        else:
            speed[1] = 0
        if abs(speed[2]) > 0.03:
            speed[2] += 0.03 * ((speed[2] < 0) - 0.5)*2
        else:
            speed[2] = 0
        if abs(speed[3]) > 0.03:
            speed[3] += 0.03 * ((speed[3] < 0) - 0.5)*2
        else:
            speed[3] = 0

    print(speed, i1, i2)

    if override:
        api._set_velocity(speed)
    else:
        api.set_velocity([speed[0], speed[2]])

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

# api.enable_log('data/DELETE')

print("Manual control ready")
listener.join()
# t.kill = True
# t.thread.join()

# api.disable_log()
api.disable()
