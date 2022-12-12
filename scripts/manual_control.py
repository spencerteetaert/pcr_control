import time 
import sys
sys.path.append('.')

from pynput import keyboard

from src.api.motor_api import MotorController

velocity = 1
dt = 3

api = MotorController(False)

api.enable()
print("API enabled.")
speed = [0, 0, 0, 0]

i1, i2 = 0, 1

def on_press(key):
    global speed, api, i1, i2
    if key == keyboard.KeyCode.from_char('t'):
        api.auto_tension = not api.auto_tension
        print("Auto tension:", api.auto_tension)
    elif key == keyboard.KeyCode.from_char('a'):
        speed[i1] += 0.1
    elif key == keyboard.KeyCode.from_char('q'):
        speed[i1] -= 0.1
    elif key == keyboard.KeyCode.from_char('w'):
        speed[i2] += 0.1
    elif key == keyboard.KeyCode.from_char('s'):
        speed[i2] -= 0.1
    elif key == keyboard.Key.up:
        i1 += 2
        i2 += 2
        i1 = i1 % 4
        i2 = i2 % 4
    elif key == keyboard.Key.left:
        speed[i1] += 0.1
        speed[i2] += 0.1
    elif key == keyboard.Key.right:
        speed[i2] -= 0.1
        speed[i1] -= 0.1
    else:
        speed = [0, 0, 0, 0]
    print(speed)
    api.set_velocity(speed)

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

api.enable_log('data/DELETE')

print("Manual control ready")
listener.join()

api.disable_log()
api.disable()
