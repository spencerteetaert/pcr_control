import time 
import sys
sys.path.append('.')

from pynput import keyboard

from src.api.single_motor_api import MotorController

velocity = 1
dt = 3

api = MotorController()
api.enable()

print("API enabled.")
speed = [0, 0, 0, 0]

def on_press(key):
    global speed, api
    if key == keyboard.KeyCode.from_char('t'):
        api.auto_tension = not api.auto_tension
    elif key == keyboard.KeyCode.from_char('w'):
        speed[0] += 0.1
    elif key == keyboard.KeyCode.from_char('s'):
        speed[0] -= 0.1
    elif key == keyboard.KeyCode.from_char('q'):
        speed[1] += 0.1
    elif key == keyboard.KeyCode.from_char('a'):
        speed[1] -= 0.1
    elif key == keyboard.Key.left:
        speed[1] -= 0.1
        speed[0] += 0.1
    elif key == keyboard.Key.right:
        speed[0] -= 0.1
        speed[1] += 0.1
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
print("Manual control ready")
listener.join()

api.disable()
