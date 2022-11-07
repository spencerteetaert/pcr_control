import time 
import sys
sys.path.append('.')

from pynput import keyboard

from src.api.single_motor_api import MotorController

velocity = 1
dt = 3

api = MotorController()
api.enable()

speed = [0, 0, 0, 0]

def on_press(key):
    global speed, api
    if key == keyboard.Key.up:
        print("UP")
        speed[0] += 0.1
    elif key == keyboard.Key.down:
        print("DOWN")
        speed[0] -= 0.1
    else:
        speed[0] = 0
    api.set_velocity(speed)

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
# listener.start()

print("Manual control ready")
api.set_velocity([0.2, 0.2, 0, 0])
time.sleep(2)
# listener.join()

api.disable

# k = 0
# while k != 'q':
#     k = keyboard.read_key()
#     print(k)

#     if k == 'up':
#         api.set_velocity([0.1, 0, 0, 0])
#     elif k == 'down':
#         api.set_velocity([-0.1, 0, 0, 0])
#     else:
#         api.set_velocity([0, 0, 0, 0])

# api.disable()