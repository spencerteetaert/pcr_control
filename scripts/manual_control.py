import time 
import sys
import threading
sys.path.append('.')

from pynput import keyboard
import matplotlib.pyplot as plt

from src.api.motor_api import MotorController
from src.api.aurora_api import AuroraAPI
from blmc.pid import PID

api = MotorController(False)
time.sleep(2)
api.enable()
print("API enabled.")
pos = [0, 0, 0, 0]
aurora = AuroraAPI(verbose=False)
step = 0.1
# step = 0.005

i1, i2 = 0,1
FLAG = time.time() 

def on_press(key):
    global pos, api, i1, i2, FLAG

    override = False
    if key == keyboard.KeyCode.from_char('t'):
        api.auto_tension = not api.auto_tension
        print("Auto tension:", api.auto_tension)
    elif key == keyboard.KeyCode.from_char('a'):
        override = True
        pos[i1] += step
    elif key == keyboard.KeyCode.from_char('q'):
        override = True
        pos[i1] -= step
    elif key == keyboard.KeyCode.from_char('w'):
        override = True
        pos[i2] -= step
        # pos[i1] += step/16
    elif key == keyboard.KeyCode.from_char('s'):
        override = True
        pos[i2] += step
        # pos[i1] -= step/16
        # print("TIME:", time.time()- FLAG)

        FLAG = time.time() 

    # elif key == keyboard.Key.up:
    #     i1 += 2
    #     i2 += 2
    #     i1 = i1 % 4
    #     i2 = i2 % 4
    # elif key == keyboard.Key.left:
    #     pos[i1] += step
    #     pos[i2] += step
    # elif key == keyboard.Key.right:
    #     pos[i2] -= step
    #     pos[i1] -= step
    else:
        override = True
        if abs(pos[0]) > 0.03:
            pos[0] += 0.03 * ((pos[0] < 0) - 0.5)*2
        else:
            pos[0] = 0
        if abs(pos[1]) > 0.03:
            pos[1] += 0.03 * ((pos[1] < 0) - 0.5)*2
        else:
            pos[1] = 0
        if abs(pos[2]) > 0.03:
            pos[2] += 0.03 * ((pos[2] < 0) - 0.5)*2
        else:
            pos[2] = 0
        if abs(pos[3]) > 0.03:
            pos[3] += 0.03 * ((pos[3] < 0) - 0.5)*2
        else:
            pos[3] = 0

    # print(pos, i1, i2)
    print(aurora.get_position())

    if override:
        api._set_position(pos)
    else:
        api.set_position([pos[0], pos[2]])

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

pos[0] = api.mtr_datas[0].mtr1.position.value
pos[1] = api.mtr_datas[0].mtr2.position.value



aurora.start_tracking()

# Update endpoint
print("POSITION", aurora.get_position()[:2])
# print("End point updated:", model.update_end_point(aurora.get_position()[:2]))
# print("Controller position updated.", model.end_point)
input("1: Press enter to continue")

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

# plt.show()

# api.enable_log('data/DELETE')

print("Manual control ready")
listener.join()
# t.kill = True
# t.thread.join()

# api.disable_log()
api.disable()
