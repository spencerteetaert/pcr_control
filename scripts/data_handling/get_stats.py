import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt

# Termination summary 
print("Termination summary")
path = '/home/spencer/Documents/thesis/pcr_control/data/*/*/meta_data.txt'

files = glob.glob(path)
accum = {
    'duration': 0,
    'termination_causes':{
        'success': 0,
        'stalled': 0,
        'aurora_lost': 0,
        'recovery': 0,
        'mode_switch': 0,
        'user_terminated': 0,
    },
}
for file in files:
    f = open(file, 'rb')
    lines = f.readlines()
    for i in range(1,len(lines)):
        trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")
        
        accum['duration'] += float(duration)

        if termination_cause == 'SUCCESS':
            accum['termination_causes']['success'] += 1
        elif termination_cause == 'STALLED':
            accum['termination_causes']['stalled'] += 1
        elif termination_cause == 'MODE SWITCH':
            accum['termination_causes']['mode_switch'] += 1
        elif termination_cause == 'RECOVERY':
            accum['termination_causes']['recovery'] += 1
        elif termination_cause == 'AURORA LOST':
            accum['termination_causes']['aurora_lost'] += 1
        elif termination_cause == 'USER TERMINATION':
            accum['termination_causes']['user_terminated'] += 1
        else:
            print("Unhandled termination case:", termination_cause)

print(json.dumps(accum, indent=2))


# Task space distribution 
print("Aurora summary")
path = '/home/spencer/Documents/thesis/pcr_control/data/*/*/*_aurora.txt'

files = glob.glob(path)
size = 25

accum = {
    'readings': 0,
}
dist = np.zeros((size, size))

for file in files:
    f = open(file, 'rb')
    lines = f.readlines()
    for i in range(1,len(lines)-1):
        time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = lines[i].decode().split(",")

        _y, _x = min(max(int(float(y)*size*2), 0), size-1), min(max(int(float(x)*size*2), 0), size-1)
        dist[_y, _x] += 1
        accum['readings'] += 1


print(json.dumps(accum, indent=2))

plt.imshow(dist)
plt.colorbar()
plt.show()