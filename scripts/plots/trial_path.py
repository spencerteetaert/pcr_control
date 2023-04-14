import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# Task space distribution 
mux = 0# 0: CC, 1: PID 
mux1 = 1 # 0: point, 1: traj 

if mux: 
    path = '/home/spencer/Documents/thesis/pcr_control/data/PID_test*/meta_data.txt'
else:
    path = '/home/spencer/Documents/thesis/pcr_control/data/CC_test*/meta_data.txt'

files = glob.glob(path)

ref_trajs = [
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/square.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/circle.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/zigzag.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/hline.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/vline.npy", 'rb')),
]

print("ref durations", [len(traj) * 0.2 for traj in ref_trajs])

dists = []
for traj in ref_trajs:
    delta = traj[1:] - traj[:-1]
    dists += [np.sum(np.linalg.norm(delta, axis=1))]

print("ref lengths", dists)


size = 500

accum = {
    'readings': 0,
}
x_min, x_max, y_min, y_max = 0, 0.5, 0, 0.5
rmses = [0 for _ in range(5)]
path_lengths = [0 for _ in range(5)]
durations = [0 for _ in range(5)]

plt.figure(figsize=(10, 7))

if mux: 
    plt.title("PID Controller")
else:
    plt.title("Differential CC Controller")

plt.tight_layout(pad=2)
ax0 = plt.subplot(2,3,1)
ax0.set_ylabel('Y position [m]')
ax1 = plt.subplot(2,3,2)
ax2 = plt.subplot(2,3,3)
ax3 = plt.subplot(2,3,4)
ax3.set_ylabel('Y position [m]')
ax4 = plt.subplot(2,3,5)

axs = [ax0, ax1, ax2, ax3, ax4]

for i in range(5):
    axs[i].set_xlabel('X position [m]')
    axs[i].set_xlim(0, 0.5)
    axs[i].set_ylim(0, 0.5)
    # axs[i].figure.set_figheight(2)
    axs[i].set_aspect(1)
    axs[i].plot(ref_trajs[i][:,0], ref_trajs[i][:,1], label="Reference Trajectory")

for file in files:
    if mux1: 
        if '_pos' in file:
            continue
    else:
        if '_pos' not in file:
            continue
    f = open(file, 'rb')
    lines = f.readlines()
    trials_to_plot = []

    counter = 0 
    for i in range(1, len(lines)):
        trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")
        if termination_cause == 'TRACKING_FINISHED':
            trials_to_plot += [os.path.join('/'.join(file.split('/')[:-1]), trial_prefix + '_aurora.txt')]
            durations[counter] += float(duration)
            counter += 1

    assert len(trials_to_plot) == 5, trials_to_plot

    for i, trial in enumerate(trials_to_plot):
        _f = open(trial, 'rb')
        _lines = _f.readlines()
        xs = []
        ys = []

        for j in range(1,len(_lines)-1):
            time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = _lines[j].decode().split(",")
            x, y = float(x), float(y)
            accum['readings'] += 1

            # Position 
            xs += [x]
            ys += [y]
        
        axs[i].plot(xs, ys, label=trial.split('/')[-1])

        tracked = np.array([xs, ys]).T

        if mux1:
            rmse = (np.sum(np.linalg.norm(ref_trajs[i][:len(tracked)] - tracked, axis=1)**2) / len(tracked))**0.5
            rmses[i] += rmse


        delta = tracked[1:] - tracked[:-1]
        dist = np.linalg.norm(delta, axis=1)
        path_lengths[i] += np.sum(dist)

if not mux1:
    print("rmses", [rmses[i] for i in range(5)])
    print("path_lengths", [path_lengths[i] for i in range(5)])
    print("durations", [durations[i] for i in range(5)])
else:
    print("rmses", [rmses[i] / 3 for i in range(5)])
    print("path_lengths", [path_lengths[i] / 3 for i in range(5)])
    print("durations", [durations[i] / 3 for i in range(5)])

print(json.dumps(accum, indent=2))

plt.show()
input()