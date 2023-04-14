import os
import glob
import json
import numpy as np
import tqdm 
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# Task space distribution 
path = '/home/spencer/Documents/thesis/pcr_control/data/*/*/*_motor.txt'

files = glob.glob(path)
size = 25

accum = {
    'readings': 0,
}
dist = np.zeros((size, size))
vel = np.zeros((size, size))
acc = np.zeros((size, size))

i1, i2, vi1, vi2, ai1, ai2 = [], [], [], [], [], []
i1_min, i1_max, i2_min, i2_max = -2.274606227874756, 2.5731682777404785, -2.281010627746582, 3.364811897277832
vi1_min, vi1_max, vi2_min, vi2_max = -225.12433722163308, 341.98464999338364, -298.0684813563812, 296.3470831664664
ai1_min, ai1_max, ai2_min, ai2_max = -43269.116975861456, 34361.743925538845, -42359.83867371155, 42922.1210987319

for file in tqdm.tqdm(files):
    f = open(file, 'rb')
    lines = f.readlines()

    for i in range(1,len(lines)-1):
        time,command_i1,command_i2,set_pos1,set_pos2,set_vel1,set_vel2,_i1,_i2,pos1,pos2,vel1,vel2,recovery,ctrl_type = lines[i].decode().split(",")
        curr_i1, curr_i2, curr_time = float(_i1), float(_i2), float(time)

        # i1 += [curr_i1]
        # i2 += [curr_i2]
        accum['readings'] += 1

        # Position 
        _y, _x = min(max(int((curr_i2 - i2_min)*size / (i2_max - i2_min)), 0), size-1), min(max(int((curr_i1 - i1_min)*size / (i1_max - i1_min)), 0), size-1)
        dist[_y, _x] += 1
        
        if i > 1: 
            dt = curr_time - last_time
            curr_vi1 = (curr_i1 - last_x) / dt
            curr_vi2 = (curr_i2 - last_y) / dt
            # vi1 += [curr_vi1]
            # vi2 += [curr_vi2]

            # Velocity 
            _y, _x = min(max(int((curr_vi2 - vi2_min) * size / (vi2_max - vi2_min)), 0), size-1), min(max(int((curr_vi1 - vi1_min) * size / (vi1_max - vi1_min)), 0), size-1)
            vel[_y, _x] += 1

            if i > 2: 
                curr_ai1 = (curr_vi1 - last_vi1) / dt
                curr_ai2 = (curr_vi2 - last_vy) / dt
                # ai1 += [curr_ai1]
                # ai2 += [curr_ai2]

                # Acceleration 
                _y, _x = min(max(int((curr_ai2 - ai2_min) * size / (ai2_max - ai2_min)), 0), size-1), min(max(int((curr_ai1 - ai1_min) * size / (ai1_max - ai1_min)), 0), size-1)
                acc[_y, _x] += 1

            last_vi1, last_vy = curr_vi1, curr_vi2
        last_x, last_y, last_time = curr_i1, curr_i2, curr_time

# print("pos", min(i1), max(i1), min(i2), max(i2))
# print("vel", min(vi1), max(vi1), min(vi2), max(vi2))
# print("acc", min(ai1), max(ai1), min(ai2), max(ai2))
# raise
def forceAspect(ax,aspect=1):
    im = ax.get_images()
    extent =  im[0].get_extent()
    ax.set_aspect(abs((extent[1]-extent[0])/(extent[3]-extent[2]))/aspect)

print(json.dumps(accum, indent=2))
fig, axs = plt.subplots(3, 1)
fig.set_size_inches(4, 16)
fig.tight_layout(pad=4)

map = axs[0].imshow(dist[::-1,...], norm=LogNorm(vmin=1, vmax=10e6), extent=[i1_min, i1_max, i2_min, i2_max])
axs[0].set_xlabel('Motor 1 Current [A]')
axs[0].set_ylabel('Motor 2 Current [A]')
fig.colorbar(map, ax=axs[0])

map = axs[1].imshow(vel[::-1,...], norm=LogNorm(vmin=1, vmax=10e6), extent=[vi1_min, vi1_max, vi2_min, vi2_max])
axs[1].set_xlabel('Motor 1 Current Velocity [A/s]')
axs[1].set_ylabel('Motor 2 Current Velocity [A/s]')
fig.colorbar(map, ax=axs[1])

map = axs[2].imshow(acc[::-1,...], norm=LogNorm(vmin=1, vmax=10e6), extent=[ai1_min, ai1_max, ai2_min, ai2_max])
axs[2].set_xlabel('Motor 1 Current Accerlation [A/s^2]')
axs[2].set_ylabel('Motor 2 Current Accerlation [A/s^2]')
fig.colorbar(map, ax=axs[2])

forceAspect(axs[0],aspect=1)
forceAspect(axs[1],aspect=1)
forceAspect(axs[2],aspect=1)

fig.show()

input()