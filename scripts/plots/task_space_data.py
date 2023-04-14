import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# Task space distribution 
path = '/home/spencer/Documents/thesis/pcr_control/data/*/*/*_aurora.txt'

files = glob.glob(path)
size = 25

accum = {
    'readings': 0,
}
dist = np.zeros((size, size))
vel = np.zeros((size, size))
acc = np.zeros((size, size))

vx_min, vx_max, vy_min, vy_max = -0.524186952021008, 0.49993044928603675, -0.2534799966521659, 1.074085009517687
ax_min, ax_max, ay_min, ay_max = -1.7126083215719718, 1.9316183541563128, -5.095962738546678, 2.2758181560188384

for file in files:
    f = open(file, 'rb')
    lines = f.readlines()

    for i in range(1,len(lines)-1):
        time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = lines[i].decode().split(",")
        accum['readings'] += 1

        # Position 
        _y, _x = min(max(int(float(y)*size*2), 0), size-1), min(max(int(float(x)*size*2), 0), size-1)
        dist[_y, _x] += 1
        
        # Velocity + Acceleration 
        curr_x, curr_y, curr_time = float(x), float(y), float(time)
        if i > 1: 
            dt = curr_time - last_time
            curr_vx = (curr_x - last_x) / dt
            curr_vy = (curr_y - last_y) / dt

            _y, _x = min(max(int((curr_vy - vy_min) * size / (vy_max - vy_min)), 0), size-1), min(max(int((curr_vx - vx_min) * size / (vx_max - vx_min)), 0), size-1)
            vel[_y, _x] += 1

            if i > 2: 
                curr_ax = (curr_vx - last_vx) / dt
                curr_ay = (curr_vy - last_vy) / dt

                _y, _x = min(max(int((curr_ay - ay_min) * size / (ay_max - ay_min)), 0), size-1), min(max(int((curr_ax - ax_min) * size / (ax_max - ax_min)), 0), size-1)
                acc[_y, _x] += 1

            last_vx, last_vy = curr_vx, curr_vy
        last_x, last_y, last_time = curr_x, curr_y, curr_time


def forceAspect(ax,aspect=1):
    im = ax.get_images()
    extent =  im[0].get_extent()
    ax.set_aspect(abs((extent[1]-extent[0])/(extent[3]-extent[2]))/aspect)

print(json.dumps(accum, indent=2))

fig, axs = plt.subplots(3, 1)
fig.set_size_inches(4, 16)
fig.tight_layout(pad=4)

map = axs[0].imshow(dist[::-1,...], extent=[0, 0.5, 0, 0.5])
axs[0].set_xlabel('X position [m]')
axs[0].set_ylabel('Y position [m]')
fig.colorbar(map, ax=axs[0])

map = axs[1].imshow(vel[::-1,...], norm=LogNorm(vmin=1, vmax=10000), extent=[vx_min, vx_max, vy_min, vy_max])
axs[1].set_xlabel('X velocity [m/s]')
axs[1].set_ylabel('Y velocity [m/s]')
fig.colorbar(map, ax=axs[1])

map = axs[2].imshow(acc[::-1,...], norm=LogNorm(vmin=1, vmax=10000), extent=[ax_min, ax_max, ay_min, ay_max])
axs[2].set_xlabel('X acceleration [m/s^2]')
axs[2].set_ylabel('Y acceleration [m/s^2]')
fig.colorbar(map, ax=axs[2])

forceAspect(axs[0],aspect=1)
forceAspect(axs[1],aspect=1)
forceAspect(axs[2]  ,aspect=1)

fig.show()

input()