import os
import numpy as np
import matplotlib.pyplot as plt

save_dir = "data/test_trajs"

# Circle 
save_path = os.path.join(save_dir, 'circle.npy')
start_point = [0.25, 0.1]
radius = 0.15
duration = 10 

ts = np.arange(0, duration, 0.2)
xs = start_point[0] + radius*np.sin(ts/duration*2*np.pi)
ys = start_point[1] - radius*np.cos(ts/duration*2*np.pi) + radius
print("Circle", xs.shape, ys.shape)
to_save = np.vstack([xs, ys]).T
np.save(open(save_path, 'wb'), to_save)

plt.scatter(xs, ys)
plt.show()


# Square 
save_path = os.path.join(save_dir, 'square.npy')

start_point = [0.1, 0.05]
side_length = 0.3
duration = 10 

ts = np.arange(0, duration/4, 0.2)
xs = []
ys = []

xs += [np.arange(start_point[0], start_point[0] + side_length, side_length / len(ts))]
ys += [np.ones((len(xs[-1]),)) * start_point[1]]

ys += [np.arange(start_point[1], start_point[1] + side_length, side_length / len(ts))]
xs += [np.ones((len(ys[-1]),)) * (start_point[0] + side_length)]

xs += [np.arange(start_point[0] + side_length, start_point[0], -side_length / len(ts))]
ys += [np.ones((len(xs[-1]),)) * start_point[1] + side_length]

ys += [np.arange(start_point[1] + side_length, start_point[1], -side_length / len(ts))]
xs += [np.ones((len(ys[-1]),)) * start_point[0]]

xs = np.hstack(xs)
ys = np.hstack(ys)
print("Square", xs.shape, ys.shape)
to_save = np.vstack([xs, ys]).T
np.save(open(save_path, 'wb'), to_save)

plt.scatter(xs, ys)
plt.show()


# Zig Zag 
save_path = os.path.join(save_dir, 'zigzag.npy')

start_point = [0.05, 0.05]
width = 0.4
height = 0.3
duration = 10 

ts = np.arange(0, duration/4, 0.2)
ys = []
ys += [np.arange(start_point[1], start_point[1] + height, height / len(ts))]
ys += [np.arange(start_point[1] + height, start_point[1], -height / len(ts))]
ys += [np.arange(start_point[1], start_point[1] + height, height / len(ts))]
ys += [np.arange(start_point[1] + height, start_point[1], -height / len(ts))]
ys = np.hstack(ys)
xs = np.arange(start_point[0], start_point[0] + width, width / len(ys))

print("Zigzag", xs.shape, ys.shape)
to_save = np.vstack([xs, ys]).T
np.save(open(save_path, 'wb'), to_save)

plt.scatter(xs, ys)
plt.show()


# Horizontal line 
save_path = os.path.join(save_dir, 'hline.npy')

start_point = [0.1, 0.1]
length = 0.3
duration = 10

ts = np.arange(0, duration, 0.2)
xs = np.arange(start_point[0], start_point[0] + length, length / len(ts))
ys = np.ones((len(xs),)) * start_point[1]

print("HLine", xs.shape, ys.shape)
to_save = np.vstack([xs, ys]).T
np.save(open(save_path, 'wb'), to_save)

plt.scatter(xs, ys)
plt.show()


# Vertical line 
save_path = os.path.join(save_dir, 'vline.npy')

start_point = [0.25, 0.1]
length = 0.3
duration = 10

ts = np.arange(0, duration, 0.2)
ys = np.arange(start_point[1], start_point[1] + length, length / len(ts))
xs = np.ones((len(ys),)) * start_point[0]

print("VLine", xs.shape, ys.shape)
to_save = np.vstack([xs, ys]).T
np.save(open(save_path, 'wb'), to_save)

plt.scatter(xs, ys)
plt.show()
