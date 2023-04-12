import matplotlib.pyplot as plt
import numpy as np 

# # Feedback horizon 
# xs = 0.05 * np.array([1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 150, 200, 300]) 
# ys = [0.03104, 0.02609, 0.02372, 0.01096, 0.01027, 0.01057, 0.01176, 0.00978, 0.01074, 0.01137, 0.01146, 0.00925, 0.00867, 0.01179]

# fig = plt.figure(figsize=(7,4))
# plt.plot(xs, ys)
# plt.xlabel("Feedback Horizon (s)")
# plt.ylabel("Best Validation RMSE (A)")
# plt.show()
# plt.cla()

# # Configuration 
# xs = [0, 2, 4, 6, 8, 10, 15, 20]
# ys = [0.02345, 0.02407, 0.0245, 0.02254, 0.02242, 0.02294, 0.02274, 0.024]

# fig = plt.figure(figsize=(7,4))
# plt.plot(xs, ys)
# plt.xlabel("Configuration Parameter Size")
# plt.ylabel("Best Validation RMSE (A)")
# plt.show()
# plt.cla()


# Weighting 
X = ['Unweighted','Linearly Increasing','Exponentially Increasing','Quadratic']
X_axis = np.arange(len(X))
ys = [
    0.008080933699742532,
    0.008275709759730559,
    0.008250417366910439,
    0.00814957064218246,
    0.008315084466280846,
    0.00819726506821238,
    0.008176743052899837,
    0.008002553815738512,
    0.00831165830962933,
    0.00879219425125764,
    0.008231024616039716,
    0.008300100453197956,
    0.008197931572794914,
    0.008294874802231789,
    0.008615118702157186,
    0.007940788359309617,
    0.008323464041145949,
    0.008349377303742446,
    0.008166107241637431,
    0.008355809268183433,
    0.008000754607984653,
    0.008297537560932912,
    0.008317144802556587,
    0.00836026668548584
]

fig = plt.figure(figsize=(10,4))

plt.bar(X_axis - 0.30, ys[:4], 0.1, label = 'Model 1')
plt.bar(X_axis - 0.18, ys[4:8], 0.1, label = 'Model 2')
plt.bar(X_axis - 0.06, ys[8:12], 0.1, label = 'Model 3')
plt.bar(X_axis + 0.06, ys[12:16], 0.1, label = 'Model 4')
plt.bar(X_axis + 0.18, ys[16:20], 0.1, label = 'Model 5')
plt.bar(X_axis + 0.30, ys[20:], 0.1, label = 'Model 6')

plt.legend(loc='lower right')
plt.xlabel("Weight Type")
plt.ylabel("Best Validation RMSE (A)")
plt.xticks(X_axis, X)
plt.show()
plt.cla()