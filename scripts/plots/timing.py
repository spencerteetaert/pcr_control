import numpy as np 
import matplotlib.pyplot as plt 
  
# X = ['Create Controller','Get Command','Update End Point','Update Goal Point','Per Iteration']
# times_PID = [0.00021100258827209473, 1.1658668518066406e-07, 2.4495601654052735e-05, 7.998943328857422e-07]
# times_PID += [sum(times_PID[1:])]

# times_cc_diff = [1.1353731155395508e-05, 1.2135505676269531e-07, 0.0003642268180847168, 1.9001960754394532e-07]
# times_cc_diff += [sum(times_cc_diff[1:])]

# # times_cc = [7.662153244018555e-05, 1.373291015625e-07, 0.0003567368984222412, 0.011107032537460327]
# # times_cc += [sum(times_cc[1:])]

# # times_learned = [0.0011843225955963134, 2.1004676818847656e-07, 1.8477439880371094e-07, 0.0003562905788421631] # ORIGINAL
# times_learned = [0.0011843225955963134, 2.1004676818847656e-07, 0.0003562905788421631, 1.8477439880371094e-07]
# times_learned += [sum(times_learned[1:])]

# X_axis = np.arange(len(X))
  
# plt.bar(X_axis - 0.25, times_PID, 0.2, label = 'PID Controller')
# plt.bar(X_axis, times_cc_diff, 0.2, label = 'Differential CC Controller')
# plt.bar(X_axis + 0.25, times_learned, 0.2, label = 'Learning-Based Controller')
  
# plt.xticks(X_axis, X)
# plt.xlabel("Command Type")
# plt.ylabel("Average Runtime (s)")
# # plt.title("Runtime Breakdown of Proposed Controllers")
# plt.legend()
# plt.show()



times_PID = [0.00021100258827209473, 1.1658668518066406e-07, 2.4495601654052735e-05, 7.998943328857422e-07]
times_PID += [sum(times_PID[1:])]

times_cc_diff = [1.1353731155395508e-05, 1.2135505676269531e-07, 0.0003642268180847168, 1.9001960754394532e-07]
times_cc_diff += [sum(times_cc_diff[1:])]

# times_cc = [7.662153244018555e-05, 1.373291015625e-07, 0.0003567368984222412, 0.011107032537460327]
# times_cc += [sum(times_cc[1:])]

# times_learned = [0.0011843225955963134, 2.1004676818847656e-07, 1.8477439880371094e-07, 0.0003562905788421631] # ORIGINAL
times_learned = [0.0011843225955963134, 2.1004676818847656e-07, 0.0003562905788421631, 1.8477439880371094e-07]
times_learned += [sum(times_learned[1:])]

print(times_PID, times_cc_diff, times_learned)

plt.bar(-0.05, times_PID[-1], 0.045, label = 'PID Controller')
plt.bar(0, times_cc_diff[-1], 0.045, label = 'Differential CC Controller')
plt.bar(0.05, times_learned[-1], 0.045, label = 'Learning-Based Controller')
  
plt.xticks([])
plt.ylabel("Average Runtime (s)")
# plt.title("Runtime Breakdown of Proposed Controllers")
plt.legend()
plt.show()