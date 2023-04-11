import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt


# Termination summary 
print("Termination summary")
path = '/home/spencer/Documents/thesis/pcr_control/data/sorted_data/*/*/meta_data.txt'

files = glob.glob(path)
durations = [0 for _ in range(5)]

for file in files:
    f = open(file, 'rb')
    lines = f.readlines()
    config = int(file.split('/')[-3]) 

    if config == 1:
        continue
    
    for i in range(1,len(lines)):
        trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")

        
        durations[config] += float(duration)

durations = [durations[0]] + durations[2:]
print("Individual", durations)
print("mean", sum(durations) / len(durations) / 60)
print("std", np.std(durations) / 60)