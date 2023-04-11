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
    'distance_travelled':{
        'success': [],
        'stalled': [],
        'aurora_lost': [],
        'recovery': [],
        'mode_switch': [],
        'user_terminated': [],
    },
    'average_velocity':{
        'success': [],
        'stalled': [],
        'aurora_lost': [],
        'recovery': [],
        'mode_switch': [],
        'user_terminated': [],
    },
    'duration':{
        'success': [],
        'stalled': [],
        'aurora_lost': [],
        'recovery': [],
        'mode_switch': [],
        'user_terminated': [],
    },
}
for file in files:
    f = open(file, 'rb')
    lines = f.readlines()
    for i in range(1,len(lines)):
        trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")

        if termination_cause == 'SUCCESS':
            accum['distance_travelled']['success'] += [float(np.linalg.norm(np.array([float(end_x), float(end_y)]) - np.array([float(start_x), float(start_y)])))]
            accum['duration']['success'] += [float(duration)]
            accum['average_velocity']['success'] += [accum['distance_travelled']['success'][-1] / accum['duration']['success'][-1]]
        elif termination_cause == 'STALLED':
            accum['distance_travelled']['stalled'] += [float(np.linalg.norm(np.array([float(end_x), float(end_y)]) - np.array([float(start_x), float(start_y)])))]
            accum['duration']['stalled'] += [float(duration)]
            accum['average_velocity']['stalled'] += [accum['distance_travelled']['stalled'][-1] / accum['duration']['stalled'][-1]]
        elif termination_cause == 'MODE SWITCH':
            accum['distance_travelled']['mode_switch'] += [float(np.linalg.norm(np.array([float(end_x), float(end_y)]) - np.array([float(start_x), float(start_y)])))]
            accum['duration']['mode_switch'] += [float(duration)]
            accum['average_velocity']['mode_switch'] += [accum['distance_travelled']['mode_switch'][-1] / accum['duration']['mode_switch'][-1]]
        elif termination_cause == 'RECOVERY':
            accum['distance_travelled']['recovery'] += [float(np.linalg.norm(np.array([float(end_x), float(end_y)]) - np.array([float(start_x), float(start_y)])))]
            accum['duration']['recovery'] += [float(duration)]
            accum['average_velocity']['recovery'] += [accum['distance_travelled']['recovery'][-1] / accum['duration']['recovery'][-1]]
        elif termination_cause == 'AURORA LOST':
            accum['distance_travelled']['aurora_lost'] += [float(np.linalg.norm(np.array([float(end_x), float(end_y)]) - np.array([float(start_x), float(start_y)])))]
            accum['duration']['aurora_lost'] += [float(duration)]
            accum['average_velocity']['aurora_lost'] += [accum['distance_travelled']['aurora_lost'][-1] / accum['duration']['aurora_lost'][-1]]
        elif termination_cause == 'USER TERMINATION':
            accum['distance_travelled']['user_terminated'] += [float(np.linalg.norm(np.array([float(end_x), float(end_y)]) - np.array([float(start_x), float(start_y)])))]
            accum['duration']['user_terminated'] += [float(duration)]
            accum['average_velocity']['user_terminated'] += [accum['distance_travelled']['user_terminated'][-1] / accum['duration']['user_terminated'][-1]]
        else:
            print("Unhandled termination case:", termination_cause)

averages = {
    'average_distance_travelled':{
        'success': sum(accum['distance_travelled']['success']) / len(accum['distance_travelled']['success']),
        'stalled': sum(accum['distance_travelled']['stalled']) / len(accum['distance_travelled']['stalled']),
        'aurora_lost': sum(accum['distance_travelled']['aurora_lost']) / len(accum['distance_travelled']['aurora_lost']),
        'recovery': sum(accum['distance_travelled']['recovery']) / len(accum['distance_travelled']['recovery']),
        'mode_switch': sum(accum['distance_travelled']['mode_switch']) / len(accum['distance_travelled']['mode_switch']),
        'user_terminated': sum(accum['distance_travelled']['user_terminated']) / len(accum['distance_travelled']['user_terminated']),
    },
    'average_velocity':{
        'success': sum(accum['average_velocity']['success']) / len(accum['average_velocity']['success']),
        'stalled': sum(accum['average_velocity']['stalled']) / len(accum['average_velocity']['stalled']),
        'aurora_lost': sum(accum['average_velocity']['aurora_lost']) / len(accum['average_velocity']['aurora_lost']),
        'recovery': sum(accum['average_velocity']['recovery']) / len(accum['average_velocity']['recovery']),
        'mode_switch': sum(accum['average_velocity']['mode_switch']) / len(accum['average_velocity']['mode_switch']),
        'user_terminated': sum(accum['average_velocity']['user_terminated']) / len(accum['average_velocity']['user_terminated']),
    },
    'average_duration':{
        'success': sum(accum['duration']['success']) / len(accum['duration']['success']),
        'stalled': sum(accum['duration']['stalled']) / len(accum['duration']['stalled']),
        'aurora_lost': sum(accum['duration']['aurora_lost']) / len(accum['duration']['aurora_lost']),
        'recovery': sum(accum['duration']['recovery']) / len(accum['duration']['recovery']),
        'mode_switch': sum(accum['duration']['mode_switch']) / len(accum['duration']['mode_switch']),
        'user_terminated': sum(accum['duration']['user_terminated']) / len(accum['duration']['user_terminated']),
    },
}

totals = {
    'total_distance_travelled':{
        'success': sum(accum['distance_travelled']['success']),
        'stalled': sum(accum['distance_travelled']['stalled']),
        'aurora_lost': sum(accum['distance_travelled']['aurora_lost']),
        'recovery': sum(accum['distance_travelled']['recovery']),
        'mode_switch': sum(accum['distance_travelled']['mode_switch']),
        'user_terminated': sum(accum['distance_travelled']['user_terminated']),
    },
    'total_duration':{
        'success': sum(accum['duration']['success']),
        'stalled': sum(accum['duration']['stalled']),
        'aurora_lost': sum(accum['duration']['aurora_lost']),
        'recovery': sum(accum['duration']['recovery']),
        'mode_switch': sum(accum['duration']['mode_switch']),
        'user_terminated': sum(accum['duration']['user_terminated']),
    },
    'total_runs': {
        'success': len(accum['duration']['success']),
        'stalled': len(accum['duration']['stalled']),
        'aurora_lost': len(accum['duration']['aurora_lost']),
        'recovery': len(accum['duration']['recovery']),
        'mode_switch': len(accum['duration']['mode_switch']),
        'user_terminated': len(accum['duration']['user_terminated']),
    }
}

print(json.dumps(averages, indent=2))
print(json.dumps(totals, indent=2))