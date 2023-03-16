import os
import glob
import json

path = '/home/jimmy/spencer_thesis/pcr_control/logs/*/*/meta_data.txt'

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