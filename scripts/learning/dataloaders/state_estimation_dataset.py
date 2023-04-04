import glob 
import os
import time as t

import torch
from torch.utils.data import Dataset, DataLoader
torch.manual_seed(0)

default_device = 'cuda' if torch.cuda.is_available() else 'cpu'

class PCRDataSet(Dataset):
    def __init__(self, folder, min_travel_distance=0, run_types=['SUCCESS'], device=default_device, debug=False):
        '''
        Args: 
            folder (str): parent folder of data collected
            min_travel_distance (float): minimum distance (m) in task space required to load in data
            run_types (list): termination conditions that should be included in the dataset. Options are:
                "SUCCESS"
                "STALLED"
                "MODE SWITCH" 
                "RECOVERY"
                "AURORA LOST"
                "USER TERMINATION"

        '''
        print("Loading in data...")
        START = t.time()
        self.debug = debug
        self.run_types = run_types

        files_meta_data = glob.glob(os.path.join(folder, '*', '*','meta_data.txt'))

        self.device = device

        self.aurora_data = []
        self.motor_data = []
        self.aurora_data_map = []

        # Load in data 
        for file_meta_data in files_meta_data:
            par_path = '/' + os.path.join(*file_meta_data.split('/')[:-1])
            configuration_number = int(file_meta_data.split('/')[-3])

            f_meta_data = open(file_meta_data, 'rb')
            lines = f_meta_data.readlines()
            f_meta_data.close() 

            for i in range(1,len(lines)):
                trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")
                
                # Read in aurora 
                aurora_file = os.path.join(par_path, trial_prefix + '_aurora.txt')
                f_aurora = open(aurora_file, 'rb')
                lines_aurora = f_aurora.readlines()[1:-1]
                f_aurora.close()

                # Filters 
                if ((float(start_x) - float(end_x))**2 + (float(start_y) - float(end_y))**2)**0.5 < min_travel_distance:
                    continue # Too little distance travelled 
                if termination_cause not in self.run_types:
                    continue # Not right type 
                if len(lines_aurora) < 2:
                    continue # Not long enough 

                aurora_curr_idx = 0
                aurora_start_idx = 0 
                aurora_end_idx = len(lines_aurora)


                # Read in motor data 
                motor_file = os.path.join(par_path, trial_prefix + '_motor.txt')
                f_motor = open(motor_file, 'rb')
                lines_motor = f_motor.readlines()[2:-1] # disregard first motor line as it may not have setpoints 
                f_motor.close()

                # Time alignment 
                # Find first valid aurora reading  
                start_time = float(lines_aurora[aurora_curr_idx].decode().split(",")[0])
                to_add_motor_data = {
                    "time": [],
                    "command_i1": [],
                    "command_i2": [],
                    "set_vel1": [],
                    "set_vel2": [],
                    "i1": [],
                    "i2": [],
                    "pos1": [],
                    "pos2": [],
                    "vel1": [],
                    "vel2": [],
                }

                to_add_start_idx = -1
                to_add_end_idx = 0
                for j in range(0, len(lines_motor)):
                    time,command_i1,command_i2,set_pos1,set_pos2,set_vel1,set_vel2,i1,i2,pos1,pos2,vel1,vel2,recovery,ctrl_type = lines_motor[j].decode().split(",")
                    time = float(time)

                    to_add_motor_data["time"] += [float(time)]
                    to_add_motor_data["command_i1"] += [float(command_i1)]
                    to_add_motor_data["command_i2"] += [float(command_i2)]
                    to_add_motor_data["set_vel1"] += [float(set_vel1)]
                    to_add_motor_data["set_vel2"] += [float(set_vel2)]
                    to_add_motor_data["i1"] += [float(i1)]
                    to_add_motor_data["i2"] += [float(i2)]
                    to_add_motor_data["pos1"] += [float(pos1)]
                    to_add_motor_data["pos2"] += [float(pos2)]
                    to_add_motor_data["vel1"] += [float(vel1)]
                    to_add_motor_data["vel2"] += [float(vel2)]

                    # Find aurora reading match 
                    if time > start_time:
                        # Finds first valid aurora reading 
                        if to_add_start_idx == -1:
                            aurora_start_idx = aurora_curr_idx
                            to_add_start_idx = j
                            
                        to_add_end_idx = len(to_add_motor_data["time"])

                        aurora_curr_idx += 1
                        if to_add_start_idx != -1:
                            # Synchronizes with aurora data. Assumes rate motor >> rate aurora
                            aurora_end_idx = aurora_curr_idx
                            if aurora_curr_idx == len(lines_aurora):
                                to_add_end_idx -= 1
                                break
                            
                        start_time = float(lines_aurora[aurora_curr_idx].decode().split(",")[0])

                if aurora_end_idx - aurora_start_idx < 2:
                    # Not enough measurements 
                    continue 

                to_add_motor_data["time"] = to_add_motor_data["time"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["command_i1"] = to_add_motor_data["command_i1"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["command_i2"] = to_add_motor_data["command_i2"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["set_vel1"] = to_add_motor_data["set_vel1"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["set_vel2"] = to_add_motor_data["set_vel2"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["i1"] = to_add_motor_data["i1"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["i2"] = to_add_motor_data["i2"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["pos1"] = to_add_motor_data["pos1"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["pos2"] = to_add_motor_data["pos2"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["vel1"] = to_add_motor_data["vel1"][to_add_start_idx:to_add_end_idx]
                to_add_motor_data["vel2"] = to_add_motor_data["vel2"][to_add_start_idx:to_add_end_idx]
                self.motor_data += [to_add_motor_data]

                time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = lines_aurora[0].decode().split(",")
                to_add_aurora_data = {
                    "time": [float(time)],
                    "x": [float(x)],
                    "y": [float(y)],
                    "z": [float(z)],
                    "configuration_number": [float(configuration_number)],
                    "trial_prefix": [trial_prefix],
                    "weight": [0]
                }
                motor_index = 0

                # Add Aurora data 
                for j in range(aurora_start_idx, aurora_end_idx):
                    # Add data 
                    time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = lines_aurora[j].decode().split(",")
                    time = float(time)
                    x, y, z = float(x), float(y), float(z)

                    if j == aurora_start_idx:
                        last_time = time
                        lastx, lasty, lastz = x, y, z
                        continue

                    sx = (x - lastx)/(time - last_time)
                    sy = (y - lasty)/(time - last_time)
                    sz = (z - lastz)/(time - last_time)

                    curr_motor_time = self.motor_data[-1]['time'][motor_index]
                    while curr_motor_time < time: 
                        dt = curr_motor_time - last_time

                        to_add_aurora_data["time"] += [float(last_time + dt)]
                        to_add_aurora_data["x"] += [float(lastx + dt*sx)]
                        to_add_aurora_data["y"] += [float(lasty + dt*sy)]
                        to_add_aurora_data["z"] += [float(lastz + dt*sz)]
                        to_add_aurora_data["configuration_number"] += [configuration_number]
                        to_add_aurora_data["trial_prefix"] += [trial_prefix]
                        to_add_aurora_data["weight"] += [1]

                        motor_index += 1
                        if motor_index == len(self.motor_data[-1]['time']):
                            break
                        curr_motor_time = self.motor_data[-1]['time'][motor_index]

                    last_time = time
                    lastx, lasty, lastz = x, y, z
                    
                self.aurora_data += [to_add_aurora_data]

        print(f"Data loaded in {t.time() - START:.3}s")
        print("Total data points:", len(self))

    def __len__(self):
        return len(self.aurora_data)

    def __getitem__(self, idx):
        '''
        Data: 
        - Starting position 
        - Configuration number 
        - All subsequent motor feedback 

        Label: 
        - Interpolated state values for each motor feedback step 
        '''

        position_data = torch.tensor([
            self.aurora_data[idx]['x'][0], 
            self.aurora_data[idx]['y'][0],
            self.aurora_data[idx]['configuration_number'][0]
            ]).to(self.device)

        label = torch.tensor([
            self.aurora_data[idx]['x'][1:],
            self.aurora_data[idx]['y'][1:],
            self.aurora_data[idx]['weight'][1:]
            ]).to(self.device).T
        

        feedback_data = torch.tensor([
            self.motor_data[idx]['i1'], 
            self.motor_data[idx]['i2'],
            self.motor_data[idx]['pos1'],
            self.motor_data[idx]['pos2'],
            self.motor_data[idx]['vel1'],
            self.motor_data[idx]['vel2'],
        ]).to(self.device).T

        return (position_data, feedback_data), label
        

if __name__ == '__main__':
    # dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data')
    dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev')

    train_dataloader = DataLoader(dataset, batch_size=1, shuffle=True)

    counter = 0 
    for (position_data, feedback_data), labels in train_dataloader:
        print("Position data shape:", position_data.shape)
        print("feedback_data data shape:", feedback_data.shape)
        print("Labels shape:", labels.shape)
        break