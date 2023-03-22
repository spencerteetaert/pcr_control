import glob 
import os
import time as t

import torch
from torch.utils.data import Dataset, DataLoader
torch.manual_seed(0)

class PCRDataSet(Dataset):
    def __init__(self, folder, feedback_horizon=0, prediction_horizon=10, min_travel_distance=0, run_types=['SUCCESS'], debug=False):
        '''
        Args: 
            folder (str): parent folder of data collected
            feedback_horizon (int): number of motor feedback steps to include in data
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
        self.feedback_horizon = feedback_horizon
        self.prediction_horizon = prediction_horizon
        assert self.prediction_horizon <= 20, "Prediction horizon should be less than 20"
        self.debug = debug
        self.run_types = run_types

        files_meta_data = glob.glob(os.path.join(folder, '*', '*','meta_data.txt'))


        self.aurora_data = []
        self.indexable_aurora_data = []
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
                    continue
                if termination_cause not in self.run_types:
                    continue
                if len(lines_aurora) < 2:
                    continue

                if self.feedback_horizon != 0:
                    # Read in motor data 
                    motor_file = os.path.join(par_path, trial_prefix + '_motor.txt')
                    f_motor = open(motor_file, 'rb')
                    lines_motor = f_motor.readlines()[2:-1] # disregard first motor line as it may not have setpoints 
                    f_motor.close()

                    # Time alignment 
                    # Find first valid aurora reading  
                    aurora_curr_idx = 0
                    aurora_start_idx = 0 
                    start_time = float(lines_aurora[aurora_curr_idx].decode().split(",")[0])
                    to_add_data = []
                    to_add_mappings = []
                    to_add_start_idx = -1
                    to_add_end_idx = 0
                    for j in range(0, len(lines_motor)):
                        time,command_i1,command_i2,set_pos1,set_pos2,set_vel1,set_vel2,i1,i2,pos1,pos2,vel1,vel2,recovery,ctrl_type = lines_motor[j].decode().split(",")
                        time = float(time)

                        to_add_data += [{
                            "time": float(time),
                            "configuration_number": configuration_number,
                            "command_i1": float(command_i1),
                            "command_i2": float(command_i2),
                            # "set_pos1": float(set_pos1),
                            # "set_pos2": float(set_pos2),
                            "set_vel1": float(set_vel1),
                            "set_vel2": float(set_vel2),
                            "i1": float(i1),
                            "i2": float(i2),
                            "pos1": float(pos1),
                            "pos2": float(pos2),
                            "vel1": float(vel1),
                            "vel2": float(vel2),
                            # "recovery": recovery,
                            # "ctrl_type": ctrl_type,
                        }]

                        # Find aurora reading match 
                        if time > start_time:
                            # Finds first valid aurora reading 
                            if to_add_start_idx == -1 and j >= self.feedback_horizon:
                                aurora_start_idx = aurora_curr_idx
                                to_add_start_idx = j-self.feedback_horizon

                            if to_add_start_idx != -1:
                                # Synchronizes with aurora data. Assumes rate motor >> rate aurora 
                                to_add_mappings += [len(self.motor_data) - 1 + len(to_add_data) - to_add_start_idx]
                                if len(to_add_data) + self.prediction_horizon > len(lines_motor):
                                    break
                                to_add_end_idx = len(to_add_data) + self.prediction_horizon
                                
                                
                            aurora_curr_idx += 1
                            if aurora_curr_idx == len(lines_aurora):
                                start_time = float('inf')
                            else:
                                start_time = float(lines_aurora[aurora_curr_idx].decode().split(",")[0])
                        
                    if len(to_add_mappings) < 2:
                        # Measurements are not long enough 
                        continue
                
                    # if aurora_curr_idx != len(lines_aurora):
                    #     # Case for when motor data runs out before hitting end of aurora 
                    #     to_add_mappings += [None]

                    self.aurora_data_map += to_add_mappings
                    self.motor_data += to_add_data[to_add_start_idx:to_add_end_idx]

                for j in range(aurora_start_idx, aurora_curr_idx): # -1 here fixes, not ideal 
                    if j > 0:
                        # Restrict dataset retrieval to ensure at least one prior aurora reading
                        self.indexable_aurora_data += [len(self.aurora_data)]

                    # Add data 
                    time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = lines_aurora[j].decode().split(",")
                    self.aurora_data += [{
                        "time": float(time),
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "x_raw": float(x_raw),
                        "y_raw": float(y_raw),
                        "z_raw": float(z_raw),
                        "q1": float(q1),
                        "q2": float(q2),
                        "q3": float(q3),
                        "q4": float(q4)
                    }]
                    

        print(f"Data loaded in {t.time() - START:.3}s")
        print("Total data points:", len(self))

        print(len(self.aurora_data), len(self.aurora_data_map))
        print(len(self.indexable_aurora_data))
        # print(len(self.motor_data), max(self.aurora_data_map))

    def __len__(self):
        return len(self.indexable_aurora_data)

    def __getitem__(self, _idx):
        '''
        Data: 
        - last known location
        - goal (terminal) location 
        - last N (feedback_horizon) motor feedback 

        Label: 
        - M motor commands 
        '''

        idx = self.indexable_aurora_data[_idx]
        # print(self.aurora_data_map[idx-1], len(self.motor_data))

        position_data = torch.tensor([
            self.aurora_data[idx-1]['x'], 
            self.aurora_data[idx-1]['y'],
            self.aurora_data[idx]['x'], 
            self.aurora_data[idx]['y'],
            self.motor_data[self.aurora_data_map[idx-1]]['configuration_number']
            ])
        
        feedback_data = torch.tensor([
            [
            self.motor_data[i]['i1'], 
            self.motor_data[i]['i2'],
            self.motor_data[i]['pos1'],
            self.motor_data[i]['pos2'],
            self.motor_data[i]['vel1'],
            self.motor_data[i]['vel2'],
            ]
            for i in range(self.aurora_data_map[idx-1] - self.feedback_horizon, self.aurora_data_map[idx-1])
        ])
        
        try:
            label = torch.tensor([
                [self.motor_data[i]['command_i1'], 
                self.motor_data[i]['command_i2']] 
                for i in range(self.aurora_data_map[idx-1], self.aurora_data_map[idx-1] + self.prediction_horizon)
            ])
        except:
            print(self.aurora_data_map[idx-1], self.aurora_data_map[idx-1] + self.prediction_horizon, len(self.motor_data))
            raise

        return (position_data, feedback_data), label
        

if __name__ == '__main__':
    dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', feedback_horizon=30)

    train_dataloader = DataLoader(dataset, batch_size=256, shuffle=True)

    print(torch.cuda.is_available())

    counter = 0 
    for (position_data, feedback_data), labels in train_dataloader:
        print("Position data shape:", position_data.shape)
        print("Feedback data shape:", feedback_data.shape)
        print("Labels shape:", labels.shape)
        break
        # counter += 1
        # print(counter, position_data.shape)