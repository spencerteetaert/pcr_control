import os
import shutil 

import torch 
from torch.utils.data import DataLoader

import train
from dataloaders.ik_dataset import PCRDataSet
import models.joint_model as model_file 

# Model parameters from joint_model_search experiment
model_params = {'linear_layers': [[10]], 'lstm': 15}
train.parameters['MODEL'] = model_params

trials = [1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 150, 200, 300]

from dataloaders.ik_dataset import PCRDataSet

experiment_folder = 'feedback_horizon_search2'
if not os.path.exists(os.path.join('logs', experiment_folder)):
    os.makedirs(os.path.join('logs', experiment_folder))
experiment_log_file = open(os.path.join(os.path.join('logs', experiment_folder), 'log.txt'), 'a')
shutil.copyfile(__file__, os.path.join(os.path.join('logs', experiment_folder), 'trial_script.py'))
shutil.copyfile(model_file.__file__, os.path.join('logs', experiment_folder, 'model.py'))

for trial in trials: 
    _trial_name = f'{experiment_folder}/horizon_{trial}'
    train.parameters['FEEDBACK_HORIZON'] = trial
    model = model_file.PCR_Learned_Model(train.parameters['PREDICTION_HORIZON'], device=train.parameters['DEVICE'], **train.parameters['MODEL'])
    
    dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', train.parameters['PREDICTION_HORIZON'], feedback_horizon=train.parameters['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
    train_dataloader = DataLoader(model, train_dataset, batch_size=train.parameters['BATCH_SIZE'], shuffle=True)
    val_dataloader = DataLoader(val_dataset, batch_size=train.parameters['BATCH_SIZE'], shuffle=True)

    val_loss = train.main(train_dataloader, val_dataloader, _trial_name)

    experiment_log_file.write(f'val_loss: {val_loss}, feedback_horizon: {str(trial)}\n')
