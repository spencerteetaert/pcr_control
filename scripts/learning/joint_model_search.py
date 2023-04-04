import os
import shutil 

import torch 
from torch.utils.data import DataLoader

import train
from dataloaders.ik_dataset import PCRDataSet
import models.joint_model as model_file 

trials = [
    {'linear_layers': [[10]], 'lstm': 15}, 
    {'linear_layers': [[20]], 'lstm': 15}, 
    {'linear_layers': [[30]], 'lstm': 15}, 

    {'linear_layers': [[10, 10]], 'lstm': 15}, 
    {'linear_layers': [[20, 20]], 'lstm': 15}, 
    {'linear_layers': [[30, 30]], 'lstm': 15}, 

    {'linear_layers': [[10, 10, 10]], 'lstm': 15}, 
    {'linear_layers': [[20, 20, 20]], 'lstm': 15}, 
    {'linear_layers': [[30, 30, 30]], 'lstm': 15}, 
]

# Dataset loading 
# dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', train.parameters['MODEL']['PREDICTION_HORIZON'], feedback_horizon=train.parameters['TRAINING']['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', train.parameters['MODEL']['PREDICTION_HORIZON'], feedback_horizon=train.parameters['TRAINING']['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
train_dataloader = DataLoader(train_dataset, batch_size=train.parameters['TRAINING']['BATCH_SIZE'], shuffle=True)
val_dataloader = DataLoader(val_dataset, batch_size=train.parameters['TRAINING']['BATCH_SIZE'], shuffle=True)

experiment_folder = 'joint_model_search2'
if not os.path.exists(os.path.join('logs', experiment_folder)):
    os.makedirs(os.path.join('logs', experiment_folder))
experiment_log_file = open(os.path.join('logs', experiment_folder, 'log.txt'), 'a')
shutil.copyfile(__file__, os.path.join('logs', experiment_folder, 'trial_script.py'))
shutil.copyfile(train.__file__, os.path.join('logs', experiment_folder, 'train.py'))
shutil.copyfile(model_file.__file__, os.path.join('logs', experiment_folder, 'model.py'))

for trial in trials: 
    _trial_name = f'{experiment_folder}/l_'
    for ll in trial['linear_layers']:
        _trial_name += f'{ll}_'
    _trial_name += f'lstm_{trial["lstm"]}'

    train.parameters['MODEL']['MODEL_PARAMS'] = trial
    val_loss = train.main(model_file.PCR_Learned_Model, train_dataloader, val_dataloader, _trial_name)

    experiment_log_file.write(f'val_loss: {val_loss}, params: {str(trial)}\n')
