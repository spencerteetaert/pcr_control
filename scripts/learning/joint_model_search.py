import os

import torch 
from torch.utils.data import DataLoader

import train
from dataloaders.ik_dataset import PCRDataSet

trials = [
    # LSTM Tuning
    {'linear_layers': [[10, 10], [50]], 'lstm': 0}, 
    {'linear_layers': [[10, 10], [50]], 'lstm': 20}, 
    {'linear_layers': [[10, 10], [50]], 'lstm': 40}, 
    {'linear_layers': [[10, 10], [50]], 'lstm': 60}, 
    {'linear_layers': [[10, 10], [50]], 'lstm': 80}, 
    {'linear_layers': [[10, 10], [50]], 'lstm': 100}, 

    # Head Tuning
    {'linear_layers': [[10, 10]], 'lstm': 20}, 
    {'linear_layers': [[10, 10], [25]], 'lstm': 20}, 
    {'linear_layers': [[10, 10], [50]], 'lstm': 20}, 

    {'linear_layers': [[10, 10], [25, 25]], 'lstm': 20}, 
    {'linear_layers': [[10, 10], [25, 25, 25]], 'lstm': 20}, 
    {'linear_layers': [[10, 10], [25, 25, 25, 25]], 'lstm': 20}, 

    # Body Tuning
    {'linear_layers': [[10], [25, 25]], 'lstm': 20}, 
    {'linear_layers': [[20], [25, 25]], 'lstm': 20}, 
    {'linear_layers': [[30], [25, 25]], 'lstm': 20}, 

    {'linear_layers': [[5], [25, 25]], 'lstm': 20}, 
    {'linear_layers': [[5, 5], [25, 25]], 'lstm': 20}, 
    {'linear_layers': [[5, 5, 5], [25, 25]], 'lstm': 20}, 
]

# Dataset loading 
from dataloaders.ik_dataset import PCRDataSet
# dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', train.parameters['PREDICTION_HORIZON'], feedback_horizon=train.parameters['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', train.parameters['PREDICTION_HORIZON'], feedback_horizon=train.parameters['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
train_dataloader = DataLoader(train_dataset, batch_size=train.parameters['BATCH_SIZE'], shuffle=True)
val_dataloader = DataLoader(val_dataset, batch_size=train.parameters['BATCH_SIZE'], shuffle=True)

experiment_folder = 'joint_model_search'
if not os.path.exists(os.path.join('logs', experiment_folder)):
    os.makedirs(os.path.join('logs', experiment_folder))
experiment_log_file = open(os.path.join(os.path.join('logs', experiment_folder), 'log.txt'), 'a')

for trial in trials: 
    _trial_name = f'{experiment_folder}/l_'
    for ll in trial['linear_layers']:
        _trial_name += f'{ll}_'
    _trial_name += f'lstm_{trial["lstm"]}'

    train.parameters['MODEL'] = trial
    val_loss = train.main(train_dataloader, val_dataloader, _trial_name)

    experiment_log_file.write(f'val_loss: {val_loss}, params: {str(trial)}\n')
