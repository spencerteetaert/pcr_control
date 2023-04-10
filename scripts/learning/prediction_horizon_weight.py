import os
import shutil 

import torch 
from torch.utils.data import DataLoader

import train
from dataloaders.ik_dataset import PCRDataSet
import models.joint_model_with_config as model_file 

models = [
    # LSTM Tuning
    {'linear_layers': [[10], [20, 20]], 'lstm': 15, 'configuration_state': 2}, 
    {'linear_layers': [[10], [20, 20]], 'lstm': 15, 'configuration_state': 4}, 
    {'linear_layers': [[10], [20, 20]], 'lstm': 30, 'configuration_state': 2}, 
    {'linear_layers': [[10], [20, 20]], 'lstm': 30, 'configuration_state': 4}, 
    {'linear_layers': [[10], [20, 20]], 'lstm': 45, 'configuration_state': 2}, 
    {'linear_layers': [[10], [20, 20]], 'lstm': 45, 'configuration_state': 4}, 
]
trials = {
    "baseline": torch.ones(train.parameters['MODEL']['PREDICTION_HORIZON']), # Unweighted
    "linear": torch.linspace(1, 3, train.parameters['MODEL']['PREDICTION_HORIZON']), # Linear 
    "exp": torch.exp(torch.linspace(0,2,train.parameters['MODEL']['PREDICTION_HORIZON'])), # Exp 
    "quad": torch.pow(torch.linspace(0,2,train.parameters['MODEL']['PREDICTION_HORIZON']), 2) - 1.5*torch.linspace(0,2,train.parameters['MODEL']['PREDICTION_HORIZON']) + 1.5, # Quadratic (bathtub) 
}

for trial in trials.keys(): # Normalize 
    trials[trial] /= torch.sum(trials[trial])

train.parameters['TRAINING']['FEEDBACK_HORIZON'] = 200

# Dataset loading 
# dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', train.parameters['MODEL']['PREDICTION_HORIZON'], feedback_horizon=train.parameters['TRAINING']['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', train.parameters['MODEL']['PREDICTION_HORIZON'], feedback_horizon=train.parameters['TRAINING']['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
train_dataloader = DataLoader(train_dataset, batch_size=train.parameters['TRAINING']['BATCH_SIZE'], shuffle=True)
val_dataloader = DataLoader(val_dataset, batch_size=train.parameters['TRAINING']['BATCH_SIZE'], shuffle=True)

experiment_folder = 'prediction_weighting2'
if not os.path.exists(os.path.join('logs', experiment_folder)):
    os.makedirs(os.path.join('logs', experiment_folder))
experiment_log_file = open(os.path.join('logs', experiment_folder, 'log.txt'), 'a')
shutil.copyfile(__file__, os.path.join('logs', experiment_folder, 'trial_script.py'))
shutil.copyfile(train.__file__, os.path.join('logs', experiment_folder, 'train.py'))
shutil.copyfile(model_file.__file__, os.path.join('logs', experiment_folder, 'model.py'))

for j, model in enumerate(models):
    train.parameters['TRAINING']['MODEL_PARAMS'] = model

    for i, trial in enumerate(trials.keys()): 
        if i == 0 and j == 0: 
            continue
        _trial_name = f'{experiment_folder}/t{trial}_m{j}'

        val_loss = train.main(model_file.PCR_Learned_Model, train_dataloader, val_dataloader, trials[trial], trial_name=_trial_name)

        experiment_log_file.write(f'val_loss: {val_loss}, params: {str(trial)}, model: {str(j)}\n')