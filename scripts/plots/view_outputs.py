import argparse 
import sys 
import os
import json 

import torch
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt 
import numpy as np

from dataloaders.ik_dataset import PCRDataSet


# Grab model path 
parser = argparse.ArgumentParser()
parser.add_argument('model_path')
model_path = parser.parse_args().model_path


save_folder = os.path.join('logs', 'model_outputs')
if not os.path.exists(save_folder):
    os.makedirs(save_folder)


# Load in configuration 
sys.path.append('/' + os.path.join(*model_path.split('/')[:-3]))
from model import PCR_Learned_Model

params = json.load(open(os.path.join('/' + os.path.join(*model_path.split('/')[:-2]), 'parameters.yaml'), 'rb'))
training_params, model_params = params['TRAINING'], params['MODEL']

weights = torch.load(open(model_path, 'rb'))
model = PCR_Learned_Model(model_params['PREDICTION_HORIZON'], **model_params['MODEL_PARAMS'])
model.load_state_dict(weights)


# Load in data 
# dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', model_params['PREDICTION_HORIZON'], feedback_horizon=training_params['FEEDBACK_HORIZON'], device=train.parameters['DEVICE'])
dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', model_params['PREDICTION_HORIZON'], feedback_horizon=training_params['FEEDBACK_HORIZON'], device=params['DEVICE'])
train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
train_dataloader = DataLoader(train_dataset, batch_size=training_params['BATCH_SIZE'], shuffle=True)
val_dataloader = DataLoader(val_dataset, batch_size=training_params['BATCH_SIZE'], shuffle=True)


model.train(False)
for i, vdata in enumerate(val_dataloader):
    (vposition_data, vfeedback_data), vlabels = vdata
    vpred = model(vposition_data, vfeedback_data)
    print(vpred.shape, vlabels.shape)

    for j in range(10):
        x = list(range(model_params['PREDICTION_HORIZON']))
        ys = np.hstack([vlabels[j].detach().cpu().numpy(), vpred[j].detach().cpu().numpy()])

        plt.clf()
        plt.plot(x, ys)
        plt.legend(['i1', 'i2', 'i1_pred', 'i2_pred'])
        plt.savefig(os.path.join(save_folder, f'{j}.png'))
    break