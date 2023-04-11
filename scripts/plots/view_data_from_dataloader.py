import time 
import os 
import sys 
import shutil
import tqdm 

import torch 
from torch.utils.data import DataLoader
torch.manual_seed(0)
import matplotlib.pyplot as plt
import numpy as np
import cv2

sys.path.append('..')
from learning.dataloaders.ik_dataset import PCRDataSet

PREDICTION_HORIZON = 20
FEEDBACK_HORIZON = 20

# Dataset loading 
dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', PREDICTION_HORIZON, feedback_horizon=FEEDBACK_HORIZON, device='cpu')
# dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', PREDICTION_HORIZON, feedback_horizon=FEEDBACK_HORIZON)

start_idx = 0
fig, axs = plt.subplots(1, 2)
for i, vdata in enumerate(dataset):
    
    (vposition_data, vfeedback_data), vlabels = vdata

    sx, sy, ex, ey, conf = vposition_data
    
    axs[0].scatter(sx.numpy(), sy.numpy(), c='b')
    axs[0].scatter(ex.numpy(), ey.numpy(), c='r')

    if FEEDBACK_HORIZON > 0:
        v1 = vfeedback_data[:,4]
        v2 = vfeedback_data[:,5]
        feedback_t = list(range(start_idx, start_idx+len(v1)))
        axs[1].plot(feedback_t, v1, c='r')
        axs[1].plot(feedback_t, v2, c='b')
        start_idx += len(v1)

    fig.canvas.draw()

    # Now we can save it to a numpy array.
    data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    cv2.imshow('plot', data)
    key = cv2.waitKey(0)

    if key == ord('q'):
        break