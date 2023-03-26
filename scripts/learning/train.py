import time 
import os 
import shutil
import tqdm 

import torch 
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
torch.manual_seed(0)
import matplotlib.pyplot as plt

from data_loader import PCRDataSet
from model import PCR_Learned_Model


BATCH_SIZE = 256
PREDICTION_HORIZON = 20
FEEDBACK_HORIZON = 30
NUM_EPOCHS = 50

# Logging setup 
root = os.path.dirname(os.path.abspath(__file__))
save_directory = os.path.join(root, 'logs', str(time.time())) # Generates unique filename
if not os.path.exists(save_directory):
    os.makedirs(save_directory)
model_save_directory = os.path.join(save_directory, 'models')
if not os.path.exists(model_save_directory):
    os.makedirs(model_save_directory)
shutil.copyfile(os.path.join(root, 'train.py'), os.path.join(save_directory, 'train.py'))
shutil.copyfile(os.path.join(root, 'model.py'), os.path.join(save_directory, 'model.py'))

writer = SummaryWriter(os.path.join(save_directory, 'training_data'))

# Dataset loading 
# dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', PREDICTION_HORIZON, feedback_horizon=FEEDBACK_HORIZON)
dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', PREDICTION_HORIZON, feedback_horizon=FEEDBACK_HORIZON)
train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
train_dataloader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
val_dataloader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=True)


# Model loading 
model = PCR_Learned_Model(PREDICTION_HORIZON)

# Training objects 
loss_fn = torch.nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)


def train_one_epoch(epoch_index, tb_writer):
    running_loss = 0 

    for i, data in tqdm.tqdm(enumerate(train_dataloader)):
        (position_data, feedback_data), labels = data
        optimizer.zero_grad()

        pred = model(position_data, feedback_data)

        loss = loss_fn(pred, labels)
        loss.backward()
        optimizer.step()

        running_loss += loss.item()

        tb_x = epoch_index * len(train_dataloader) + i + 1
        tb_writer.add_scalar('Loss/train', loss.item(), tb_x)

    avg_loss = running_loss / len(train_dataloader)
    print(f"Epoch {epoch_index}: Train Loss: {avg_loss:>8.3}")

    return avg_loss


best_vloss = float('inf')
for epoch in range(NUM_EPOCHS):
    model.train(True)
    avg_loss = train_one_epoch(epoch, writer)

    model.train(False)
    running_vloss = 0 
    for i, vdata in enumerate(val_dataloader):
        (vposition_data, vfeedback_data), vlabels = vdata
        vpred = model(vposition_data, vfeedback_data)
        vloss = loss_fn(vpred, vlabels)
        running_vloss += vloss.item()
    avg_vloss = running_vloss / (i + 1) 

    writer.add_scalars('Training vs Validation Loss', {'Training': avg_loss, 'Validation':avg_vloss}, epoch + 1)
    writer.flush()

    if avg_vloss < best_vloss:
        best_vloss = avg_vloss
        model_path = os.path.join(model_save_directory, f'model_{epoch}')
        torch.save(model.state_dict(), model_path)


