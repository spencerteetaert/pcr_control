import time 
import os
import sys
import shutil
import tqdm 
import json

import torch 
from torch.utils.tensorboard import SummaryWriter
torch.manual_seed(0)

parameters = {
    'DEVICE': 'cuda' if torch.cuda.is_available() else 'cpu',
    'TRAINING': {
        'BATCH_SIZE': 512,
        'NUM_EPOCHS': 1000,
        'LEARNING_RATE': 0.001,
        'FEEDBACK_HORIZON': 20,
    },
    'MODEL': {
        'MODEL_PARAMS': {}, 
        'PREDICTION_HORIZON': 20,
    }
}

def main(model_constructor, train_dataloader, val_dataloader, trial_name='', trial_description=''):
    # Logging setup 
    root = os.path.dirname(os.path.abspath(__file__))
    if trial_name == '':
        save_directory = os.path.join(root, 'logs', str(time.time())) # Generates unique filename
    else:
        save_directory = os.path.join(root, 'logs', trial_name) # Generates unique filename
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)

    log_file = open(os.path.join(save_directory, 'description.txt'), 'a')
    sys.stdout = log_file
    json.dump(parameters, open(os.path.join(save_directory, 'parameters.yaml'), 'a'), indent=4)

    training_params = parameters['TRAINING']
    model_params = parameters['MODEL']

    print("Training start time:", str(time.time()))
    print("Training location:", save_directory)
    print(trial_description)
    # print("-----------------\nRun parameters")
    # print(json.dumps(parameters, indent=4))
    # print("-----------------\n")

    

    model_save_directory = os.path.join(save_directory, 'models')
    if not os.path.exists(model_save_directory):
        os.makedirs(model_save_directory)

    writer = SummaryWriter(os.path.join(save_directory, 'training_data'))

    # Training objects 
    loss_fn = torch.nn.MSELoss()
    model = model_constructor(model_params['PREDICTION_HORIZON'], **model_params['MODEL_PARAMS'])
    optimizer = torch.optim.Adam(model.parameters(), lr=training_params['LEARNING_RATE'])


    def train_one_epoch(epoch, tb_writer):
        running_loss = 0 

        for i, data in tqdm.tqdm(enumerate(train_dataloader)):
            (position_data, feedback_data), labels = data
            optimizer.zero_grad()

            pred = model(position_data, feedback_data)

            loss = loss_fn(pred, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()

            tb_x = epoch * len(train_dataloader) + i + 1
            tb_writer.add_scalar('Loss/train', loss.item(), tb_x)

        avg_loss = running_loss / len(train_dataloader)

        return avg_loss


    best_vloss = float('inf')
    early_stopping = 10
    for epoch in range(training_params['NUM_EPOCHS']):
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

        model_path = os.path.join(model_save_directory, f'last')
        torch.save(model.state_dict(), model_path)

        if avg_vloss < best_vloss:
            print(f"Epoch {epoch}: Train Loss: {avg_loss:>8.3} Val Loss: {avg_vloss:>8.3} - New best.")
            best_vloss = avg_vloss
            model_path = os.path.join(model_save_directory, f'best_val')
            torch.save(model.state_dict(), model_path)
            early_stopping = 10
        else:
            print(f"Epoch {epoch}: Train Loss: {avg_loss:>8.3} Val Loss: {avg_vloss:>8.3}")
            early_stopping -= 1

        if early_stopping == 0:
            print("Early stop.")
            break

    print("Training end time:", str(time.time()))
    print("Best validation loss:", str(best_vloss))

    return best_vloss


if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('trial_name', default='')
    args = parser.parse_args()

    # Dataset loading 
    from dataloaders.ik_dataset import PCRDataSet
    from torch.utils.data import DataLoader
    # dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/dev', parameters['PREDICTION_HORIZON'], feedback_horizon=parameters['FEEDBACK_HORIZON'], device=parameters['DEVICE'])
    dataset = PCRDataSet('/home/spencer/Documents/thesis/pcr_control/data/sorted_data', parameters['PREDICTION_HORIZON'], feedback_horizon=parameters['FEEDBACK_HORIZON'], device=parameters['DEVICE'])
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [0.8, 0.2])
    train_dataloader = DataLoader(train_dataset, batch_size=parameters['BATCH_SIZE'], shuffle=True)
    val_dataloader = DataLoader(val_dataset, batch_size=parameters['BATCH_SIZE'], shuffle=True)

    from models.joint_model import PCR_Learned_Model
    model = PCR_Learned_Model(parameters['PREDICTION_HORIZON'], **parameters['MODEL'])

    main(train_dataloader, val_dataloader, args.trial_name)