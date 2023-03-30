import torch
import torch.nn as nn
import torch.nn.functional as F

default_device = 'cuda' if torch.cuda.is_available() else 'cpu'

# PyTorch models inherit from torch.nn.Module
class PCR_Learned_Model(nn.Module):
    def __init__(self, prediction_horizon, linear_layers = [[10, 10], [100]], lstm = 30, device=default_device):
        '''
        Arguments 
            prediction_horizon (int): number of motor current readings in the future this model is meant to predict 
            linear_layers (list): number of nodes in each hidden layer.
        '''
        super(PCR_Learned_Model, self).__init__()

        self.device = device

        self.prediction_horizon = prediction_horizon 

        # Construct fc body
        self.fc1 = [nn.Linear(5, linear_layers[0][0]), nn.ReLU()]
        for i in range(1, len(linear_layers[0])):
            self.fc1 += [nn.Linear(linear_layers[0][i-1], linear_layers[0][i]), nn.ReLU()]
        self.fc1 = nn.Sequential(*self.fc1)

        # Construct fc head
        if len(linear_layers) > 1:
            self.fc2 = [nn.Linear(linear_layers[0][-1] + lstm, linear_layers[1][0]), nn.ReLU()]
            for i in range(1, len(linear_layers[1])):
                self.fc2 += [nn.Linear(linear_layers[1][i-1], linear_layers[1][i]), nn.ReLU()]
            self.fc2 += [nn.Linear(linear_layers[1][-1], self.prediction_horizon*2)]
        else:
            self.fc2 = [nn.Linear(linear_layers[0][-1] + lstm, self.prediction_horizon*2)]
        self.fc2 = nn.Sequential(*self.fc2)

        # Construct state estimator 
        self.lstm = None
        if lstm != 0: 
            self.lstm = nn.LSTM(input_size=6, hidden_size=lstm, batch_first=True)

        self.to(self.device)

    def forward(self, position_data, feedback_data):
        # Position / goal information 
        x = self.fc1(position_data)

        # State estimation 
        if self.lstm is not None:
            x2, (h_n, c_n) = self.lstm(feedback_data)
            x2 = x2[:,-1]

            x = torch.cat([x, x2], 1)

        x = torch.reshape(self.fc2(x), (-1, self.prediction_horizon, 2))
        
        return x


if __name__=='__main__':
    model = PCR_Learned_Model()
