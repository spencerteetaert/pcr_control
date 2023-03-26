import torch
import torch.nn as nn
import torch.nn.functional as F

# PyTorch models inherit from torch.nn.Module
class PCR_Learned_Model(nn.Module):
    def __init__(self, prediction_horizon):
        super(PCR_Learned_Model, self).__init__()

        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'

        self.prediction_horizon = prediction_horizon 

        self.fc1 = nn.Linear(5, 45)
        self.fc2 = nn.Linear(45, 45)

        self.fc3 = nn.Linear(45 + 30, 100)
        self.fc4 = nn.Linear(100, self.prediction_horizon*2)

        self.lstm1 = nn.LSTM(input_size=6, hidden_size=30, num_layers=self.prediction_horizon, batch_first=True)

        self.to(self.device)

    def forward(self, position_data, feedback_data):
        x1 = F.relu(self.fc1(position_data))
        x1 = F.relu(self.fc2(x1))

        x2, (h_n, c_n) = self.lstm1(feedback_data)
        x2 = x2[:,-1]

        x3 = F.relu(self.fc3(torch.cat([x1, x2], 1)))
        x3 = self.fc4(x3)
        x3 = torch.reshape(x3, (-1, self.prediction_horizon, 2))

        return x3


if __name__=='__main__':
    model = PCR_Learned_Model()
