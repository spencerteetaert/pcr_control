import argparse
import os
import json 
import yaml, sys, time
from math import sin, cos

import numpy as np
import torch 


class Learned_Controller:
    def __init__(self, model_path, real_time=False):
        self.real_time = real_time

        self.ee_pos = np.array([0, 0])
        self.goal_point = np.array([0, 0])
        self.u = []
        self.motor_feedback = []

        # Load in configuration 
        sys.path.append('/' + os.path.join(*model_path.split('/')[:-3]))
        from model import PCR_Learned_Model

        params = json.load(open(os.path.join('/' + os.path.join(*model_path.split('/')[:-2]), 'parameters.yaml'), 'rb'))
        training_params, model_params = params['TRAINING'], params['MODEL']

        weights = torch.load(open(model_path, 'rb'))
        self.model = PCR_Learned_Model(model_params['PREDICTION_HORIZON'], **model_params['MODEL_PARAMS'])
        self.model.load_state_dict(weights)
        self.model.train(False)

    def get_command(self):
        return self.u.pop(0)

    def update_goal_point(self, goal): 
        self.goal_point = goal 
        # Configuration = -1 assumes robot has not changed since the last training implementation. Ideally this parameter is retuned for each trial
        position_data = torch.Tensor([self.ee_pos[0], self.ee_pos[1], self.goal_point[0], self.goal_point[1], -1])
        feedback_data = torch.Tensor(self.motor_feedback)
        self.motor_feedback = []
        
        output = self.model(position_data, feedback_data)

        self.u += output.cpu().tolist()

    def update_end_point(self, pos, tracking=False, timestamp=None):
        self.ee_pos = pos

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('model_path')
    model_path = parser.parse_args().model_path

    controller = Learned_Controller(model_path)

    controller.update_goal_point(np.array([0.3, 0.25]))
    controller.update_end_point(np.array([0.25, 0.25]))

    print(controller.get_command())