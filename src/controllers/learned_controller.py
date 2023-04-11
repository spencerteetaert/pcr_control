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
        self.last_time = time.time()

        # Load in configuration 
        sys.path.append('/' + os.path.join(*model_path.split('/')[:-3]))
        from model import PCR_Learned_Model

        params = json.load(open(os.path.join('/' + os.path.join(*model_path.split('/')[:-2]), 'parameters.yaml'), 'rb'))
        training_params, model_params = params['TRAINING'], params['MODEL']
        self.FEEDBACK_HORIZON = training_params['FEEDBACK_HORIZON']
        self.motor_feedback = [[0, 0, 0, 0, 0, 0] for _ in range(self.FEEDBACK_HORIZON)]

        weights = torch.load(open(model_path, 'rb'), map_location=torch.device('cpu'))
        self.model = PCR_Learned_Model(model_params['PREDICTION_HORIZON'], **model_params['MODEL_PARAMS'])
        self.model.load_state_dict(weights)
        self.model.train(False)
        print("Learned model loaded.")

    def update_feedback(self, mtr_data):
        self.motor_feedback.pop(0)
        self.motor_feedback += [[
            mtr_data.mtr1.current.value,
            mtr_data.mtr2.current.value,
            mtr_data.mtr1.position.value,
            mtr_data.mtr2.position.value,
            mtr_data.mtr1.velocity.value,
            mtr_data.mtr2.velocity.value,
        ]]

    def get_command(self):
        if len(self.u) > 0:
            return self.u[0]
        else:
            return [0, 0]

    def _get_command(self):
        if len(self.u) > 0:
            t = time.time()
            if t - self.last_time > 1/100:
                # Model is trained on 100hz data, in reality system runs closer to 1000hz 
                self.last_time = t
                return self.u.pop(0)
            else: 
                return self.u[0]
        else:
            return [0, 0]

    def update_goal_point(self, goal): 
        self.goal_point = goal 
        # Configuration = -1 assumes robot has not changed since the last training implementation. 
        # Ideally this parameter is retuned for each trial
        position_data = torch.Tensor([[self.ee_pos[0], self.ee_pos[1], self.goal_point[0], self.goal_point[1], -1]])
        feedback_data = torch.Tensor([self.motor_feedback])

        output = self.model(position_data, feedback_data)
        self.u = self.u[:3] + output.tolist()[0] # Keep 3 existing to ensure continuity 

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