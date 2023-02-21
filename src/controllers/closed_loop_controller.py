import argparse
import yaml, sys

import cv2 
import numpy as np

from blmc.pid import PID

if __name__!='__main__':
    from .PCR_controller import PCRController
else:
    sys.path.append('src/')
    from controllers.PCR_controller import PCRController

class Closed_Loop_Controller(PCRController):
    def __init__(self, config, Kp=0.005, Kd=0.00002, Ki=0):
        PCRController.__init__(self)

        self.ee_pos = np.array([0, 0])
        self.base_pos = []
        self._pid = []
        self.error = []
        self.dt = [] 
        self.u = []
        
        for i in range(len(config['links'])):
            self.base_pos += [np.array(config['links'][i]['base_point'])]
            self._pid += [PID()]
            self._pid[-1].SetKp(Kp)
            self._pid[-1].SetKi(Ki)
            self._pid[-1].SetKd(Kd)

    def step(self, pos, goal, timestamp):
        '''
        Updates control signal after receiving end effector position feedback 
        Arguments: 
        - pos: aurora position object 
        - goal: aurora goal position 
        ''' 
        self.ee_pos = pos 
        u = []
        for i in range(len(self.base_pos)):
            error = np.linalg.norm(goal - self.base_pos[i]) - np.linalg.norm(self.ee_pos - self.base_pos[i])
            u += [self._pid[i].GenOut(error, timestamp)]
        self.u = u

    def get_command(self):
        return self.u


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config1.yaml")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config, 'r'))

    controller = Closed_Loop_Controller(config['controller_params'])

    goal = np.array([0.3, 0.3])
    curr_loc = np.array([0., 0.])

    for i in range(500):
        # Motor control loop for 5s 
        time = i / 100
        if i % 20 == 0: 
            # Update aurora at 5 hz 
            controller.step(curr_loc, goal, time)

        u = controller.get_command()

        print(u)

        delta1 = (curr_loc - controller.base_pos[0]) / np.linalg.norm(curr_loc - controller.base_pos[0]) * u[0]
        delta2 = (curr_loc - controller.base_pos[1]) / np.linalg.norm(curr_loc - controller.base_pos[1]) * u[1]
        curr_loc += delta1 + delta2 + np.random.rand(2)*0.001

        disp = np.ones([1000, 1000, 3])*255

        # Coordinate displays
        cv2.circle(disp, (goal*1000).astype(int), 3, (0, 255, 0), -1)
        cv2.circle(disp, (curr_loc*1000).astype(int), 3, (255, 0, 0), -1)

        cv2.arrowedLine(disp, (curr_loc*1000).astype(int), ((curr_loc + delta1*10)*1000).astype(int), (0, 0, 255), 3)
        cv2.arrowedLine(disp, (curr_loc*1000).astype(int), ((curr_loc + delta2*10)*1000).astype(int), (0, 0, 255), 3)

        # Window displays 
        # disp = disp[::-1,...]
        cv2.putText(disp, f"Time: {time:.3}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))

        cv2.imshow("Closed loop sim", disp)
        cv2.waitKey(1)

        

