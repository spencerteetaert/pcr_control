import argparse
import yaml, sys, time
from math import sin, cos

import numpy as np
import cv2
from scipy.optimize import fsolve

class Link:
    def __init__(self, index, config, verbose=False):
        self.INDEX = index
        self.LENGTH = config['length']
        self.BASE_POINT = np.array(config['base_point'])
        self.EXCLUSION_RADIUS = config['exclusion_radius']

        self.MOTOR_RADIUS = config['motor_radius']
        self.TENDON_RADIUS = config['tendon_radius']

        self.GEAR_RATIO = config['gear_ratio']

        self.end_point = np.array([0, 0])
        self.verbose = verbose

        # physical properties 
        self.k = 1 # curvature 
        self.dq = [0, 0] # rad/s

    #region kinematics 
    def _inverse_kinematics(self, k):
        return 2*np.sin(self.LENGTH*k/2)/(self.LENGTH*k) - np.linalg.norm(self.end_point - self.BASE_POINT)/self.LENGTH

    def _solve_for_k(self):
        ret = fsolve(self._inverse_kinematics, self.k, full_output=True)
        if self.verbose:
            print(f"Link {self.INDEX} curvature solved to be {ret}")
        return ret[0]
    
    def solve_for_dq(self, dk):
        return dk * self.GEAR_RATIO * self.TENDON_RADIUS * self.LENGTH / self.MOTOR_RADIUS

    def _check_end_point(self, end_point):
        # Checks bounds of end_point to ensure solution exists
        ret = np.linalg.norm(self.BASE_POINT - end_point) < self.LENGTH and not np.linalg.norm(self.BASE_POINT - end_point) < self.EXCLUSION_RADIUS

        if ret: 
            return ret
        
        elif np.linalg.norm(self.BASE_POINT - end_point) >= self.LENGTH:
            print("End point longer than arms.", np.linalg.norm(self.BASE_POINT - end_point))
        elif np.linalg.norm(self.BASE_POINT - end_point) <= self.EXCLUSION_RADIUS:
            print("End point inside exclusion zone.", np.linalg.norm(self.BASE_POINT - end_point))

        print("Base position:", self.BASE_POINT)
        print("End effector position:", end_point)
        print("Length:", self.LENGTH)
        print("Exclusion:", self.EXCLUSION_RADIUS)

        return ret

    def update_end_point(self, end_point):
        if not self._check_end_point(end_point):
            return False
        self.end_point = np.array(end_point)
        self.k = self._solve_for_k()

    #endregion

class CC_Model:
    def __init__(self, config, Kp = 0.04, real_time=False):
        self.type = "CC_differential_model"
        self.real_time = real_time
        self.links = []
        for i in range(config['num_links']):
            self.links += [Link(i, config['links'][i])]

        self.k_dot = np.array([0, 0])
        self.k = np.array([0, 0])
        self.ee_pos = np.array([0, 0])
        self.goal_point = np.array([0, 0])
        self.u = []

        self.A = np.eye(len(self.links))
        self.Binv = np.eye(len(self.links)) 
        self.Kp = Kp

    def get_command(self):
        return self.u

    def update_goal_point(self, goal): 
        self.goal_point = goal 

    #region kinematics
    def _check_end_point(self, end_point):
        for link in self.links:
            if not link._check_end_point(end_point):
                return False
        return True

    def update_end_point(self, pos, timestamp=None):
        if not self._check_end_point(pos):
            return False
        for i, link in enumerate(self.links):
            link.update_end_point(pos)
            self.k[i] = link.k
        self.ee_pos = pos
        
        self.goal_point_vel = self.Kp * (self.goal_point - self.ee_pos)
        self._gen_A_B()
        self.k_dot = fsolve(self._inverse_kinematics, self.k_dot)

        u = []
        for i, link in enumerate(self.links):
            u += [link.solve_for_dq(self.k_dot[i])]
        self.u = u

        return True
    
    def _inverse_kinematics(self, k):
        return -self.Binv @ self.A @ k - self.goal_point_vel  
    
    def _gen_A_B(self):
        '''
        Updates control signal after receiving end effector position feedback 
        Arguments: 
        - pos: (2,) numpy array with robot end effector position (x,y) [m]

        TODO: Possible race condition here that should be fixed 
        ''' 
        a = []
        b = []
        for link in self.links:
            dhdk = cos(link.k * link.LENGTH / 2) / link.k - 2 * sin(link.k * link.LENGTH / 2) / (link.LENGTH * link.k**2)
            norm = np.linalg.norm(self.ee_pos - link.BASE_POINT)
            dhdx = (link.BASE_POINT[0] - self.ee_pos[0]) / (link.LENGTH * norm)
            dhdy = (link.BASE_POINT[1] - self.ee_pos[1]) / (link.LENGTH * norm)
            b += [[dhdx, dhdy]]
            a += [dhdk.item()]

        self.A = np.diag(a)
        self.Binv = np.linalg.inv(np.array(b))

    #endregion

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config1.yaml")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config, 'r'))

    controller = CC_Model(config['controller_params'])

    controller.update_goal_point(np.array([0.3, 0.25]))
    controller.update_end_point(np.array([0.25, 0.25]))

    print(controller.get_command())