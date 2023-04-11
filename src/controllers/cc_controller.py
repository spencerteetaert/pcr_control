import argparse
import yaml, sys

import numpy as np
import cv2
from scipy.optimize import fsolve

if __name__!='__main__':
    from ..math_helper import convert_arc
    from ..trajectory_generation.trajectory_planner import TrajectoryPlanner
else:
    sys.path.append('src/')  
    from math_helper import convert_arc
    from trajectory_generation.trajectory_planner import TrajectoryPlanner

class Link:
    def __init__(self, index, config, verbose=False):
        self.INDEX = index
        self.LENGTH = config['length']
        self.BASE_POINT = np.array(config['base_point'])
        self.EXCLUSION_RADIUS = config['exclusion_radius']

        self.MOTOR_RADIUS = config['motor_radius']
        self.TENDON_RADIUS = config['tendon_radius']

        self.GEAR_RATIO = config['gear_ratio']
        self.CONVERSION_CONSTANTS = -self.GEAR_RATIO * self.TENDON_RADIUS * self.LENGTH / self.MOTOR_RADIUS

        self.end_point = np.array([0, 0])
        self.verbose = verbose

        # physical properties 
        self.k = 1 # curvature 
        self.dq = 0 # rad/s
        self.q = 0 # rad

    #region kinematics 
    def _inverse_kinematics(self, k):
        return 2*np.sin(self.LENGTH*k/2)/(self.LENGTH*k) + self.OPTIMIZER_CONSTANT

    def _solve_for_k(self):
        self.OPTIMIZER_CONSTANT = -np.linalg.norm(self.end_point - self.BASE_POINT)/self.LENGTH
        ret = fsolve(self._inverse_kinematics, self.k, full_output=True)
        if self.verbose:
            print(f"Link {self.INDEX} curvature solved to be {ret}")
        self.k = ret[0]
    
    def _solve_for_q(self):
        q = self.k * self.CONVERSION_CONSTANTS
        self.dq = q - self.q
        self.q = q

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
        self._solve_for_k()
        self._solve_for_q()

    #endregion

    #region draw 
    def draw(self, img, offset, scale):
        params = self._get_draw_arc_params(offset, scale)
        cv2.ellipse(img, *params)
        params[5] = 255
        params[6] = 2
        cv2.ellipse(img, *params)

        # cv2.circle(img, params[0], 3, (0, 0, 0), -1)
        # cv2.line(img, (params[0][0] - 100, params[0][1]), (params[0][0] + 100, params[0][1]), (0, 255, 0), 3)
        # cv2.line(img, (params[0][0], params[0][1] - 100), (params[0][0], params[0][1] + 100), (255, 0, 0), 3)
        cv2.circle(img, *self._get_draw_base_params(offset, scale, 5, (255, 255, 255)))
        cv2.circle(img, *self._get_draw_base_params(offset, scale, 3, (0, 0, 0)))
    def _get_draw_arc_params(self, offset_point, scale):
        centerCoordinates, radius, startAngle, endAngle = convert_arc(scale*self.BASE_POINT + offset_point, scale*self.end_point + offset_point, self.LENGTH*scale, scale/self.k)
        angle = 0
        color = 0
        thickness = 3
        return [centerCoordinates, radius, angle, startAngle, endAngle, color, thickness]
    def _get_draw_base_params(self, offset_point, scale, radius, color):
        centerCoordinates = scale*(self.BASE_POINT + offset_point)
        centerCoordinates = (int(centerCoordinates[0]), int(centerCoordinates[1]))
        return [centerCoordinates, radius, color, -1]
    #endregion 

class CC_Controller:
    def __init__(self, config, real_time=False):
        self.real_links = []
        self.sim_links = []
        for i in range(config['controller_params']['num_links']):
            self.real_links += [Link(i, config['controller_params']['links'][i])]
            self.sim_links += [Link(i, config['controller_params']['links'][i])]

        self.goal_point = None
        self.end_point = None
        self.u = [[0, 0]]
        self.real_time = real_time

        self.costmap_area = np.array([0.5, 0.5])
        self.scale = 100 # px/m 
        self.costmap_offset_m = np.array([0,0]) # self.costmap_area / 2

        size = self.costmap_area*self.scale
        self.costmap = self._draw_area(size.astype(int), self.costmap_offset_m, self.scale)

        self.planner = TrajectoryPlanner(self, config['trajectory_params'], C_smoothness=4)

    #region kinematics
    def _check_end_point(self, end_point):
        for link in self.real_links:
            if not link._check_end_point(end_point):
                return False
        return True
    
    def _update_end_point(self, end_point):
        # Used in trajectory generation
        if not self._check_end_point(end_point):
            return False
        for link in self.sim_links:
            link.update_end_point(end_point)
        return True

    def update_end_point(self, end_point, tracking=False, timestamp=None):
        # Runs in real time when Aurora reads 
        self.end_point = end_point
        for i in range(len(self.real_links)):
            self.real_links[i].update_end_point(end_point)
            self.sim_links[i].update_end_point(end_point)
            self.sim_links[i].dq = 0

        if len(self.u) > 1:
            self.u.pop(0)
        return True

    def update_goal_point(self, goal): 
        self.goal_point = goal 
        self.u += self.planner.gen_trajectory(self.goal_point, dt=0.2, add_noise=False, verbose=False)

    def get_command(self):
        return self.u[0]

    # def update_qs(self, dqs):
    #     ks = []
    #     mags = []

    #     for i in range(len(self.real_links)):
    #         self.real_links[i].q = [self.real_links[i].q[0] + dqs[i], self.real_links[i].q[1] - dqs[i]] #TODO: Fix 
    #         ks += [self.real_links[i].q[0] * self.real_links[i].MOTOR_RADIUS / (self.real_links[i].GEAR_RATIO*self.real_links[i].LENGTH*self.real_links[i].TENDON_RADIUS)]
    #         mags += [2/ks[-1] * np.sin(self.real_links[i].LENGTH*ks[-1]/2)]

    #     d = np.linalg.norm(self.real_links[0].BASE_POINT - self.real_links[1].BASE_POINT)
    #     a = (self.real_links[0].LENGTH**2 - self.real_links[1].LENGTH**2) / (2*d)
    #     h = (self.real_links[0].LENGTH**2 - a**2)**0.5
    #     x5 = self.real_links[0].BASE_POINT[0] + a*(self.real_links[0].BASE_POINT[0] - self.real_links[1].BASE_POINT[0])/d
    #     y5 = self.real_links[0].BASE_POINT[1] + a*(self.real_links[0].BASE_POINT[1] - self.real_links[1].BASE_POINT[1])/d

    #     x = x5 + h*(self.real_links[1].BASE_POINT[1] - self.real_links[0].BASE_POINT[1]) / d
    #     y = y5 + h*(self.real_links[1].BASE_POINT[0] - self.real_links[0].BASE_POINT[0]) / d

    #     self.update_end_point([x, y])

    #endregion
        
    #region drawing
    def _draw_area(self, img_size, offset, scale):
        img = np.ones(img_size, dtype=np.uint8)*255
        g_mask = np.ones(img_size[:2], dtype=np.uint8)
        
        for link in self.real_links:
            mask = np.zeros(img_size[:2], dtype=np.uint8)
            center = scale*(link.BASE_POINT - offset)
            cv2.circle(mask, (int(center[0]), int(center[1])), int(scale*link.LENGTH), (255, 255, 255), -1)
            cv2.circle(mask, (int(center[0]), int(center[1])), int(scale*link.EXCLUSION_RADIUS), (0, 0, 0), -1)
            g_mask = cv2.bitwise_and(g_mask, mask)

        img = cv2.bitwise_and(img, img, mask=g_mask)
        return img

    def draw(self):
        img = self.costmap.copy()

        for link in self.real_links:
            link.draw(img, self.costmap_offset_m, self.scale)
            # break
        return img
    #endregion 

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config1.yaml")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config, 'r'))

    controller = CC_Controller(config['controller_params'])