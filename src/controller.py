import argparse
import yaml

import numpy as np
import cv2
from scipy.optimize import fsolve

if __name__!='__main__':
    from .math_helper import convert_arc
    from .trajectory_planner import TrajectoryPlanner
else:
    from math_helper import convert_arc
    from trajectory_planner import TrajectoryPlanner

class Link:
    def __init__(self, index, length, base_point, exclusion_radius, verbose=False):
        self.INDEX = index
        self.LENGTH = length
        self.BASE_POINT = np.array(base_point)
        self.EXCLUSION_RADIUS = exclusion_radius

        self.MOTOR_RADIUS = 0.0075
        self.TENDON_RADIUS = 0.01

        self.end_point = np.array([0, 0])
        self.verbose = verbose

        # physical properties 
        self.k = 1 # curvature 
        self.dq = [0, 0] # rad/s
        self.q = [0, 0] # rad

    #region kinematics 
    def _inverse_kinematics(self, k):
        return 2*np.sin(self.LENGTH*k/2)/(self.LENGTH*k) - np.linalg.norm(self.end_point - self.BASE_POINT)/self.LENGTH

    def _solve_for_k(self):
        ret = fsolve(self._inverse_kinematics, self.k, full_output=True)
        if self.verbose:
            print(f"Link {self.INDEX} curvature solved to be {ret}")
        return ret[0]
    def _solve_for_q(self):
        q = [self.k * self.TENDON_RADIUS * self.LENGTH / self.MOTOR_RADIUS, -self.k * self.TENDON_RADIUS * self.LENGTH / self.MOTOR_RADIUS]
        '''
        K = (q  * r_motor) / (r_tendon * l_arc)
        q = K * r_tendon * l_arc / r_motor  
        '''
        self.dq = [q[0] - self.q[0], q[1] - self.q[1] ]
        return q

    def _check_end_point(self, end_point):
        # Checks bounds of end_point to ensure solution exists
        return np.linalg.norm(self.BASE_POINT - end_point) < self.LENGTH and not np.linalg.norm(self.BASE_POINT - end_point) < self.EXCLUSION_RADIUS

    def update_end_point(self, end_point):
        if not self._check_end_point(end_point):
            return False
        self.end_point = np.array(end_point)
        self.k = self._solve_for_k()
        self.q = self._solve_for_q()

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
        centerCoordinates = scale*self.BASE_POINT + offset_point
        centerCoordinates = (int(centerCoordinates[0]), int(centerCoordinates[1]))
        return [centerCoordinates, radius, color, -1]
    #endregion 

class PCRController:
    def __init__(self, num_links, link_lengths, link_base_points, link_exclusion_radii):
        self.links = []
        for i in range(num_links):
            self.links += [Link(i, link_lengths[i], link_base_points[i], link_exclusion_radii[i])]
        self.scale = 100 # px/m 
        self.costmap = self._draw_area((self.scale, self.scale), (self.scale/2, self.scale/2), self.scale)
        self.trajectory_planner = TrajectoryPlanner()

        self.trajectory_planner.gen_trajectory(self, self.links[0].end_point, (0.1, -0.1), verbose=True)

    #region kinematics
    def _check_end_point(self, end_point):
        for link in self.links:
            if not link._check_end_point(end_point):
                return False
        return True

    def update_end_point(self, end_point):
        if not self._check_end_point(end_point):
            return False
        for link in self.links:
            link.update_end_point(end_point)
        return True
    #endregion

    #region planning
    # def _solve_path(self, goal, noisy=False): 
    #     return astar(self.costmap, (int((self.links[0].end_point[0]*40)+50), int((self.links[0].end_point[1]*40)+50)), goal)
    


    #endregion
        
    #region drawing
    def _draw_area(self, img_size, offset, scale):
        img = np.ones(img_size, dtype=np.uint8)*255
        g_mask = np.ones(img_size[:2], dtype=np.uint8)
        
        for link in self.links:
            mask = np.zeros(img_size[:2], dtype=np.uint8)
            center = scale*link.BASE_POINT + offset
            cv2.circle(mask, (int(center[0]), int(center[1])), int(scale*link.LENGTH), (255, 255, 255), -1)
            cv2.circle(mask, (int(center[0]), int(center[1])), int(scale*link.EXCLUSION_RADIUS), (0, 0, 0), -1)
            g_mask = cv2.bitwise_and(g_mask, mask)

        img = cv2.bitwise_and(img, img, mask=g_mask)
        return img

    def draw(self):
        self.offset = self.costmap.shape[0]//2, self.costmap.shape[1] // 2
        img = self.costmap.copy()

        for link in self.links:
            link.draw(img, self.offset, self.scale)
            # break
        return img
    #endregion 

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config1.yaml")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config, 'r'))

    controller = PCRController(**config['controller_params'])