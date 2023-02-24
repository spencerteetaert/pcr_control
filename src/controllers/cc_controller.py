import argparse
import yaml, sys

import numpy as np
import cv2
from scipy.optimize import fsolve

if __name__!='__main__':
    from ..math_helper import convert_arc
else:
    sys.path.append('src/')  
    from math_helper import convert_arc

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
        q = [self.k * self.GEAR_RATIO * self.TENDON_RADIUS * self.LENGTH / self.MOTOR_RADIUS, -self.k * self.GEAR_RATIO * self.TENDON_RADIUS * self.LENGTH / self.MOTOR_RADIUS]
        '''
        q is in rad in motor frame 
        '''
        self.dq = [q[0] - self.q[0], q[1] - self.q[1] ]
        return q

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
        centerCoordinates = scale*(self.BASE_POINT + offset_point)
        centerCoordinates = (int(centerCoordinates[0]), int(centerCoordinates[1]))
        return [centerCoordinates, radius, color, -1]
    #endregion 

class CC_Model:
    def __init__(self, config):
        self.type = "CC_Model"
        self.links = []
        for i in range(config['num_links']):
            self.links += [Link(i, config['links'][i])]

        self.costmap_area = np.array([0.5, 0.5])
        self.scale = 40 # px/m 
        self.costmap_offset_m = np.array([0,0]) # self.costmap_area / 2

        size = self.costmap_area*self.scale
        self.costmap = self._draw_area(size.astype(int), self.costmap_offset_m, self.scale)

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
        self.end_point = self.links[0].end_point
        return True

    def update_qs(self, dqs):
        ks = []
        mags = []

        for i in range(len(self.links)):
            self.links[i].q = [self.links[i].q[0] + dqs[i], self.links[i].q[1] - dqs[i]] #TODO: Fix 
            ks += [self.links[i].q[0] * self.links[i].MOTOR_RADIUS / (self.links[i].GEAR_RATIO*self.links[i].LENGTH*self.links[i].TENDON_RADIUS)]
            mags += [2/ks[-1] * np.sin(self.links[i].LENGTH*ks[-1]/2)]

        d = np.linalg.norm(self.links[0].BASE_POINT - self.links[1].BASE_POINT)
        a = (self.links[0].LENGTH**2 - self.links[1].LENGTH**2) / (2*d)
        h = (self.links[0].LENGTH**2 - a**2)**0.5
        x5 = self.links[0].BASE_POINT[0] + a*(self.links[0].BASE_POINT[0] - self.links[1].BASE_POINT[0])/d
        y5 = self.links[0].BASE_POINT[1] + a*(self.links[0].BASE_POINT[1] - self.links[1].BASE_POINT[1])/d

        x = x5 + h*(self.links[1].BASE_POINT[1] - self.links[0].BASE_POINT[1]) / d
        y = y5 + h*(self.links[1].BASE_POINT[0] - self.links[0].BASE_POINT[0]) / d

        self.update_end_point([x, y])

    #endregion
        
    #region drawing
    def _draw_area(self, img_size, offset, scale):
        img = np.ones(img_size, dtype=np.uint8)*255
        g_mask = np.ones(img_size[:2], dtype=np.uint8)
        
        for link in self.links:
            mask = np.zeros(img_size[:2], dtype=np.uint8)
            center = scale*(link.BASE_POINT - offset)
            cv2.circle(mask, (int(center[0]), int(center[1])), int(scale*link.LENGTH), (255, 255, 255), -1)
            cv2.circle(mask, (int(center[0]), int(center[1])), int(scale*link.EXCLUSION_RADIUS), (0, 0, 0), -1)
            g_mask = cv2.bitwise_and(g_mask, mask)

        img = cv2.bitwise_and(img, img, mask=g_mask)
        return img

    def draw(self):
        img = self.costmap.copy()

        for link in self.links:
            link.draw(img, self.costmap_offset_m, self.scale)
            # break
        return img
    #endregion 

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config1.yaml")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config, 'r'))

    controller = CC_Model(config['controller_params'])