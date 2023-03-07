import argparse
import yaml, sys, time

import cv2 
import numpy as np

from blmc.pid import PID

class Link:
    def __init__(self, index, config):
        self.INDEX = index
        self.LENGTH = config['length']
        self.BASE_POINT = np.array(config['base_point'])
        self.EXCLUSION_RADIUS = config['exclusion_radius']

        self.MOTOR_RADIUS = config['motor_radius']
        self.TENDON_RADIUS = config['tendon_radius']

        self.GEAR_RATIO = config['gear_ratio']


class Closed_Loop_Controller:
    def __init__(self, config, Kp=4, Kd=0, Ki=0, real_time=False):
        self.type = "Closed_Loop_Controller"
        self.real_time = real_time
        self.START_TIME = time.time()

        self.costmap = None
        self.scale = 0
        self.end_point = np.array([0.0])

        # Initialize links 
        self.links = []
        for i in range(config['num_links']):
            self.links += [Link(i, config['links'][i])]

        # Initialize cost map 
        self.bounds = config['bounds']
        self.costmap_area = np.array([config['bounds'][1][1] - config['bounds'][1][0], config['bounds'][0][1] - config['bounds'][0][0]])
        self.scale = 1000 # px/m 
        self.costmap_offset_m = np.array([config['bounds'][1][0], config['bounds'][0][0]]) # m

        size = self.costmap_area*self.scale
        self.costmap = self._draw_area(size.astype(int), self.costmap_offset_m, self.scale)

        # Initialize controller parameters 
        self.ee_pos = np.array([0, 0])
        self.ref = np.array([0, 0])
        self._pid = []
        self.u = []
        
        # Initialize PID controllers 
        for i in range(len(config['links'])):
            self._pid += [PID()]
            self._pid[-1].SetKp(Kp)
            self._pid[-1].SetKi(Ki)
            self._pid[-1].SetKd(Kd)

    def update_end_point(self, pos, timestamp=None):
        '''
        Updates control signal after receiving end effector position feedback 
        Arguments: 
        - pos: (2,) numpy array with robot end effector position (x,y) [m]

        TODO: Possible race condition here that should be fixed 
        ''' 
        self.ee_pos = pos
        u = []
        for i in range(len(self.links)):
            error = np.linalg.norm(self.ref - self.links[i].BASE_POINT) - np.linalg.norm(self.ee_pos - self.links[i].BASE_POINT)
            if self.real_time:
                u += [self._pid[i].GenOut(error, time.time())]
            else:
                assert timestamp is not None, 'Timestamp must be provided when not running in real time'
                u += [self._pid[i].GenOut(error, timestamp)]
        self.u = u

    def update_goal_point(self, goal):
        self.ref = goal 
        for p in self._pid:
            p.Initialize()

    def get_command(self):
        return self.u

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

    def draw(self, timestamp = None):
        img = self.costmap.copy()

        cv2.circle(img, (self.ref*self.scale).astype(int), 3, (0, 255, 0), -1)
        cv2.circle(img, (self.ee_pos*self.scale).astype(int), 3, (255, 0, 0), -1)

        delta1 = (self.ee_pos - self.links[0].BASE_POINT) / np.linalg.norm(self.ee_pos - self.links[0].BASE_POINT) * self.u[0]
        delta2 = (self.ee_pos - self.links[1].BASE_POINT) / np.linalg.norm(self.ee_pos - self.links[1].BASE_POINT) * self.u[1]

        cv2.arrowedLine(img, (self.ee_pos*1000).astype(int), ((self.ee_pos + delta1*10)*1000).astype(int), (0, 0, 255), 3)
        cv2.arrowedLine(img, (self.ee_pos*1000).astype(int), ((self.ee_pos + delta2*10)*1000).astype(int), (0, 0, 255), 3)

        img = cv2.flip(img, 0)

        if self.real_time:
            cv2.putText(img, f"Time: {time.time() - self.START_TIME:.3}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))
        elif timestamp is not None:
            cv2.putText(img, f"Time: {timestamp:.3}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))

        return img

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config1.yaml")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config, 'r'))

    controller = Closed_Loop_Controller(config['controller_params'])
    goal = np.array([0.3, 0.3])
    
    curr_loc = np.array([0., 0.])

    def mouse_callback(event,x,y,flags,param):
        global goal
        if event==cv2.EVENT_MOUSEMOVE:
            goal = np.array([x,controller.costmap.shape[1] - y]) / controller.scale + controller.costmap_offset_m
            print(goal)

    cv2.namedWindow('Closed loop sim') 
    cv2.setMouseCallback('Closed loop sim',mouse_callback)   

    i = 0 
    dt = 0.01

    START_TIME = time.time()
    while True:
        timestamp = i / 100
        controller.update_goal_point(goal)

        # Motor control loop for 5s 
        if i % 20 == 0: 
            # Update aurora at 5 hz 
            controller.update_end_point(curr_loc, timestamp)

        u = controller.get_command()

        delta1 = (curr_loc - controller.links[0].BASE_POINT) / np.linalg.norm(curr_loc - controller.links[0].BASE_POINT) * u[0]
        delta2 = (curr_loc - controller.links[1].BASE_POINT) / np.linalg.norm(curr_loc - controller.links[1].BASE_POINT) * u[1]
        curr_loc += delta1 + delta2 #+ np.random.rand(2)*0.001

        i += 1

        # Draw 
        disp = controller.draw(timestamp)
        cv2.imshow("Closed loop sim", disp)
        delay = int(max(0.001, timestamp - time.time() + START_TIME)*1000)
        k = cv2.waitKey(delay)
        if k == ord('q'):
            break

        

