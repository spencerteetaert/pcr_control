import time 
import logging
# logging.basicConfig(level=logging.DEBUG)

import numpy as np
np.random.seed(0)
import cv2
import matplotlib.pyplot as plt

#region Path Planning
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = tuple(position)
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position 

def astar(map, start, end, noise_map=None, visualize=False):
    if visualize:
        disp_map = np.copy(map)
        disp_map = np.stack([disp_map, disp_map, disp_map]).astype(np.uint8)
        disp_map = np.transpose(disp_map, (1, 2, 0))
    START = time.time() 

    

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    if map[end_node.position[1]][end_node.position[0]] != 255:
        print("Requested position is unreachable")
        return []

    open_list = []
    closed_list = []

    open_list.append(start_node)
    index = 0
    
    while len(open_list) > 0:
        current_node = open_list[0]
        
        # Finds lowest f score node, pops that one --- TODO: Make into tree structure 
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        if visualize:
            disp_map[current_node.position[1], current_node.position[0]] = [0, 255, 0]
            cv2.imshow("test", cv2.resize(disp_map, (500, 500)))
            cv2.waitKey(3)

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            if visualize:
                cv2.waitKey(0)
            path = []
            current = current_node
            while current is not None:
                path.append(np.array(current.position))
                current = current.parent
            logging.info(f"Path found in {round(time.time() - START, 2)}s")
            return np.array(path[::-1])

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # Make sure within range
            if node_position[0] > (map.shape[1]-1) or node_position[0] < 0 or node_position[1] > (map.shape[0]-1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if map[node_position[1]][node_position[0]] != 255:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g 
            if noise_map is not None:
                child.g += noise_map[child.position[0], child.position[1]]
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open or closed list
            if (child in open_list) or (child in closed_list):
                continue

            # Add the child to the open list -- TODO: Remove lower f child 
            if visualize:
                disp_map[child.position[1], child.position[0]] = [0, 0, 255]
            open_list.append(child)
    
    logging.info("Path not found.")
    return []


def generate_perlin_noise_2d(shape, res):
    def f(t):
        return 6*t**5 - 15*t**4 + 10*t**3

    delta = (res[0] / shape[0], res[1] / shape[1])
    d = (shape[0] // res[0], shape[1] // res[1])
    grid = np.mgrid[0:res[0]:delta[0],0:res[1]:delta[1]].transpose(1, 2, 0) % 1
    # Gradients
    angles = 2*np.pi*np.random.rand(res[0]+1, res[1]+1)
    gradients = np.dstack((np.cos(angles), np.sin(angles)))
    g00 = gradients[0:-1,0:-1].repeat(d[0], 0).repeat(d[1], 1)
    g10 = gradients[1:,0:-1].repeat(d[0], 0).repeat(d[1], 1)
    g01 = gradients[0:-1,1:].repeat(d[0], 0).repeat(d[1], 1)
    g11 = gradients[1:,1:].repeat(d[0], 0).repeat(d[1], 1)
    # Ramps
    n00 = np.sum(grid * g00, 2)
    n10 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1])) * g10, 2)
    n01 = np.sum(np.dstack((grid[:,:,0], grid[:,:,1]-1)) * g01, 2)
    n11 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]-1)) * g11, 2)
    # Interpolation
    t = f(grid)
    n0 = n00*(1-t[:,:,0]) + t[:,:,0]*n10
    n1 = n01*(1-t[:,:,0]) + t[:,:,0]*n11
    return np.sqrt(2)*((1-t[:,:,1])*n0 + t[:,:,1]*n1)
#endregion

#region Trajectory Planning
class Profile:
    def __init__(self, index, start_time, L, C_smoothness=4):
        # TODO ? : Add max jerk values, add synchronization between motors 
        self.v_max = 1 # max velocity of motors, m/s in tendon displacement 
        self.a_max = 5 # max acceleration of motors, m/s**2 in tendon displacement 
        self.d_max = 5 # max deacceleration of motors, m/s**2 in tendon displacement 
        assert C_smoothness > 1 and C_smoothness < 12, "C_smoothness only support between 2 and 11"
        self.C_smoothness = C_smoothness
        self.start_time = start_time
        self.index = index

        self.Ca = [0, 0, 3/2, 15/8, 35/16, 315/128, 693/256, 3003/1024, 6435/2048, 109395/32768, 230945/65536, 969969/262144]
        self.Cj = [
            0, 0, 
            6**0.5, 
            (10/3**0.5)**0.5,
            (84/(5*5**0.5))**0.5,
            (1215/(49*7**0.5))**0.5,
            (24640/2187)**0.5,
            (2559375/(58564*11**0.5))**0.5,
            (20207880/(371293*13**0.5))**0.5,
            (2002033033/(30375000*15**0.5))**0.5,
            (32051036160/(410338673*17**0.5))**0.5,
            (98891016919695/(1086948034624*19**0.5))**0.5
        ] 

        self.pre_blend = False
        self.post_blend = False
        self.last_profile = None
        self.next_profile = None

        self.update_time_constants(L)

    def __add__(self, other):
        raise NotImplementedError()
    def __mult__(self, factor):
        raise NotImplementedError()

    def update_time_constants(self, L):
        # Time constants 
        self.L = L
        self.sign = -1 if self.L < 0 else 1

        self.Tlo = self.Ca[self.C_smoothness] * self.v_max / self.a_max
        self.Tsd = self.Ca[self.C_smoothness] * self.v_max / self.d_max
        self.Tcr = abs(self.L) / self.v_max - 0.5*(self.Tlo + self.Tsd)

        if self.Tcr < 0:
            self.lamb = 2*self.a_max*self.d_max*abs(self.L) / ((self.a_max + self.d_max)*self.v_max**2*self.Ca[self.C_smoothness])
            self.Tcr = 0
        else:
            self.lamb = 1
            
        self.T = self.Tlo + self.Tsd + self.Tcr


    def tau(self, t):
        if t >= 0 and t < self.Tlo: 
            return t / self.Tlo
        elif t >= self.Tlo and t < self.Tlo + self.Tcr:
            return (t - self.Tlo)/self.Tcr
        elif t >= self.Tlo + self.Tcr: # TODO: Add upper bound
            return (t - self.Tlo - self.Tcr)/self.Tsd

    def vn4(self, tau):
        return -20*tau**7 + 70*tau**6 - 84*tau**5 + 35*tau**4
    def Vn4(self, tau):
        return -2.5*tau**8 + 10*tau**7 - 14*tau**6 + 7*tau**5
    
    def v(self, _t):
        t = _t - self.start_time
        if t < 0:
            raise ValueError("Requested time is before profile defined range")

        elif t < self.Tlo: # Lift off
            if not self.pre_blend:
                return self.sign*self.lamb*self.v_max*self.vn4(self.tau(t))
            raise
            return self.last_profile.sign*self.last_profile.v_max + (self.sign*self.v_max - self.last_profile.sign*self.last_profile.v_max)*self.vn4(self.tau(t))

        elif t < self.Tlo + self.Tcr: # Cruise 
            return self.sign*self.lamb*self.v_max

        elif t <= self.T + 0.00001: # Set down, small constant for floating point error
            if not self.post_blend:
                return self.sign*self.lamb*self.v_max*self.vn4(1 - self.tau(t))
            # return self.sign*self.v_max + (self.next_profile.sign*self.next_profile.v_max - self.sign*self.v_max)*self.vn4(self.tau(t))
            return self.sign*self.v_max*self.lamb + (self.next_profile.sign*self.next_profile.lamb*self.next_profile.v_max - self.sign*self.lamb*self.v_max)*self.vn4(self.tau(t))
        else:
            print(self.T, self.Tlo, self.Tcr, self.Tsd)
            raise ValueError(f"Requested time is after profile defined range.\n  Profile {self.index}: {t}s")

    # def s(self, _t): 
    #     t = _t - self.start_time 
    #     if t < 0:
    #         return 0

    #     elif t < self.Tlo:
    #         if not self.pre_blend:
    #             return self.sign*self.lamb*self.v_max*self.Tlo*self.Vn4(self.tau(t))
    #         return self.last_profile.sign*self.last_profile.v_max*(t-self.Tlo) + self.Tlo*(self.v_max - self.last_profile.v_max)*self.Vn4(t - self.Tlo)

    #     elif t < self.Tlo + self.Tcr:
    #         return self.sign*self.lamb*self.v_max*(self.Tcr*self.tau(t) + self.Tlo/2)

    #     elif t < self.T:
    #         if not self.post_blend:
    #             return self.sign*(abs(self.L) - self.lamb*self.v_max*self.Tsd*self.Vn4(1 - self.tau(t)))
    #         return 0

    #     else:
    #         return self.L


def blend_profiles(p1, p2):
    '''
    Given two profiles p1 and p2, blend them together. 
    '''
    assert (p1.v_max == p2.v_max), NotImplementedError()
    # assert(p1.lamb == p2.lamb == 1), NotImplementedError(f"{p1.lamb}, {p2.lamb}")
    
    p1.post_blend = True
    p2.pre_blend = True

    v_star = p2.sign*p2.v_max - p1.sign*p1.v_max
    Tble = p1.Ca[4]*max(
        abs(v_star)/p2.a_max,
        abs(v_star)/p1.d_max,
        p2.v_max/p2.a_max,
        p1.v_max/p1.d_max
    )
    # Shift start time of the later node 
    p1_original_T = p1.T

    p1.d_max = p1.v_max*p1.Ca[4]/Tble
    p2.a_max = p2.v_max*p2.Ca[4]/Tble

    p1.update_time_constants(p1.L)
    p2.update_time_constants(p2.L)
    # p1.Tsd = Tble
    # p2.Tlo = Tble

    p2.start_time += p1.Tlo + p1.Tcr - p1_original_T

    p1.next_profile = p2
    p2.last_profile = p1   

class TrajectoryPlanner:
    def __init__(self, C_smoothness=4):
        assert C_smoothness > 1 and C_smoothness < 12, "C_smoothness only support between 2 and 11"
        self.C_smoothness = 4
        self.blend = True

    def gen_trajectory(self, PCR_controller, target_pt, dt=0.001, add_noise=False, verbose=True):
        '''
        Given map centered at 0m,0m, a gridscale, and starting/ending points, generates a smooth 
        profile for motor velocities. 

        Args:
            PCR_controller: PCR_controller object for robot
            target_pt (tuple): ending point (m) 
            dt (float): time between returned velocity commands (s) 
            add_noise (bool): whether to add positional noise to the trajectory 

        Returns: 
            nx4 list containing discrete velocity commands for each motor, each dt time apart
        '''
        costmap = PCR_controller.costmap
        start_pt_px = (np.array(PCR_controller.end_point)*PCR_controller.scale + np.array(costmap.shape) / 2).astype(int)
        end_pt_px = (np.array(target_pt)*PCR_controller.scale + np.array(costmap.shape) / 2).astype(int)

        # Path planning testing 
        if add_noise:
            noise_map = generate_perlin_noise_2d(costmap.shape, (10, 10))*5
            noise_map = np.clip(noise_map, 0, 1)*255
            path = astar(costmap, start_pt_px, end_pt_px, noise_map=noise_map, visualize=add_noise)
        else:
            path = astar(costmap, start_pt_px, end_pt_px, visualize=verbose)


        if verbose:
            disp_image = np.zeros((*costmap.shape, 3)).astype(np.uint8)
            disp_image[...,0] = costmap
            disp_image[...,1] = costmap
            disp_image[...,2] = costmap

            for item in path:  
                disp_image[item[1], item[0]] = [0, 255, 0]

            cv2.imshow("Generated path", cv2.resize(disp_image, (500, 500)))
            cv2.waitKey(0)

        '''
        Path is nx2 points on map 
        1) scale points to physical space 
        2) for each point:
            i) solve for K
            ii) solve for tendon displacements 
        3) generate and return smooth tendon displacement profile 
        '''

        path = [(item - np.array(costmap.shape) / 2) / PCR_controller.scale for item in path]
        qs = [] # rad
        for item in path: 
            if not PCR_controller.update_end_point(item):
                print("Unable to set link to point", item)
            
            print(np.array([link.dq for link in PCR_controller.links]).flatten().shape)
            qs += [np.array([link.dq for link in PCR_controller.links]).flatten().tolist()]

        print(np.array(qs), np.array(qs).shape)
        smoothed_qs = [self.gen_smooth_trajectory(np.array(qs)[:,i], 1/dt) for i in range(len(qs[0]))]

        return smoothed_qs




    def gen_smooth_trajectory(self, pos_values, command_freq, verbose = True):
        '''Given a series of relative position commands, generates a Cn smooth trajectory linking each point.

        Args: 
            pos_values (iterable): position values to fit path to 
            command_freq (int): frequency for return values to be executed at (hz) 
        
        Returns:
            list of smoothly interpolated values fit to pos_values input to be executed at command_freq
        
        '''
        START = time.time()
        profiles = []
        profiles += [Profile(0, 0, pos_values[0], self.C_smoothness)]

        # Initialize profile objects + blending
        for i, pos_value in enumerate(pos_values[1:]):
            profiles += [Profile(i+1, profiles[-1].start_time + profiles[-1].T, pos_value, self.C_smoothness)]
            if self.blend:
                blend_profiles(profiles[-2], profiles[-1])

        # Create time profiles
        if self.blend:
            profile_times = [profiles[0].T] + [profiles[i].T - profiles[i].Tlo for i in range(1, len(profiles))]
        else:
            profile_times = [profiles[i].T for i in range(0, len(profiles))]
        
        boundaries = np.cumsum(profile_times)
        total_time = np.sum(profile_times)
        command_times = np.linspace(0, total_time, int(total_time * command_freq))

        print('\n'.join([f"Profile {i}:" + str((s.start_time, s.start_time + s.T, s.T)) for i, s in enumerate(profiles)]))

        # Generate velocity profile
        profile_output_v = []
        index = 0
        for t in command_times:
            if t > boundaries[index]:
                index += 1
            profile_output_v += [profiles[index].v(t)]

        if verbose:
            print(f"Trajectory generation runtime: {time.time() - START}")
            plt.plot(command_times, profile_output_v, label="Planned trajectory velocity")

            # Draw manually integrated position profile, TODO: find analytic solution 
            pos_estimate = []
            curr_pos = 0
            for vel in profile_output_v:
                curr_pos += vel*1/command_freq
                pos_estimate += [curr_pos]
            plt.plot(command_times, pos_estimate, label="Estimated trajectory position")
            
            # Draw signal profile 
            aligned_pos_values = []
            pos_values_cumsum = np.cumsum(pos_values)
            boundaries_start = [profiles[i].start_time + profiles[i].Tsd for i in range(1, len(profiles))] + [profiles[-1].start_time + profiles[-1].T]

            index = 0
            for t in command_times:
                if t > boundaries_start[index]:
                    index += 1
                aligned_pos_values += [pos_values_cumsum[index]]
            plt.plot(command_times, aligned_pos_values, label="Desired position")

            plt.legend()
            plt.show()

        return profile_output_v
#endregion

if __name__=='__main__':
    ## Trajectorty planning testing 
    planner = TrajectoryPlanner()
    planner.gen_smooth_trajectory([1, -1.5, 2, 3, -3.5, 0.3, -1.5, 0.2], 100)
    planner.gen_smooth_trajectory([0.1, 0.06, 0.3, -0.07, -0.01, -0.1, 0.05], 100)
    planner.gen_smooth_trajectory([1, -1, 0.5], 3000)


    # Path planning testing 
    # map_size = (50, 50)
    # test_map = np.ones(map_size)*255
    # cv2.circle(test_map, (35, 20), 10, (0, 0, 0), -1)
    # noise_map = generate_perlin_noise_2d(map_size, (10, 10))*5
    # noise_map = np.clip(noise_map, 0, 1)*255

    # path = astar(test_map, [5, 5], [43, 44], noise_map=noise_map, visualize=False)

    # disp_image = np.zeros((*map_size, 3)).astype(np.uint8)
    # disp_image[...,0] = test_map
    # disp_image[...,1] = test_map
    # disp_image[...,2] = test_map

    # for item in path:  
    #     disp_image[item[1], item[0]] = [0, 255, 0]

    # cv2.imshow("test", cv2.resize(disp_image, (500, 500)))
    # cv2.waitKey(0)


    # disc_pos_points = path / 25 - 1 # scaled from -1 to 1
    # print(disc_pos_points)

    # plt.scatter(disc_pos_points[:,0], disc_pos_points[:,1]*-1, s=1)
    # plt.legend()
    # plt.show()
