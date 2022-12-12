import time 
import os 
import random

import cv2
import numpy as np

MIN_DISTANCE = 0.2
def get_random_end_point(controller):
    selection_map = controller.costmap.copy()

    end_point_px = (controller.end_point - controller.costmap_offset_m) * controller.scale
    selection_map = cv2.circle(selection_map, end_point_px.astype(int), int(MIN_DISTANCE*controller.scale), (0, 0, 0), -1)
    
    mask = np.argwhere(selection_map == 255)
    cv2.imshow('selection_map', selection_map)
    cv2.waitKey(0)
    i = random.randint(0, len(mask)-1)

    

    return (mask[i][::-1] / controller.scale) + controller.costmap_offset_m

def do_random_movement(workspace, model, aurora, controller):
    # Reset position from Aurora 
    # model.update_end_point(aurora.get_position())

    filename = os.path.join(workspace, str(time.time()))

    # model.enable_log(filename)
    # aurora.enable_log(filename)
    # controller.enable_log(filename)

    #movement code goes here 


    # model.disable_log()
    # aurora.disable_log()
    # controller.disable_log()





