import argparse

import yaml
import numpy as np
import cv2
from src.controller import PCRController


parser = argparse.ArgumentParser()
parser.add_argument('--config', type=str, default="configs/config1.yaml")
args = parser.parse_args()

config = yaml.safe_load(open(args.config, 'r'))

controller = PCRController(**config['controller_params'])
end_point = [0, 0]

mouse_pressed = False
playback = False
playback_points = []

def mouse_callback(event, x, y, flags, param):
    global end_point, mouse_pressed, playback, playback_points
    if event == cv2.EVENT_MOUSEMOVE:
        if mouse_pressed:
            playback_points += [(np.array([x, y]) - controller.offset)/controller.scale]
        else:
            end_point = (np.array([x, y]) - controller.offset)/controller.scale
            controller.update_end_point(end_point)
    elif event == cv2.EVENT_LBUTTONDOWN:
        if not playback:
            playback_points = []
            mouse_pressed = True
            playback = False
    elif event == cv2.EVENT_LBUTTONUP:
        mouse_pressed = False
        playback = len(playback_points) > 0

cv2.namedWindow("Controller")
cv2.setMouseCallback("Controller", mouse_callback)

while True:
    if playback:
        controller.update_end_point(playback_points.pop(0))
        if len(playback_points) == 0:
            playback = False
    img = controller.draw()
    # img = cv2.circle(img, (int(end_point[0]*100 + img.shape[0]//2), int(end_point[1]*100) + img.shape[1]//2), 5, (0, 0, 0), -1)
    cv2.imshow('Controller', img)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break