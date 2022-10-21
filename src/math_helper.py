from email.mime import base
import numpy as np
from math import sin, cos, atan2, pi

def get_R(theta):
    R = np.array([
        [cos(theta), sin(theta)],
        [-sin(theta), cos(theta)]
    ])
    return R

def convert_arc(base_point, end_point, arc_length, radius):
    arc_angle = arc_length / radius
    alpha = (pi - arc_angle)/2

    # Unit vector from base to end point 
    v = (end_point - base_point) / np.linalg.norm(end_point - base_point)
    
    # unit vector towards ellipse center from base point 
    u = v @ get_R(-alpha)

    # length from base point to ellipse center 
    b = np.linalg.norm(end_point - base_point) / (2*cos(alpha))

    arc_center = base_point + b*u

    base_point_v = base_point - arc_center 
    end_point_v = end_point - arc_center 

    pt1_angle = atan2(base_point_v[1], base_point_v[0]) * 180 / pi
    pt2_angle = atan2(end_point_v[1], end_point_v[0]) * 180 / pi


    # Logic just for drawing arc in proper orientation, does not effect model at all
    s = pt1_angle < pt2_angle

    b1 = pt1_angle <= 0 and pt1_angle > -90
    b2 = pt1_angle <= -90
    b3 = pt1_angle >= 90
    b4 = pt1_angle > 0 and pt1_angle < 90

    e1 = pt2_angle <= 0 and pt2_angle > -90
    e2 = pt2_angle <= -90
    e3 = pt2_angle >= 90
    e4 = pt2_angle > 0 and pt2_angle < 90

    if (not b3 and (e4 or (e1 and not b4))) or (e2 and b2) or e3:
        if ((e1 and b1) or (e2 and b2) or (e3 and b3) or e4 and b4) and not s:
            pass
        else:
            pt2_angle -= 360

    return (int(arc_center[0]), int(arc_center[1])), (int(radius), int(radius)), int(pt1_angle), int(pt2_angle)