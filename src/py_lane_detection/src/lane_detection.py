#!/usr/bin/env python

import sys
import cv2
import numpy as np

def get_image(path):
    """get picture, return it and a white pixel mask
    param: path path to image file
    return image and mask 
    """
    # get image
    cv_image = cv2.imread("Bilder/streetview.png")
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get mask
    lower_hsv = np.array([0,0,220])
    upper_hsv = np.array([179,51,255])
    hsv_red_min = np.array([0, 70, 50])
    hsv_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    hsv_mask = cv2.bilateralFilter(hsv_mask,9,75,75)

    return cv_image, hsv_mask

def get_distance(points, m, c):
    """ Uebernommen von
    https://github.com/AutoModelCar/model_car/blob/version-3-kinetic/catkin_ws/src/fub_steering_calibration/scripts/angle_calibrator_online.py#L76
    """
    pos = np.array((0, c))  # origin
    dir = np.array((1, m))  # line gradient
    # element-wise cross product of each points origin offset with the gradient
    cross = np.cross(dir, pos - points, axisb=-1)
    return np.abs(cross) / np.linalg.norm(dir)

def get_inliers(points, m, c):
    """Uebernommen von 
    https://github.com/AutoModelCar/model_car/blob/version-3-kinetic/catkin_ws/src/fub_steering_calibration/scripts/angle_calibrator_online.py#L85
    """
    return get_distance(points, m, c) <= 5

def find_best_params(points):
    """Uebernommen von 
    https://github.com/AutoModelCar/model_car/blob/version-3-kinetic/catkin_ws/src/fub_steering_calibration/scripts/angle_calibrator_online.py#L90
    """
    best_count = 0
    best_params = (0,0)

    xs, ys = points.T
    
    for _ in xrange(50):
        ind = np.random.randint(0, len(xs), 2)
        x_a, x_b = xs[ind]
        if x_a == x_b:
            continue

        y_a, y_b = ys[ind].astype(np.float64)
        
        m = (y_b - y_a) / (x_b - x_a)
        c = y_a - x_a*m

        inlier = (get_distance(points, m, c) <= 5)
        inl_count = np.sum(inlier)
        if inl_count > best_count:
            best_count = inl_count
            best_params = (m,c)
    
    return best_params
    

cv_image, mask = get_image('Bilder/streetview.png')
white = np.nonzero(mask)
points = np.column_stack((white[1],white[0]))
ms = []
cs = []

for _ in range(3):
    m, c = find_best_params(points)
    
    ms.append(m)
    cs.append(c)
    
    points = points[np.where(get_distance(points, m, c) > 5)]

ms.append(m)
cs.append(c)

for i in range(len(ms)):
    cv2.line(cv_image,
             (0,int(round(cs[i]))),
             (cv_image.shape[1], int(round(ms[i]*cv_image.shape[1]+cs[i]))),
             (0, 0, 255))
cv2.imshow('line', cv_image)
cv2.waitKey(0)
