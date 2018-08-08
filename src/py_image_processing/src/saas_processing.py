#!/usr/bin/env python
# roslib.load_manifest('my_package')
import sys
import cv2
import numpy as np
import math
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

cv_image = cv2.imread("image2.png")
gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

#bi_gray
bi_gray_max = 255
bi_gray_min = 205
ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
cv2.imshow('image',thresh1)
#cv2.waitKey(0)

class Cluster:
  def __init__(self,i):
    self.num = i
    self.points = []
    self.size = 0

  def add_point(self, x, y, c):
    self.points.append([x, y, c])
    self.size += 1

  def get_center(self):
    mat = np.array(self.points)
    center = np.sum(mat, axis=0)/len(self.points)
    return (center[0], center[1])

# list of clusters
clusters = []
# reference matrix to record cluster assignments
ref = np.ones(thresh1.shape) * -1

for y in range(thresh1.shape[0]):
    for x in range(thresh1.shape[1]):
        # is the pixel we are looking at white?
        if thresh1[y,x] >=bi_gray_min and thresh1[y,x] <= bi_gray_max:
            # radius in which the point will belong to a cluster 
            r = 9

            # look for neighbours, already assigned to a cluster
            try:
                cluster_num = np.max(ref[y-r:y+r,x-r:x+r])            
            except ValueError:  #raised if 'cluster_num' is empty.
                cluster_num = -1

            if cluster_num <= 0:
                cluster_num = len(clusters)
                clusters.append(Cluster(cluster_num))

            # add Point to cluster
            clusters[int(cluster_num)].add_point(x, y, 1)
            # set reference
            ref[y,x] = cluster_num

# minimum cluster size
c_min_size = 3

points = []

for cluster in clusters:
    # paint into control image and collect as img points
    if cluster.size >= c_min_size:
        x, y = cluster.get_center()
        cv_image[int(y),int(x)] = [0, 0, 255]
        points.append([int(x),int(y)])

cv2.imshow('image',cv_image)
cv2.waitKey(0)

fx = 614.1699
fy = 614.9002
cx = 329.9491
cy = 237.2788

object_points = np.array([[0.,0.,0.],
                          [40.,0.,0.],
                          [0.,30.,0.],
                          [40.,30.,0.],
                          [0.,60.,0.],
                          [40.,60.,0.]])

print('object points:')
print(object_points)

if len(points) == 6:
  img_points = np.zeros((6,2))
  image_points = np.zeros((6,2,1))
  img_points = np.array(points)
  image_points[:,:,0] = np.array(points)
  
  print('\nimage points:')
  print(img_points)

k1 = 0.1115
k2 = -0.1089
t1 = 0
t2 = 0

dist_coeffs = np.zeros((4,1))
dist_coeffs[:,0] = np.array([[k1, k2, t1, t2]])

camera_matrix = np.array([[fx,  0, cx],
                         [ 0, fy, cy],
                         [ 0,  0,  1]])
#print(cameraMatrix)
retval, rvec, tvec = cv2.solvePnP(object_points, 
				  image_points, 
                                  camera_matrix, 
                                  dist_coeffs)

print('\nrotation vector:')
print(rvec)
print('\ntranslation vector:')
print(tvec)

rmat = np.zeros((3,3))

cv2.Rodrigues(rvec, rmat, jacobian=0)

print('\nrotation matrix:')
print(rmat)

inv_rmat = rmat.transpose()
print 'inv_rmat \n' , inv_rmat
inv_rmat_ = np.negative(inv_rmat)
inv_tvec = inv_rmat_.dot(tvec)
print 'inv_tvec \n' , inv_tvec
sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0]);
singular = sy < 1e-6; # If
if not singular:
     x = math.atan2(-rmat[2,1] , rmat[2,2]);
     y = math.atan2(-rmat[2,0], sy);
     z = math.atan2(rmat[1,0], rmat[0,0]);
else:
     x = math.atan2(-rmat[1,2], rmat[1,1]);
     y = math.atan2(-rmat[2,0], sy);
     z = 0;
print 'x,y,z', x,y,z


#cv2.imshow('image',gray)
#cv2.destroyAllWindows()


