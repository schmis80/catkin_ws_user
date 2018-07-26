#!/usr/bin/env python
# roslib.load_manifest('my_package')
import sys
import cv2
import numpy as np
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

cv_image = cv2.imread("image2.png")
gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

#bi_gray
bi_gray_max = 255
bi_gray_min = 230
ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
cv2.imshow('image',thresh1)
#cv2.waitKey(0)

class blob:
  def __init__(self,i):
    self.num = i
    self.points = []
    self.size = 0

  def add_point(self, x, y, c):
    self.points.append([x, y, c])
    self.size += 1

  def get_center(self):
    # returns the (x,y) average of the mean of a blobs points
    mat = np.array(self.points)
    center = np.sum(mat, axis=0)/len(self.points)
    return (center[0], center[1])

# create a list of blobs
blobs = []
# create a reference matrix to record blob assignments
ref = np.ones(thresh1.shape) * -1

for y in range(thresh1.shape[0]):
    for x in range(thresh1.shape[1]):

        # is the pixel we are looking at white?
        if thresh1[y,x] == bi_gray_max:

            # the radius in which we look for pixels that already belong to a blob                    
            r = 9

            # look if a neighbour of x,y is already assigned to a blob
            try:
                n_blob = np.max(ref[y-r:y+r,x-r:x+r])            
            except ValueError:  #raised if `n_blob` is empty.
                n_blob = -1

            if n_blob > 0:
                blob_num = n_blob
            else:
                blob_num = len(blobs)
                blobs.append(blob(blob_num))

            # add x,y to the same blob
            blobs[int(blob_num)].add_point(x, y, 1)
            # set the reference
            ref[y,x] = blob_num

b_size_cutoff = 3

points = []

for b in blobs:
    # paint into control image and collect as img points
    if b.size >= b_size_cutoff:
        x = int(b.get_center()[0])
        y = int(b.get_center()[1])
        cv_image[y,x] = [0, 0, 255]

        points.append([x,y])
    print(b.points)
    print

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
  img_points = np.zeros((6,2,1))
  img_points[:,:,0] = np.array(points)
  
  print('\nimage points:')
  print(img_points[:,:,0])

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
retval, rvec, tvec = cv2.solvePnP(object_points, img_points, camera_matrix, dist_coeffs)

print('\nrotation vector:')
print(rvec)
print('\ntranlation vector:')
print(tvec)

rmat = np.zeros((3,3))

cv2.Rodrigues(rvec, rmat, jacobian=0)

print('\nrotation matrix:')
print(rmat)
#cv2.imshow('image',gray)
#cv2.destroyAllWindows()


