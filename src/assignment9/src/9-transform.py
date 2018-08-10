import cv2
import numpy as np
from matplotlib import pyplot as plt

px_size = 0,7 #mm

cv_image = cv2.imread('marker.png')
street = cv2.imread('streetview.png')
ir = cv2.imread('image_ir.png')

img_pts = np.float32([
 [ 227,  377.],[ 418.,  377.],
 [  185.,  451.],[ 464.,  454.]
 ])
warp_pts = np.float32([[585.,1775],[864.,1775],[585.,2051.],[864.,2054.]])
pts2 = np.float32([[185,175],[464,175],[185,451],[464,454]])

M = cv2.getPerspectiveTransform(img_pts,warp_pts)

dst1 = cv2.warpPerspective(cv_image,M,(1040,2080))
dst2 = cv2.warpPerspective(street,M,(1040,2080))

plt.subplot(121),plt.imshow(cv_image),plt.title('Input1')
plt.subplot(122),plt.imshow(dst1),plt.title('Output1')
#plt.subplot(121),plt.imshow(street),plt.title('Original')
#plt.subplot(122),plt.imshow(dst2),plt.title('Top down')

cv2.imwrite('transformed.png',dst1)
plt.show()
