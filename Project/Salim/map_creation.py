import cv2
import matplotlib.pyplot as plt
from skimage.filters import threshold_local
import numpy as np

scaling = 100 #px/cm


warped = cv2.imread('thymio_warped.png')

ruru = warped.copy()
ruru = cv2.cvtColor(ruru, cv2.COLOR_BGR2GRAY)


blurred = cv2.GaussianBlur(ruru, (5,5), 0)
#blurred = cv2.medianBlur(ruru,91)
edged = cv2.Canny(ruru, 195, 200)

plt.imshow(edged, cmap = 'gray')
T = threshold_local(edged, 7, offset = 10, method = "gaussian")
edged = 255 - (edged > T).astype("uint8") * 255
plt.figure()
plt.imshow(edged, cmap='gray')

contours = cv2.findContours(edged.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)



empty = np.zeros(edged.shape)
#cv2.drawContours(empty, contours[0], contourIdx=-1, color=(255,255,255),thickness=-1)
cv2.fillPoly(empty,pts=contours[0],color=(255,255,255))
#empty = cv2.medianBlur(empty,5)

plt.figure()
plt.imshow(empty, cmap='gray')



t_r = 25  # tolerance

red_thresh = cv2.inRange(warped, np.array([0x6F-t_r, 0x2E -t_r, 0x27-t_r]), np.array([0x6F+t_r, 0x2E +t_r, 0x27+t_r]))
red_mask = 255 - red_thresh

red_mask_rgb = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2RGB)
final_r = 255 - cv2.max(warped, red_mask_rgb)

plt.figure()
plt.imshow(final_r)


t_k = 50
black_thresh = cv2.inRange(warped, np.array([-t_k, -t_k, -t_k]), np.array([t_k, t_k, t_k]))
black_mask = 255 -black_thresh 
black_mask_rgb = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2RGB)
final_k = 255 - cv2.max(warped, black_mask_rgb)

plt.figure()
plt.imshow(final_k)

## extend forms
size_kernel = 50 #Tune obstacle size changing
kernel = np.ones((size_kernel,size_kernel),np.uint8)
erosion = cv2.erode(final_k,kernel,iterations = 2)
dilation = cv2.dilate(erosion,kernel,iterations = 4)
plt.figure()
plt.imshow(dilation)

# remove noise
final_k_bw = cv2.cvtColor(final_k, cv2.COLOR_RGB2GRAY)
se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (15,15))
mask = cv2.morphologyEx(final_k_bw, cv2.MORPH_CLOSE, se1)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)

mask = np.dstack([mask, mask, mask]) / 255
out = final_k * mask

plt.figure()
plt.imshow(out)

# dilate
size_kernel = 4*scaling #Tune obstacle size changing
kernel = np.ones((size_kernel,size_kernel),np.uint8)
dilation = cv2.dilate(out,kernel,iterations = 1)
plt.figure()
plt.imshow(dilation)

