import numpy as np 
import os 
import cv2 
import math
import matplotlib.pyplot as plt 

# 2020-11-2109-29 Developed by Alexander Rasmussen

# defining the canny detector function 

# Load image as greyscale to process.
img = cv2.imread('input/Eskild_fig_3_20.jpg', 0)

plt.hist(img.ravel(),256,[0,256]); 
plt.savefig('output/histogram.jpg')

edges = cv2.Canny(img,100, 170)

kernel_size = 3
kernel = np.ones((kernel_size, kernel_size), np.uint8)
dialate = cv2.dilate(edges, kernel)

linesP = cv2.HoughLinesP(dialate, 1, np.pi / 180, 10, None, 80, 5)
for i in range(0, len(linesP)):
   l = linesP[i][0]
   cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

cv2.imwrite('output/canny_edges.png',edges) 
cv2.imwrite('output/dialated_image.png', dialate)
cv2.imwrite('output/hough_lines.png', img) 

