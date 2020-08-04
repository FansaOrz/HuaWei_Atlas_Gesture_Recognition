import cv2
import numpy as np
img = np.zeros((500,500),dtype=np.uint8)
#img = np.random.random((3,3))
# img[0,0]=100
# img[0,1]=150
# img[0,2]=255
cv2.imshow('img',img)
cv2.imwrite("./black.jpg", img)

cv2.waitKey(0)