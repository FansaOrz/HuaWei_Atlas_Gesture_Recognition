import cv2
import numpy as np

keyPoint = np.zeros([2,60,14], dtype=float)
# f = open("./xxx.txt")
filepath = "./2_60frames_standing.txt"
f = open(filepath, "r")
num = 0
print keyPoint[0][0][13]
for line in f:
    print type(float(line)
    keyPoint[num / 840][(num / 14)%60][num%14] = float(line)


# cv2.imshow('img',img)
# cv2.imwrite("./black.jpg", img)

# cv2.waitKey(0)