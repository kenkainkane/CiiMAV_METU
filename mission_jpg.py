import cv2
import numpy as np
from vision_lib import *

frame = cv2.imread('8.jpg')
frame = cv2.resize(frame, (720, 480))
result = frame

font = cv2.FONT_HERSHEY_SIMPLEX

ilowH = 13
ilowS = 123
ilowV = 192

ihighH = 27
ihighS = 199
ihighV = 255

while True :
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([ilowH, ilowS, ilowV])
    upper_hsv = np.array([ihighH, ihighS, ihighV])
    
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    kernel = get_kernel('rect', (5, 5))
    mask = cv2.dilate(mask, kernel)

    _, th = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    for c in contours :
        # cv2.drawContours(frame, c, -1, (0, 255, 0), 1)
        rect= cv2.minAreaRect(c)
        center, wh, angle = cv2.minAreaRect(c)
        x, y = center
        area = cv2.contourArea(c)
        if area > 800 :
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(result, [box], 0, (0,0,255), 2)
            cv2.putText(result, str(area), (int(x),int(y)), font, 1, (255,255,0), 1, cv2.LINE_AA)
        cv2.imshow('mask', mask)
        cv2.imshow('result', result)
    
    k = cv2.waitKey(100000) & 0xFF
    if k == 113 or 27:
        break

cv2.destroyAllWindows()
