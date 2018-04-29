# CiiMAV METU
# Author kenkainkane
# 27th April 2018

import cv2
import numpy as np
import time
from vision_lib import *

imgName = 'None.jpg'
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.namedWindow('result')
cap = cv2.VideoCapture(0)

# hsv color value
upper = np.array([57, 225, 105])
lower = np.array([46, 154, 34])

while True :
    ret, img = cap.read()
    if img is None :
        continue

    # resize    
    r, c, ch = img.shape
    frame = cv2.resize(img.copy(), (int(c/3), int(r/3)))
    result = frame
    # get mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    # remove noise
    kernel = get_kernel('rect', (5, 5))
    mask = cv2.dilate(mask, kernel)
    # get contours
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(mask,127,255,0)
    frame, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        cv2.drawContours(frame, cnt, -1, (0,255,0), 1)
        rect = cv2.minAreaRect(cnt)
        center, wh, angle = cv2.minAreaRect(cnt)
        x, y = center
        area = cv2.contourArea(cnt)
        if area > 5000 :
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(result,[box],0,(0,0,255),2)
            # capture
            imgName = str(time.strftime('%Y_%m_%d_%H_%M_'))+'.jpg'
            cv2.imwrite(imgName, result)
    cv2.imshow('result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllwindows()
cv2.VidepCapture(0).release()
    