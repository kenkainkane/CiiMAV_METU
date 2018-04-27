# CiiMAV METU
# Author kenkainkane
# 26th April 2018

import cv2
import numpy as np
import time
from vision_lib import *

imgName = 'None.jpg'
count = 0
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.namedWindow('result')
cap = cv2.VideoCapture(0)

upper = np.array([122, 188, 255])
lower = np.array([53, 86, 0])

while True :
    ret, img = cap.read()
    if img is None :
        continue
    r, c, ch = img.shape
    frame = cv2.resize(img.copy(), (int(c/3), int(r/3)))
    result = frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    frame = cv2.bitwise_and(frame, frame, mask=mask)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(mask,127,255,0)
    frame, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        cv2.drawContours(frame, cnt, -1, (0,255,0), 1)
        rect = cv2.minAreaRect(cnt)
        center, wh, angle = cv2.minAreaRect(cnt)
        x,y = center
        area = cv2.contourArea(cnt)
        if area > 5000 :
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(result,[box],0,(0,0,255),2)
            imgName = str(time.strftime('%Y_%m_%d_%H_%M'))+'.jpg'
            print(area)
            cv2.imwrite(imgName, result)
    cv2.imshow('result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllwindows()
cv2.VidepCapture(0).release()
    