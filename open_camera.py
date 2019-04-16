import numpy as np
import cv2
import sys

cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()
    r,c,ch = frame.shape
    cv2.line(frame,(0,int(r/2)),(int(c),int(r/2)),(255,0,0),3)
    cv2.line(frame,(int(c/2),0),(int(c/2),int(r)),(255,0,0),3)
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
