import numpy as np
import cv2
import sys

if sys.argv[1] == 'body' :
    cap = cv2.VideoCapture(0)
if sys.argv[1] == 'bottom' :
    cap = cv2.VideoCapture(1)

while(True):
    ret, frame = cap.read()
    r,c,ch = frame.shape
    frame = cv2.resize(frame.copy(),(int(c/2),int(r/2)))
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
