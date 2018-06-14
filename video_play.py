import numpy as np
import cv2

cap = cv2.VideoCapture('output.avi')
while True:
    ret, frame = cap.read()
    if ret == True:
        cv2.imshow('frame',frame)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    else:
        break
cap.release()
cv2.destroyAllWindows()
