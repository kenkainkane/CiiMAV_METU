import cv2
import numpy as np
import time
import dronekit_sitl
from vision_lib import *
from dronekit import connect, VehicleMode

# DroneKit Variables
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')
posdata = str(vehicle.location.global_relative_frame).split(':')
poslat, poslon, posalt = posdata[1].split(',')

# CV Variables
imgName = 'None.jpg'
count = 0
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.namedWindow('result')
cap = cv2.VideoCapture(1) # 0 = Default Camera

# HSV Color Space
upper = np.array([179, 255, 255]) #upper red 
lower = np.array([141, 110, 115]) #lower red

# Main
while True :
    ret, img = cap.read()
    if img is None :
        continue
    r, c, ch = img.shape
    frame = cv2.medianBlur(frame,9)
    result = frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = get_kernel('rect', (5, 5))
    mask = cv2.dilate(mask, kernel)

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
	res_area = area / (r * c)
        if res_area > 0.003 and res_area < 0.01 : # ~ 6 metres
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(result,[box],0,(0,0,255),2)
            cv2.putText(result, str(poslat)+','+str(poslon), (0, int(r/3.1)), font, 0.40, (255, 255, 255), 1)
            while count < 10 :
                imgName = str(time.strftime('%Y_%m_%d_%H_%M_'))+str(count)+'.jpg'
                count +=1 
            	cv2.imwrite(imgName, result)
    cv2.imshow('result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllwindows()
cv2.VidepCapture(0).release()
vehicle.close()

