import cv2
import numpy as np
import time
import math
import dronekit_sitl
from vision_lib import *
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import test_goto
import Servo

# Red Value
upper = np.array([179, 255, 255])
lower = np.array([141, 115, 110])

# DroneKit Variables
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')
posdata = str(vehicle.location.global_relative_frame).split(':')
poslat, poslon, Alt = posdata[1].split(',')
Alt = Alt[4:]

# CV Variables
imgName = 'None.jpg'
count = 0
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.namedWindow('result')
cap = cv2.VideoCapture(0) # 0 = Default
area_res = -1
wh = (-1,-1)
xRes = 100
yRes = 100
w, h = wh

# Main
while True :
    ret, img = cap.read()
    if img is None :
        print('image is None')
        continue 
    
    r, c, ch = img.shape
    frame = img
    result = frame    
    cv2.putText(result, str(poslat)+','+str(poslon), (0, int(r/2.1)), font, 0.40, (255, 255, 255), 1)
    count = 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = get_kernel('rect', (5, 5))
    mask = cv2.dilate(mask, kernel)

    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(mask,127,255,0)
    frame, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    posdata = str(vehicle.location.global_relative_frame).split(':')
    poslat, poslon, Alt = posdata[1].split(',')
    Alt = Alt[4:]

    for cnt in contours:
        cv2.drawContours(frame, cnt, -1, (0,255,0), 1)
        rect = cv2.minAreaRect(cnt)
        center, wh, angle = cv2.minAreaRect(cnt)
        x, y = center
        w, h = wh
        area = cv2.contourArea(cnt)
    area_res = area/(r*c)
    if not vehicle.armed :
        continue
    if float(Alt) < 1.0 :
        continue
    if area_res < 0.1056*math.pow(float(Alt)+1,(-1.41))*1.15 and area_res > 0.1056*math.pow(float(Alt)-1,(-1.41))*0.85 :
        if (w/h) > 0.85 and (w/h) < 1.17 :
            xRes = 2*(x - int(c/2))/c
            yRes = 2*(y - int(r/2))/r
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(result,[box],0,(0,0,255),2)
            imgName = str(time.strftime('%Y_%m_%d_%H_%M_'+str(count)+'.jpg'))
            count += 1
            print('x, y', xRes, yRes)
            cv2.imwrite(imgName, result)
    result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    cv2.imshow('result', cv2.resize(result, None, fx=0.5, fy=0.5))

    if xRes > 0.05 and xRes < 100 :
        test_goto.goto(vehicle, poslat,poslon+0.000001,Alt)
    elif xRes < -0.05 :
        test_goto.goto(vehicle, poslat,poslon-0.000001,Alt)
    
    if yRes > 0.05 and yRes < 100 :
        test_goto.goto(vehicle, poslat-0.000001,poslon,Alt)
    elif yRes < -0.05 :
        test_goto.goto(vehicle, poslat+0.000001,poslon,Alt)
    
    # Drop command 
    if abs(xRes) <= 0.05 and abs(yRes) <= 0.05 :
        Servo.drop(1800, vehicle)
        vehicle.mode = VehicleMode("RTL")
    #Manual Break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
cap.release()
vehicle.close()
