import cv2
import numpy as np
import time
from math import sin,cos,atan2,pi,sqrt
from vision_lib import *
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from mission_lib import *

# Red Value
upper = np.array([179, 255, 255])
lower = np.array([140, 110, 0])

# DroneKit Variables
print(">>>>>>>>>>>>>>>>>>>>> Connecting to UAV <<<<<<<<<<<<<<<<<<<")
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')
print(">>>>>>>>>>>>>>>> UAV has been connected <<<<<<<<<<<<<<<<<<")
poslat, poslon, Alt = get_GPSvalue(vehicle)


#trigger = serial.Serial("/dev/ttyACM1",9600)
#data = int(trigger.readline()[:4])
#first = 0

#while True:
#    trigger = serial.Serial("/dev/ttyACM1",9600)
#    data = int(trigger.readline()[:4])
#    print("Servo value =",data)
#    if data >= 1500:
#        break
#    if first == 0:
#        print("Waitting for servo trigger")
#        first = 1
#    data = int(trigger.readline()[:4])
#    continue

print(">>>>>>>>   START MISSION   <<<<<<<<<<<")

# CV Variables
imgName = 'None.jpg'
count = 0
font = cv2.FONT_HERSHEY_SIMPLEX
#cv2.namedWindow('result')
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

    poslat, poslon, Alt = get_GPSvalue(vehicle)
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
    poslat,poslon,Alt = get_GPSvalue(vehicle)
    if area_res < 0.1056*math.pow(float(Alt)+1,(-1.41))*1.15 and area_res > 0.1056*math.pow(float(Alt)-1,(-1.41))*0.85 :
        if (w/h) > 0.85 and (w/h) < 1.17 :
            xRes = 2*(x - int(c/2))/c
            yRes = 2*(y - int(r/2))/r
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(result,[box],0,(0,0,255),2)
            imgName = str(time.strftime('%Y_%m_%d_%H_%M_'+str(count)+'.jpg'))
            count += 1
            print('x, y = ', xRes, yRes)
            if abs(xRes) < 0.03 and abs(yRes) < 0.03:
                cv2.imwrite(imgName, result)
    result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('result', cv2.resize(result, None, fx=0.5, fy=0.5))

    poslat,poslon,Alt = get_GPSvalue(vehicle)
    yaw = Get_YAW(vehicle)

    want_go = 0.0001

    if yaw > 0 and yaw <= pi/2:
        n_go = yRes*cos(yaw) - xRes*sin(yaw)
        e_go = yRes*sin(yaw) + xRes*cos(yaw)

    elif yaw > pi/2 and yaw <= pi:
        n_go = xRes*sin(2*pi-yaw) + yRes*cos(2*pi - yaw)
        e_go = xRes*cos(2*pi-yaw) - yRes*sin(2*pi - yaw)

    elif yaw > pi and yaw <= 3*pi/2:
        n_go = xRes*sin(yaw - pi) - yRes*cos(yaw - pi)
        e_go = -xRes*cos(yaw - pi) - yRes*sin(yaw - pi)

    elif yaw > 3*pi/2 and yaw < 2*pi:
        n_go = -xRes*cos(yaw - pi/2) - yRes*sin(yaw - pi/2)
        e_go = yRes*cos(yaw - pi/2) - xRes*sin(yaw - pi/2)

    elif yaw == 0:
        n_go = yRes
        e_go = xRes

    if abs(n_go) < 0.0000001:
        n_go = 0

    if abs(e_go) < 0.0000001:
        e_go = 0

    n_go = want_go * n_go
    e_go = want_go * e_go

    if abs(xRes) > 0.03 and abs(yRes) > 0.03 and xRes != 100 and yRes != 100:
        if vehicle.mode.name == "AUTO" or vehicle.mode.name == "QHOVER":
            poslat,poslon,Alt = get_GPSvalue(vehicle)
            if Alt > 8:
                goto(vehicle, poslat+n_go , poslon+e_go,Alt-1)
            else :
                goto(vehicle, poslat+n_go , poslon+e_go,Alt)

    
    
    # Drop command 
    if abs(xRes) <= 0.03 and abs(yRes) <= 0.03 :
        Change_Alt(vehicle,8)
        drop(vehicle,10,2064)
        drop(vehicle,10,964)
        Change_Alt(vehicle,30)
        vehicle.mode = VehicleMode("RTL")
    #Manual Break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
cap.release()
vehicle.close()
