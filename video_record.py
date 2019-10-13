import cv2
import numpy as np
import time 
from dronekit import connect, VehicleMode
import dronekit_sitl

connection_string = "/dev/ttyACM0"
baud_rate = 115200
print(">>>> Connecting with the UAV <<<<")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')
print('ready')

cap = cv2.VideoCapture(0)
 
if (cap.isOpened() == False): 
    print("Unable to read camera feed")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
t = str(time.time())
out = cv2.VideoWriter(t+'.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
 
while(True):
    posdata = str(vehicle.location.global_relative_frame).split(':')
    _, _, alt = posdata[1].split(',')
    ret, frame = cap.read()
    cv2.putText(frame, str(alt),(0,int(frame_height/2.1)),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
    if ret == True: 
        out.write(frame)
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break 
cap.release()
out.release()
cv2.destroyAllWindows() 
