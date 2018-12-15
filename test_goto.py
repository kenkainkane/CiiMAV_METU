from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit_sitl
from serial import *
import time

#print('Starting Test GOTO...')
"""
# DroneKit Variables
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')
"""
def Armed(vehicle):
	print('........Check Armed........')
	while True:
    print(">>>> Change Mode to 'GUIDED' <<<<")
		vehicle.mode = VehicleMode("GUIDED")
		
    print(">>>> ARMED <<<<")
		vehicle.armed = True

		if vehicle.armed:
			print(">>>> Already Armed <<<<")
      break
		else:
			continue
    time.sleep(2)

def Takeoff(vehicle,alt):
  print('>>>>>>>> Take off <<<<<<<<<')
	vehicle.simple_takeoff(alt)
	time.sleep(3)
	print(">>>> Change mode to 'LOITER' <<<<")
	vehicle.mode = VehicleMode("LOITER")
	print('>>>> Complete Takeoff <<<<')

def Change_Alt(vehicle,alt):
	posdata = str(vehicle.location.global_relative_frame).split(':')
	poslat, poslon, Alt = posdata[1].split(',')
	lat = float(str(poslat)[4:])
  lon = float(str(poslon)[4:])
  print(">>>> Change mode to 'GUIDED' <<<<")
  vehicle.mode = VehicleMode("GUIDED")
  print(">>>> Going to ALT:"+str(alt)+" <<<<")
  a_location = LocationGlobalRelative(lat,lon,alt)
  vehicle.simple_goto(a_location)
  time.sleep(3)
  print(">>>> Change mode to 'QLOITER' <<<<")
  vehicle.mode = VehicleMode("QLOITER")
  print(">>>> Complete Change ALT <<<<")


def goto(vehicle,lat,lon,alt):
  print(">>>> Change mode to 'GUIDED' <<<<")
  vehicle.mode = VehicleMode("GUIDED")
  a_location = LocationGlobalRelative(lat,lon,alt)
  print('>>>>>>> Going to  '+str(lat)+','+str(lon)+','+str(alt)+' <<<<<<<<<<')
  vehicle.simple_goto(a_location)
  time.sleep(3)
  print(">>> Change mode to 'LOITER' <<<<")
  vehicle.mode = VehicleMode("LOITER")
  print('>>>> Complete GOTO <<<<')

def land(vehicle):
  print(">>>> LANDING <<<<")
  vehicle.mode = VehicleMode("LAND")
  print(">>>> Landing Complete <<<<")

def get_GPSvalue(vehicle):
	posdata = str(vehicle.location.global_relative_frame).split(':')
  poslat, poslon, Alt = posdata[1].split(',')
  lat = float(str(poslat)[4:])
  lon = float(str(poslon)[4:])
	alt = float(str(Alt)[4:])
  return [lat,lon,alt]
