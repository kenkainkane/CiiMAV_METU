import dronekit_sitl
from vision_lib import *
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from mission_lib import *


print(">>>>>>>>>>>>>>>>>>>>> Connecting to UAV <<<<<<<<<<<<<<<<<<<")
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')
print(">>>>>>>>>>>>>>>> UAV has been connected <<<<<<<<<<<<<<<<<<")
poslat, poslon, Alt = get_GPSvalue(vehicle)
