from dronekit import connect, VehicleMode
import time
import dronekit_sitl

connection_string = "/dev/ttyACM0"
baud_rate = 115200

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

posdata = str(vehicle.location.global_relative_frame).split(':')
poslat, poslon, alt = posdata[1].split(',')
#- Read the actual position
print('Position: '+str(poslat)+' '+str(poslon))

print('Altitude:'+str(alt))

vehicle.close()
print("done")
