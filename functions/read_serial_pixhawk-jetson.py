# read pixhawk fused roll pitch yaw angles from Nvidia Jetson

# import libraries
from dronekit import connect, VehicleMode
import time
import numpy as np
import math

# set serial connection 
connection_string = "/dev/ttyACM0"
baud_rate=115200

# connecting (processo automatico via bilioteca drone kit)

print(">>>> Connecting with the the robot <<<")
vehicle = connect(connection_string, baud = baud_rate, wait_ready=True) 

try:
    while True:

        vehicle.wait_ready('autopilot_version')
#       print('Autopilot version: %s'%vehicle.version)
#       print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)
#       print('Local position: %s'% vehicle.location.global_relative_frame)

        Yaw = np.degrees(vehicle.attitude.yaw)
        Pitch = np.degrees(vehicle.attitude.pitch)
        Roll = np.degrees(vehicle.attitude.roll)

        print('Yaw', Yaw)
        print('Pitch', Pitch)
        print('Roll', Roll) 
        time.sleep(0.1) 


        data1 = Roll
        data2 = Pitch
        data3 = Yaw


# do not change: 

#       print('Velocity: %s'%vehicle.velocity) #- North, east, down
#       print('Last Heartbeat: %s'%vehicle.last_heartbeat)
#       print('Is the vehicle armable: %s'%vehicle.is_armable)
        #- Which is the total ground speed? Note: this is settable
#       print('Groundspeed: %s'% vehicle.groundspeed) #(%)
        #- What is the actual flight mode? Note: this is settable
#       print('Mode: %s'% vehicle.mode.name)
        #- Is the vehicle armed Note: this is settable
#       print('Armed: %s'%vehicle.armed)
#       print('EKF Ok: %s'%vehicle.ekf_ok)

        def attitude_callback(self, attr_name, value):

            print(vehicle.attitude)
#           print("")
#           print("Adding an attitude listener")
            vehicle.add_attribute_listener('attitude', attitude_callback) #-- message type, callback function
            time.sleep(1)
            #--- Now we print the attitude from the callback for 5 seconds, then we remove the callback
            vehicle.remove_attribute_listener('attitude', attitude_callback) #(.remove)
            #--- You can create a callback even with decorators, check the documentation out for more details
            #---- PARAMETERS
#           print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN'])
            #-- You can read and write the parameters
#           vehicle.parameters['THR_MIN'] = 50
#           time.sleep(1)
#           print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN'])

# save data:
        with open('angles.csv','a') as log:
                output = "{},{},{}\n".format(data1, data2, data3)
                log.write(output)

except KeyboardInterrupt:
    print('Interrompido pelo usuario')
    vehicle.close()

    print("done")
