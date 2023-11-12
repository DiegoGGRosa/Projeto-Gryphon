#!/usr/bin/env python3
#PUMA++ AIM 2020, by Diego Rosa, PUC-Rio, Brazil

perda_torque = 1.16 + 0.6 # calibration parameters related to mass 
fator_multi = 1  # calibration parameters related to control

import time
import math
import serial
import threading
import numpy as np
from time import sleep
import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode

roboteqch1 = 0 #null initial velocity on frontal wheels
roboteqch2 = 0 #null initial velocity on rear wheels

ser = serial.Serial('/dev/ttyAMA0', 115200)
ser.isOpen()

connection_string = "/dev/ttyACM0"
baud_rate=115200

print("Connecting with the accelerometer")
vehicle = connect(connection_string, baud = baud_rate, wait_ready=True) 

#polynomials:

sigma = 19.74
mu = 25.363

pr1 = -0.85073
pr2 = 1.9995
pr3 = 1.563
pr4 = -7.3849
pr5 = 2.9688
pr6 = 7.9966
pr7 = -7.5192
pr8 = -1.9578
pr9 = 4.7125
pr10 = 0.93739
pr11 = 2.6807

pf1 = 0.61597
pf2 = -0.67635
pf3 = -2.2983
pf4 = 2.0795
pf5 = 3.0899
pf6 = -1.9326
pf7 = -1.7286
pf8 = 0.50886
pf9 = 0.08188
pf10 = -0.41006
pf11 = 1.1853

try:
    while True:

        vehicle.wait_ready('autopilot_version')
        #Yaw = np.degrees(vehicle.attitude.yaw)
        Pitch = np.degrees(vehicle.attitude.pitch)
        Roll = np.degrees(vehicle.attitude.roll)
        
        if Pitch<0:
            angle = 0
        elif Pitch> 56.7: 
            angle = 56.7
        else:
            angle = Pitch
                
        z = (angle-mu)/sigma
                
        Tr1 = pr1*(z**10) + pr2*(z**9) + pr3*(z**8) + pr4*(z**7) + pr5*(z**6) + pr6*(z**5) + pr7*(z**4) + pr8*(z**3) + pr9*(z**2) + pr10*z + pr11 + perda_torque
        Tf1 = pf1*(z**10) + pf2*(z**9) + pf3*(z**8) + pf4*(z**7) + pf5*(z**6) + pf6*(z**5) + pf7*(z**4) + pf8*(z**3) + pf9*(z**2) + pf10*z + pf11 + perda_torque

        
        roboteqch1 = round(fator_multi*69.35*Tf1)
        roboteqch2 = round(fator_multi*69.35*Tr1)
        print("Sending data to Controler")
        R1 = str(roboteqch1)
        R2 = str(roboteqch2)
        my_str1 = str.encode('!G 1 '+R1+'_')
        my_str2 = str.encode('!G 2 '+R2+'_')
        ser.write(my_str1)
        print(my_str1)
        ser.write(my_str2)
        print(my_str2)
        sleep(0.001)        
        
        #print('Yaw', Yaw)
        print('Pitch', Pitch)
        print('Roll', Roll)
        print('Torque rear wheels ', Tr1)
        print('Torque frontal wheels ', Tf1)
        print('Chanel1 ', roboteqch1)
        print('Chanel2 ', roboteqch2) 
        print('Last Heartbeat: %s'%vehicle.last_heartbeat)
        
        def attitude_callback(self, attr_name, value):

            print(vehicle.attitude)
            vehicle.add_attribute_listener('attitude', attitude_callback)
            time.sleep(0.001)
            vehicle.remove_attribute_listener('attitude', attitude_callback) 
            print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN'])
            
        data1 = Pitch
        data2 = Roll
        data3 = Tf1
        data4 = Tr1
        data5 = roboteqch1
        data6 = roboteqch2
        
        with open('aim2020_2.csv', 'a') as log:
            output = "{},{},{},{},{},{}\n".format(data1, data2, data3, data4, data5, data6)
            log.write(output)
        
            
except (KeyboardInterrupt,SystemExit):
    print('Process finished')
    vehicle.close()
    ser.write('!G 1 0_')
    ser.write('!G 2 0_')
    GPIO.cleanup()
    ser.close()
    print("END_OF_PATH")
    
