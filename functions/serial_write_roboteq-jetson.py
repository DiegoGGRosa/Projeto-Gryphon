# test serial write from Jetson to Roboteq -> Gryphon/PUMA+ differential or skid-steering drive system

# import libraries
import serial
from time import sleep

# valor -1000 up to + 1000 for min or max power (maximum velocity or maximum torque depending on previous roboteq config)
# direction for linear or angular direciontos depends on electrical connections and previous calibration

valor1 = 300 # differential simples, velocity - positive value indicates that the robot moves forwards and negative the opposite
valor2 = 0 # differential simples, velocity - positive value indicates go to left and negative goes to right

# velocities
v1 = str(valor1)
v2 = str(valor2)

# write to roboteq controller 
# roboteq already programmed for velocity or torque 
my_str1 = str.encode('!G 1 '+v1+'_')
my_str2 = str.encode('!G 2 '+v2+'_')

ser = serial.Serial('/dev/ttyS0', 115200) # not used - timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

ser.isOpen()

sleep(0.1) # time interval to any other trajectories - maybe reduced intervals guarantee better performance (test and calibrate)

try:
        while True:
            
                ser.write(my_str1)
#                print(my_str1) 
                ser.write(my_str2)
#                print(my_str2)
                sleep(5)

except KeyboardInterrupt: # other interrupts for general failures
    ser.write('!G 1 0_')
