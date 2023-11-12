# light activation for robotic competitions - they are activated when inferior ldr sensors detect the correct terrain reflection

import Jetson.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

GPIO.setwarnings(False)

GPIO.setup(35, GPIO.IN) # read ldr_e
GPIO.setup(37, GPIO.IN) # read ldr_d
GPIO.setup(11, GPIO.OUT) # activate led_superior

GPIO.output(11, GPIO.LOW)
sleep(1)
GPIO.output(11, GPIO.HIGH)
sleep(0.5)
GPIO.output(11, GPIO.LOW)
sleep(2)

try:
    while True:

        if (GPIO.input(37) == GPIO.HIGH):
            print ('D = ',1)
        else:
            print ('D = ',0)

        if (GPIO.input(35) == GPIO.HIGH):
            print ('E = ',1)
        else:
            print ('E = ',0)

        if (GPIO.input(37) == GPIO.HIGH) or (GPIO.input(35) == GPIO.HIGH):
            GPIO.output(11, GPIO.HIGH)
        else:
            GPIO.output(11, GPIO.LOW)

        sleep(0.1)

except KeyboardInterrupt:
    GPIO.output(11, GPIO.LOW)
    sleep(1)
    GPIO.cleanup()
    quit()
