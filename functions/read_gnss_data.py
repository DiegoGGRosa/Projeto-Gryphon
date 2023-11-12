import os
from gps import *
from time import *
import time
import threading

gpsd = None #seting the global variable
 
os.system('clear') #clear the terminal (optional)
 
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
 
if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread
  try:
    gpsp.start() # start it up
    while True:
      os.system('clear')
      
      LA = gpsd.fix.latitude
      LO = gpsd.fix.longitude
      TIME = gpsd.utc
      ALT = gpsd.fix.altitude
      SPEED = gpsd.fix.speed
      TRACK = gpsd.fix.track
      SAT = len(gpsd.satellites)
      MODE = gpsd.fix.mode
      
      print LA
      print LO
      print TIME
      print ALT
      print SPEED
      print TRACK
      print SAT
      print MODE
      time.sleep(0.01) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "Killing Process"
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print "Done"
