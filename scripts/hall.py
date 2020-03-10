#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____
#   / _ \/ _ \(_) __/__  __ __
#  / , _/ ___/ /\ \/ _ \/ // /
# /_/|_/_/  /_/___/ .__/\_, /
#                /_/   /___/
#
#       Hall Effect Sensor
#
# This script tests the sensor on GPIO17.
#
# Author : Matt Hawkins
# Date   : 08/05/2018
#
# https://www.raspberrypi-spy.co.uk/
#
#--------------------------------------

# Import required libraries
import time
import datetime
#import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO

PIN_ID = 17

prev_time = time.time()
total_distance = 0
counter = 0

def sensorCallback(channel):
  global prev_time,total_distance,counter

  # Called if sensor output changes
  timestamp = time.time()
  stamp = datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S:%f')
  if GPIO.input(channel):
    # No magnet
    #print("Sensor HIGH " + stamp)	
    GPIO.HIGH
  else:
    # Magnet
    #print("Sensor LOW " + stamp)
    time_diff = timestamp - prev_time
    prev_time = timestamp
    distance = 0.033 #m 0.308*1.06
    ms = distance/time_diff
    kmh = ms*3.6
    str = "%.2f"% (kmh)
    str2 = "%.2f"% (ms)
    print(str + " km/h **** " + str2 + " m/s")
   
    total_distance = total_distance + distance
    print("total distance %.2f"% (total_distance))
   # counter = counter + 1
   # print("counter: ", counter)
    

def main():
  # Wrap main content in a try block so we can
  # catch the user pressing CTRL-C and run the
  # GPIO cleanup function. This will also prevent
  # the user seeing lots of unnecessary error
  # messages.

  # Get initial reading
  sensorCallback(PIN_ID)

  try:
    # Loop until users quits with CTRL-C
    while True :
      time.sleep(0.1)

  except KeyboardInterrupt:
    # Reset GPIO settings
    GPIO.cleanup()

# Tell GPIO library to use GPIO references
GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

print("Setup GPIO pin as input on GPIO17")
print("Mode: ",GPIO.getmode())
# Set Switch GPIO as input
# Pull high by default
#GPIO.setup(17 , GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ID, GPIO.IN)
GPIO.add_event_detect(PIN_ID, GPIO.BOTH, callback=sensorCallback, bouncetime=1)
#GPIO.add_event_detect(PIN_ID, GPIO.FALLING, callback=sensorCallback, bouncetime=200)

if __name__=="__main__":
   main()
