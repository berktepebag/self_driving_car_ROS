#!/usr/bin/python3
# Import required libraries
import time
import datetime
#import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO

import rospy
from std_msgs.msg import Float32

PIN_ID = 17

prev_time = time.time()
total_distance = 0
ms = 0.0

def wheel_enc_pub():
    global ms

    pub = rospy.Publisher('rc_car/wheel_encoder', Float32, queue_size=1)
    rospy.init_node('wheel_encoder', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():            
        # rospy.loginfo(ms)
        if(ms<0.1):
            ms = 0
        pub.publish(ms)
        rate.sleep()

def sensorCallback(channel):
    global prev_time,total_distance,ms

    # Called if sensor output changes
    timestamp = time.time()
    stamp = datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S:%f')

    if GPIO.input(channel):
        # No magnet   
        GPIO.HIGH    
        if(ms>0.1):
            # rospy.loginfo("decreasing... %.2f",ms)
            ms = ms - 0.5
    else:  
        # Magnet
        time_diff = timestamp - prev_time
        prev_time = timestamp
        distance = 0.033 #m 0.308*1.06
        ms = distance/time_diff
        kmh = ms*3.6
        str = "%.2f"% (kmh)
        str2 = "%.2f"% (ms)
        # print(str + " km/h **** " + str2 + " m/s")
    
        total_distance = total_distance + distance
        # print("total distance %.2f"% (total_distance))


def main(): 
    # Get initial reading
    sensorCallback(PIN_ID)

    try:
    # Loop until users quits with CTRL-C
        while (True) :
            # time.sleep(0.1)
            wheel_enc_pub()
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
GPIO.setup(PIN_ID, GPIO.IN)
GPIO.add_event_detect(PIN_ID, GPIO.BOTH, callback=sensorCallback, bouncetime=1)

if __name__ == '__main__':
    main()