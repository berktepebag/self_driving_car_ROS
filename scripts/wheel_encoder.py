#!/usr/bin/python3
# Import required libraries
import time
import datetime
#import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO

import rospy
from std_msgs.msg import Float32
from self_driving_rc_car.msg import WheelEncoder

wheelEncoderMsg = WheelEncoder()

PIN_ID = 17

prev_time = time.time()
total_distance = 0
ms = 0.0
counter = 0

def wheel_enc_pub(pub,rate):
    global ms, counter

    while not rospy.is_shutdown():            
        # rospy.loginfo(ms)
        wheelEncoderMsg.velocity = ms
        wheelEncoderMsg.counter = counter
        wheelEncoderMsg.header.stamp = rospy.Time.now()
        pub.publish(wheelEncoderMsg)

        if(ms>0.005):
                ms -= ms/5
        counter += 1
        rate.sleep()

def sensorCallback(channel):
    global prev_time,total_distance,ms

    # Called if sensor output changes
    timestamp = time.time()
    stamp = datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S:%f')

    if GPIO.input(channel):
        # No magnet   
        GPIO.HIGH  
    else:  
        # Magnet
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


def main(): 
    
    rospy.init_node('wheel_encoder', anonymous=True)
    
    pub = rospy.Publisher('rc_car/wheel_encoder', WheelEncoder, queue_size=1)
    rate = rospy.Rate(50) # 100hz

    # Get initial reading
    sensorCallback(PIN_ID)

    try:
    # Loop until users quits with CTRL-C
        while (True) :
            # time.sleep(0.1)
            wheel_enc_pub(pub,rate)
            
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