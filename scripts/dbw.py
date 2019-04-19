#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 

from self_driving_rc_car.msg import RcCarTeleop

import Adafruit_PCA9685

##min=10 max=100, for slow smooth driving 20~
forward_sensivity=50
backward_sensivity=50

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
#pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
##Sol Donus
servo_min = 300  # Min pulse length out of 4096
##Orta
servo_mid = 400  # Min pulse length out of 4096
##Sag Donus
servo_max = 500  # Max pulse length out of 4096

dc_max = 4000  # Max pulse length out of 4096

# Helper function to make setting a servo pulse width simpler.
# def set_servo_pulse(channel, pulse):
#     pulse_length = 1000000    # 1,000,000 us per second
#     pulse_length //= 60       # 60 Hz
#     print('{0}us per period'.format(pulse_length))
#     pulse_length //= 4096     # 12 bits of resolution
#     print('{0}us per bit'.format(pulse_length))
#     pulse *= 1000
#     pulse //= pulse_length
#     pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
#pwm.set_pwm_freq(60)

print('Moving servo on channel 0, press Ctrl-C to quit...')

def callback(twist_msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", twist_msg.linear.x)
    
def teleop_cmd_callback(teleop_msg):

    rospy.loginfo(rospy.get_caller_id() + "I heard servo: %s", teleop_msg.servo)

    # pwm.set_pwm(1,0,RcCarTeleop.servo)
    # pwm.set_pwm(0,0,RcCarTeleop.forward)
    # pwm.set_pwm(0,0,RcCarTeleop.reverse)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('dbw_listener', anonymous=True)
    rospy.Subscriber("rc_car/teleop_cmd", RcCarTeleop, teleop_cmd_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()