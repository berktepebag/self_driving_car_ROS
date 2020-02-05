#!/usr/bin/env python

import h5py
#Tensorflow & Keras
import tensorflow as tf
# from tensorflow.keras.models import load_model #@desktop
from keras.models import load_model #@jetson_nano

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Ros libraries
import roslib
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage

#Teleop Message
from self_driving_rc_car.msg import RcCarTeleop

VERBOSE=False

global graph,model,sess
graph = tf.get_default_graph()

#No module named tkinter -> https://askubuntu.com/questions/815874/importerror-no-named-tkinter-please-install-the-python3-tk-package
model = load_model('model.h5')

teleop_msg = RcCarTeleop()
teleop_pub = rospy.Publisher("rc_car/teleop_cmd", RcCarTeleop, queue_size=1)

class image_feature:

    def __init__(self):
        
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/image_color/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print ("subscribed to /sync/resized_image/compressed")


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        scale_percent = 25 # percent of original size
        width = int(image_np.shape[1] * scale_percent/100)
        height = int(image_np.shape[0] * scale_percent/100)
        dim = (width, height)
        # print("dim w: %f h: %f ",width,height)

        resized = cv2.resize(image_np,dim,interpolation = cv2.INTER_AREA)
        cropped = resized[20:180,]      

        with graph.as_default():
            steering_angle = float(model.predict(cropped[None, :, :, :], batch_size=1))
            teleop_msg.servo = steering_angle * 1.1
        
        teleop_msg.forward = 0.25
        teleop_msg.reverse = 1.0

        teleop_pub.publish(teleop_msg)

        # print("steering_angle: ",steering_angle)



def main():	

    
    # rospy.init_node("autonomous_drive", anonymous=True)
    
    ic = image_feature()
    rospy.init_node('img_to_steering', anonymous=True)
    
    rospy.spin()


if __name__ == '__main__':

	main()




