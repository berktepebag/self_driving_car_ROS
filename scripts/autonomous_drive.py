#!/usr/bin/env python

import h5py
#Tensorflow & Keras
import tensorflow as tf
# from tensorflow.keras.models import load_model #@desktop
from keras.models import load_model  # @jetson_nano

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Ros libraries
import roslib
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage

# Teleop Message
from self_driving_rc_car.msg import RcCarTeleop

VERBOSE = False

global graph, model, sess
graph = tf.get_default_graph()

flags = tf.app.flags
FLAGS = flags.FLAGS

flags.DEFINE_string('model_folder', '/home/berk/catkin_ws/src/self_driving_car_ROS/weights/', "Path to model folder.")
flags.DEFINE_string('model_name', 'w_b32_e50.h5', "Name of the model file.")
flags.DEFINE_bool('from_rosbag','false','Images from rosbag?')

#Check whether model is listening to topics from rosbag or jetbot_camera
if(FLAGS.from_rosbag == True):
    subscribed_topic = 'sync/resized_image/compressed'
else:
    subscribed_topic = '/rc_car/image_color/compressed'

model_path = FLAGS.model_folder + FLAGS.model_name
print("---->Model Path: {}",model_path)

# No module named tkinter -> https://askubuntu.com/questions/815874/importerror-no-named-tkinter-please-install-the-python3-tk-package
model = load_model(model_path)

teleop_msg = RcCarTeleop()
teleop_pub = rospy.Publisher("rc_car/teleop_cmd", RcCarTeleop, queue_size=1)

scale_factor = 0.5

class image_feature:

    def __init__(self):

        # subscribed Topic
        self.subscriber = rospy.Subscriber(subscribed_topic,
                                           CompressedImage, self.callback,  queue_size=1)
        if VERBOSE:
            print("subscribed to ", subscribed_topic)

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        # scale_percent = 100  # percent of original size
        width = int(image_np.shape[1] * scale_factor)
        height = int(image_np.shape[0] * scale_factor)
        dim = (width, height)
        # print("dim w: %f h: %f ",width,height)

        resized = cv2.resize(image_np, dim, interpolation=cv2.INTER_AREA)
        # cropped = resized[20:180,]

        # result = np.array(4)

        with graph.as_default():
            result = model.predict(resized[None, :, :, :], batch_size=1)
            teleop_msg.servo = result[0]

        teleop_msg.servo = result[0][0]
        # if(result[0][1]>0):
        #     fwd = result[0][1] / 2
        # else:
        #     fwd = result[0][1] * 2
        fwd = result[0][0]
        teleop_msg.forward = fwd
        teleop_msg.reverse = result[0][2]

        teleop_pub.publish(teleop_msg)

        print("steering_angle: ", result[0])


def main():

    # rospy.init_node("autonomous_drive", anonymous=True)

    ic = image_feature()
    rospy.init_node('img_to_steering', anonymous=True)

    rospy.spin()


if __name__ == '__main__':

    main()
