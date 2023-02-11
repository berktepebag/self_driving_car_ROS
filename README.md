### *** Archieved: As this repo is and will not updated anymore, it is archieved. ***

With Jetson nano somethings has changed.

Jetson nano comes with Ubuntu 18.04 and ROS Melodic. As far as I could understand Melodic's Python is 3.5+. So PCA9685 is not working directly as it was working in Raspberry Pi. So things to do and links to follow:

1- Do the starting tutorial from Adrian's tutorial: https://www.pyimagesearch.com/2019/05/06/getting-started-with-the-nvidia-jetson-nano/

2- Also you may want to follow Fei Chung's tutorial of Jetson nano for Donkey car: https://medium.com/@feicheung2016/getting-started-with-jetson-nano-and-autonomous-donkey-car-d4f25bbd1c83

Note: As I remember scipy installation from Adrian's tutorial was failing. Fei also mentioned this in his tutorial. So do not waste time trying to install scipy from Adrian's tutorial ;)

3- If you don't want to change from Python 2 to 3, create virtualenv with Python 2 (if you followed Adrian's tutorial you already created a virtualenv.)

4- If you are getting "RuntimeError: Could not determine default I2C bus for platform." error follow Kokensha's tutorial: https://kokensha.xyz/jetson/jetson-nano-pca9685-i2c-error-resolution/ . It is some language but I think you can just get the idea ;) Important thing is, since we are using virtualenv, "~/.local/lib/python3.6/site-packages/Adafruit_GPIO/I2C.py" is located under "~/.virtualenvs/your_virtual_env_name/lib/python2.7/site-packages/Adafruit_GPIO/". Follow the tutorial and put "1" instead of busnum. You have to do this for both Python 2 and 3 if you want to use PCA9685 with both in their relative virtualenvs'.

5- Since we created a Python 2 environment instead of "pip3 install Adafruit_PCA9685 --user", use "pip install Adafruit_PCA9685" in your virtualenv.

6- I think since we are in virtualenv (or maybe not I'm not sure) I had "ImportError: No module named rospkg" error. Solved by "pip install rospkg" in virtualenv. Becareful since we want to keep python 2 NOT pip3 but pip. (http://answers.ros.org/question/39657/importerror-no-module-named-rospkg/)

7-I used at Raspberry with ROS Kinetic Ubiquity Robotics, raspicam_node (https://github.com/UbiquityRobotics/raspicam_node). It was very useful to publish Raspberry pi camera images to topics with just a roslaunch so I could save images from other computer. But seems like it is only for Kinetic so either we will wait for them or we will go ROS's own image topics to share camera images.

8-While trying to install joy node (joystick controller for ROS) I came up with no server error. Seems like ROS.org had some security problems so they changed keys. Follow: (http://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/). I am not sure if this fixed my problem. After following it, I still was not able to install it. I believe after restart it started working. Sadly I cannot say what was preventing it... But at least you know one of the solutions now.

9-Sublime Text is not working on ARM CPU's sadly. So we will install Visual Studio Code. Follow: (https://devtalk.nvidia.com/default/topic/1049448/quick-build-guide-for-visual-studio-code-on-the-nano/) 

Well if you have done all you should be able to catkin_make and roslaunch img_run.launch and dbw.py. I am keeping them separately for now but when it is working properly I will put them in one launch file.

Now it is time to start getting images from camera so we can create rosbag's in order to traing our CNN.
