cmake_minimum_required(VERSION 2.8.3)
project(self_driving_rc_car)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
#camera_calibration_parsers
cv_bridge
image_transport
message_generation
roscpp
rospy
sensor_msgs
std_msgs
std_srvs
stereo_msgs
rosbag
imu_bmi160
)

#FIND_LIBRARY(JETSONGPIO_LIBRARY JetsonGPIO /usr/local/lib/libJetsonGPIO.a)

## Generate messages in the 'msg' folder
add_message_files(
FILES
RcCarTeleop.msg
WheelEncoder.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
sensor_msgs
std_msgs
stereo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES self_driving_rc_car
CATKIN_DEPENDS 
#camera_calibration_parsers 
cv_bridge 
image_transport 
roscpp 
rospy 
sensor_msgs 
std_msgs 
std_srvs 
stereo_msgs 
message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
#target_link_libraries(self_driving_rc_car
#${catkin_LIBRARIES}
#)

#add_executable(transfer_learning_saver src/transfer_learning_saver.cpp)
#target_link_libraries(transfer_learning_saver 
#${catkin_LIBRARIES} 
#${OpenCV_LIBS} 
#)

add_executable(joystick_controller src/joystick_controller.cpp)
target_link_libraries(joystick_controller 
${catkin_LIBRARIES} 
)
add_dependencies(joystick_controller self_driving_rc_car_generate_messages_cpp)

add_executable(time_sync src/time_sync.cpp)
target_link_libraries(time_sync 
${catkin_LIBRARIES} 
${OpenCV_LIBS} 
)
add_dependencies(time_sync self_driving_rc_car_generate_messages_cpp)

add_executable(img_cmp src/image_compressor.cpp)
target_link_libraries(img_cmp 
${catkin_LIBRARIES} 
${OpenCV_LIBS} 
)
add_dependencies(img_cmp self_driving_rc_car_generate_messages_cpp)

add_executable(imu_bmi160_publisher src/imu_bmi160_publisher.cpp)
target_link_libraries(imu_bmi160_publisher 
${catkin_LIBRARIES} 
)

#add_executable(pose_estimator src/pose_estimator.cpp)
#target_link_libraries(pose_estimator 
#${catkin_LIBRARIES} 
#)
#add_dependencies(pose_estimator self_driving_rc_car_generate_messages_cpp)

add_executable(pose_estimator_sync_class src/pose_estimator_sync_class.cpp)
target_link_libraries(pose_estimator_sync_class 
${catkin_LIBRARIES} 
)
add_dependencies(pose_estimator self_driving_rc_car_generate_messages_cpp)