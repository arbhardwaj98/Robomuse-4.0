cmake_minimum_required(VERSION 2.8.3)
project(phantomx_reactor_arm_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  arbotix_python
  dynamixel_controllers
  dynamixel_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  UpdatePos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/phantomx_reactor_parallel_motor_joints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
