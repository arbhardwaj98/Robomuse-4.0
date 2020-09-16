cmake_minimum_required(VERSION 2.8.3)
project(robomuse_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch message_generation geometry_msgs message_runtime)


add_service_files(
  FILES
  MoveArm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package()

roslaunch_add_file_check(launch/ridgeback_world.launch)
roslaunch_add_file_check(launch/robomuse_world.launch)

install(DIRECTORY launch media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/move_arm_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
