cmake_minimum_required(VERSION 2.8.3)
project(robomuse_drivers)
find_package(catkin REQUIRED COMPONENTS roscpp rospy actionlib std_msgs actionlib_msgs genmsg urdf tf nav_msgs message_generation
             pluginlib
             cmake_modules
             geometric_shapes
	           sensor_msgs
             )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    interactive_markers
    actionlib_msgs
)
find_package(Eigen REQUIRED)

# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(dock_phase2 src/dock_phase2.cpp)
target_link_libraries(dock_phase2 ${catkin_LIBRARIES})

add_executable(dock_phase1 src/dock_phase1.cpp)
target_link_libraries(dock_phase1 ${catkin_LIBRARIES})

add_executable(sq src/sq.cpp)
target_link_libraries(sq ${catkin_LIBRARIES})

add_executable(startTracking.cpp src/startTracking.cpp)
target_link_libraries(startTracking.cpp ${catkin_LIBRARIES})

add_executable(movebasesimple.cpp src/movebasesimple.cpp)
target_link_libraries(movebasesimple.cpp ${catkin_LIBRARIES})

add_executable(waypointtest.cpp src/waypointtest.cpp)
target_link_libraries(waypointtest.cpp ${catkin_LIBRARIES})

add_executable(docknewphase1.cpp src/docknewphase1.cpp)
target_link_libraries(docknewphase1.cpp ${catkin_LIBRARIES})

add_executable(patrol.cpp src/patrol.cpp)
target_link_libraries(patrol.cpp ${catkin_LIBRARIES})

add_executable(armove.cpp src/armove.cpp)
target_link_libraries(armove.cpp ${catkin_LIBRARIES})

add_executable(ardockp2.cpp src/ardockp2.cpp)
target_link_libraries(ardockp2.cpp ${catkin_LIBRARIES})

add_executable(security.cpp src/security.cpp)
target_link_libraries(security.cpp ${catkin_LIBRARIES})

add_executable(qr_dist_expt.cpp src/qr_dist_expt.cpp)
target_link_libraries(qr_dist_expt.cpp ${catkin_LIBRARIES})

add_executable(ar_dist_expt.cpp src/ar_dist_expt.cpp)
target_link_libraries(ar_dist_expt.cpp ${catkin_LIBRARIES})

add_executable(markerlog.cpp src/markerlog.cpp)
target_link_libraries(markerlog.cpp ${catkin_LIBRARIES})

add_executable(kidnapsolve.cpp src/kidnapsolve.cpp)
target_link_libraries(kidnapsolve.cpp ${catkin_LIBRARIES})



