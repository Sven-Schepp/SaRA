cmake_minimum_required(VERSION 2.8.3)
project(reachable_occupancy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  urdf
  message_generation
#  Threads
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   time.msg
   Capsule.msg
   CapsuleArray.msg
   Cylinder.msg
   CylinderArray.msg
   JointLocations.msg
   ReachableOccupancy.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reachable_occupancy
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf urdf message_runtime
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(SARA_INCLUDE_DIRS "include/ReachLib/include")

include_directories(
  include
  include/reachable_occupancy
  ${catkin_INCLUDE_DIRS}
  ${SARA_INCLUDE_DIRS}
)


set(SARA_SRC_PATH "include/ReachLib/src")

set(SARA_SOURCES
  ${SARA_SRC_PATH}/articulated.cpp
  ${SARA_SRC_PATH}/articulated_accel.cpp
  ${SARA_SRC_PATH}/articulated_vel.cpp
  ${SARA_SRC_PATH}/articulated_pos.cpp
  ${SARA_SRC_PATH}/body_part.cpp
  ${SARA_SRC_PATH}/body_part_accel.cpp
  ${SARA_SRC_PATH}/body_part_vel.cpp
  ${SARA_SRC_PATH}/capsule.cpp
  ${SARA_SRC_PATH}/cylinder.cpp
  ${SARA_SRC_PATH}/cylinder_perimeter.cpp
  ${SARA_SRC_PATH}/extremity.cpp
  ${SARA_SRC_PATH}/occupancy.cpp
  ${SARA_SRC_PATH}/pedestrian.cpp
  ${SARA_SRC_PATH}/pedestrian_accel.cpp
  ${SARA_SRC_PATH}/pedestrian_vel.cpp
  ${SARA_SRC_PATH}/point.cpp
  ${SARA_SRC_PATH}/system.cpp
  ${SARA_SRC_PATH}/sphere.cpp
)

add_library(SaRA ${SARA_SOURCES})
#set_target_properties(ReachRI PROPERTIES IMPORTED_LOCATION src/${PROJECT_NAME}/include/reach_lib/build/libReachRI.a)    ###########################

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/reachable_occupancy.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/reachable_occupancy_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_reachable_occupancy.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(reachable_ocpc src/scripts/reachable_ocpc.cpp)
#target_link_libraries(reachable_ocpc ${catkin_LIBRARIES})
#target_link_libraries(reachable_ocpc ReachRI)
#add_dependencies(reachable_ocpc reachable_occupancy_generate_messages_cpp)


# Adding the publisher demo

#add_executable(udp_translator src/udp_translator.cpp)

#target_link_libraries(udp_translator ${catkin_LIBRARIES})
#target_link_libraries(udp_translator SaRA)
#add_dependencies(udp_translator udp_translator_generate_messages_cpp)


# Adding the udp_receiver
#add_executable(udp_receiver src/udp_receiver.cpp)

#target_link_libraries(udp_receiver ${catkin_LIBRARIES})
#target_link_libraries(udp_receiver ReachRI)
#add_dependencies(udp_receiver udp_receiver_generate_messages_cpp)

# Adding main reachable occupancy executable dependent on udp_com
#add_executable(reachable_occupancy src/reachable_occupancy.cpp)

#target_link_libraries(reachable_occupancy ${catkin_LIBRARIES})
#target_link_libraries(reachable_occupancy SaRA)
#add_dependencies(reachable_occupancy reachable_occupancy_generate_messages_cpp)

# Adding main reachable occupancy executable without dependecy on udp_com
add_executable(reachable_occupancy src/reachable_occupancy_no_udp.cpp)

target_link_libraries(reachable_occupancy ${catkin_LIBRARIES})
target_link_libraries(reachable_occupancy SaRA)
add_dependencies(reachable_occupancy reachable_occupancy_generate_messages_cpp)
