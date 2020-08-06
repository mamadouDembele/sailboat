cmake_minimum_required(VERSION 2.8.3)
project(sailboat)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  tf
  tf2
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})

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
#add_message_files(
	#FILES
	#State_sail.msg
	#Input_sail.msg
#)


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
#generate_messages(
 #  DEPENDENCIES
  # std_msgs
#)

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
  #INCLUDE_DIRS include
  #LIBRARIES Fonction
  #CATKIN_DEPENDS geometry_msgs message_runtime roscpp rospy std_msgs tf
  #DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include 
	${catkin_INCLUDE_DIRS})
	#${Eigen3_INCLUDE_DIRS} )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sailboat.cpp

#add_library(Fonction src/Fonction.cpp)
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/sailboat_node.cpp)

add_executable(simu_sailboat src/simulation/simu_sailboat.cpp)
add_executable(drawwaypoint src/simulation/drawwaypoint.cpp)
add_executable(heading_follow src/simulation/heading_follow.cpp)
add_executable(line_follow src/simulation/line_follow.cpp)
add_executable(station_keeping src/simulation/station_keeping.cpp)
add_executable(circle src/simulation/circle.cpp)
add_executable(circle2 src/simulation/circle2.cpp)
add_executable(triangle src/simulation/triangle.cpp)
add_executable(avoidance src/simulation/avoidance.cpp)
add_executable(figure_sk src/simulation/figure_sk.cpp)
add_executable(figure_avoid src/simulation/figure_avoid.cpp)
add_executable(figure_triangle src/simulation/figure_triangle.cpp)

#executable for the differentes challenges
add_executable(fleet_race src/mission/fleet_race.cpp)
add_executable(stat_keeping_mission src/mission/stat_keeping_mission.cpp)
add_executable(stat_mission2_version2 src/mission/stat_mission2_version2.cpp)
add_executable(hide_seek src/mission/hide_seek.cpp)
add_executable(challenge1_figure src/mission/challenge1_figure.cpp)
add_executable(challenge2_figure src/mission/challenge2_figure.cpp)
add_executable(area_scanning_figure src/mission/area_scanning_figure.cpp)
add_executable(boats_area_scanning src/mission/boats_area_scanning.cpp)
add_executable(area_scanning_challenge src/mission/area_scanning_challenge.cpp)
#add_executable(qtable /src/mission/qtable)

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

#target_link_libraries(Fonction ${catkin_LIBRARIES})
target_link_libraries(simu_sailboat ${catkin_LIBRARIES})
target_link_libraries(drawwaypoint ${catkin_LIBRARIES})
target_link_libraries(heading_follow ${catkin_LIBRARIES})
target_link_libraries(line_follow ${catkin_LIBRARIES})
target_link_libraries(station_keeping ${catkin_LIBRARIES})
target_link_libraries(circle ${catkin_LIBRARIES})
target_link_libraries(circle2 ${catkin_LIBRARIES})
target_link_libraries(triangle ${catkin_LIBRARIES})
target_link_libraries(avoidance ${catkin_LIBRARIES})
target_link_libraries(figure_sk ${catkin_LIBRARIES})
target_link_libraries(figure_avoid ${catkin_LIBRARIES})
target_link_libraries(figure_triangle ${catkin_LIBRARIES})

#for the challenge
target_link_libraries(fleet_race ${catkin_LIBRARIES})
target_link_libraries(stat_keeping_mission ${catkin_LIBRARIES})
target_link_libraries(stat_mission2_version2 ${catkin_LIBRARIES})
target_link_libraries(hide_seek ${catkin_LIBRARIES})
target_link_libraries(challenge1_figure ${catkin_LIBRARIES})
target_link_libraries(challenge2_figure ${catkin_LIBRARIES})
target_link_libraries(area_scanning_figure ${catkin_LIBRARIES})
target_link_libraries(boats_area_scanning ${catkin_LIBRARIES})
target_link_libraries(area_scanning_challenge ${catkin_LIBRARIES})
#target_link_libraries(qtable ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(
#   TARGETS Fonction
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(
#   DIRECTORY include/Fonction/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
   #PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#	qtable
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sailboat.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
