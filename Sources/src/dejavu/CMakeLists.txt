cmake_minimum_required(VERSION 2.8.3)
project(dejavu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  clarus
  cv_video
  terminal_io
  yamabros
)

#find_package(Boost REQUIRED COMPONENTS filesystem system)
#find_package(OpenCV 2.4.10 REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dejavu
#  CATKIN_DEPENDS cight clarus
#  DEPENDS OpenCV boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# TODO: Check names of system library include directories (OpenCV, boost)
include_directories(
  ${catkin_INCLUDE_DIRS}
  "include/"
)

## Declare a cpp library
# add_library(dejavu
#   src/${PROJECT_NAME}/dejavu.cpp
# )

## Declare a cpp executable
# add_executable(dejavu_node src/dejavu_node.cpp)

add_library(dejavu
  "src/averager.cpp"
  "src/difference_stream.cpp"
  "src/differentiator.cpp"
  "src/dispatcher.cpp"
  "src/feature_point.cpp"
  "src/feature_selector.cpp"
  "src/file_stream.cpp"
  "src/filters.cpp"
  "src/interpolator.cpp"
  "src/settings.cpp"
  "src/shift_reduce.cpp"
)

add_executable(difference_camera
  "src/difference_camera.cpp"
)

add_executable(difference_matcher
  "src/difference_matcher.cpp"
)

add_executable(experiment_differences
  "src/experiment_differences.cpp"
)

add_executable(experiment_inputs
  "src/experiment_inputs.cpp"
)

add_executable(experiment_spectra
  "src/experiment_spectra.cpp"
)

add_executable(ground_truth
  "src/ground_truth.cpp"
)

add_executable(inspector_matching
  "src/inspector_matching.cpp"
)

add_executable(inspector_shifts
  "src/inspector_shifts.cpp"
)

#add_executable(hrr
#  "src/node/hrr.cpp"
#)

#add_executable(replay
#  "src/node/replay.cpp"
#)

#add_executable(saliences
#  "src/node/saliences.cpp"
#)

add_executable(shift_estimator
  "src/shift_estimator.cpp"
)

add_executable(steering
  "src/steering.cpp"
)

#add_executable(teach
#  "src/node/teach.cpp"
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(dejavu_node dejavu_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(dejavu_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${boost_LIBRARIES}
# )

target_link_libraries(dejavu
  ${catkin_LIBRARIES}
)

target_link_libraries(difference_camera
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(difference_matcher
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(experiment_differences
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(experiment_inputs
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(experiment_spectra
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(ground_truth
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(inspector_matching
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(inspector_shifts
  ${catkin_LIBRARIES}
  dejavu
)

#target_link_libraries(hrr
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(replay
#  ${catkin_LIBRARIES}
#)

target_link_libraries(shift_estimator
  ${catkin_LIBRARIES}
  dejavu
)

target_link_libraries(steering
  ${catkin_LIBRARIES}
  dejavu
)

#target_link_libraries(saliences
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(teach
#  ${catkin_LIBRARIES}
#)

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
# install(TARGETS dejavu dejavu_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dejavu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
