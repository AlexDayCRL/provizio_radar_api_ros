cmake_minimum_required(VERSION 3.0.2)

# Include support for ExternalProject_Add
include(ExternalProject)

project(provizio_radar_api_ros)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

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
#   std_msgs
# )

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
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

# Provizio Core Radar API
set(PROVIZIO_RADAR_CORE_API_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_radar_api_core_build")
set(PROVIZIO_RADAR_CORE_API_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_radar_api_core")
set(PROVIZIO_RADAR_CORE_API_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT "provizio/provizio_radar_api_core") # Or your own fork
set(PROVIZIO_RADAR_CORE_API_GITHUB_BRANCH "v22.11.17") # Or a specific tag you'd like to use
set(PROVIZIO_RADAR_CORE_API_INSTALL_DIR "${PROVIZIO_RADAR_CORE_API_BINARY_DIR}/install")
ExternalProject_Add(libprovizio_radar_api_core
  GIT_REPOSITORY "https://github.com/${PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT}.git" # or "git@github.com:${PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT}.git" for ssh access, or your custom repo
  GIT_TAG "${PROVIZIO_RADAR_CORE_API_GITHUB_BRANCH}"
  PREFIX "${PROVIZIO_RADAR_CORE_API_PREFIX}"
  SOURCE_DIR "${PROVIZIO_RADAR_CORE_API_SOURCE_DIR}"
  BINARY_DIR "${PROVIZIO_RADAR_CORE_API_BINARY_DIR}"
  CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}" "-DENABLE_CHECK_FORMAT=OFF" "-DBUILD_TESTING=OFF"
)
link_directories("${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/lib")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/provizio_radar_api_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  libprovizio_radar_api_core # Provizio Core API
)

# Provizio Core API headers
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC "${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/include")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  provizio_radar_api_core
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_provizio_radar_api_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
