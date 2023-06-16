cmake_minimum_required(VERSION 3.1.2)
project(mrs_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  cmake_modules
  rviz
  mrs_msgs
  nav_msgs
  mrs_lib
  geometry_msgs
  )

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)
# ignore c++17 deprecation warnings for the register storage class specifier because of OGRE
add_definitions("-Wno-register")

set(LIBRARIES
  MrsBumperRviz MrsCovarianceRviz MrsRvizNavGoal MrsRvizPoseEstimate MrsSphereRviz MrsPoseWithCovarianceArrayRviz MrsTrackArrayRviz MrsNamedGoalTool
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp mrs_msgs rviz mrs_lib nav_msgs geometry_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## BUMPER VIZUALIZATION

add_library(MrsBumperRviz
  include/bumper/display.h
  include/bumper/visual.h
  src/bumper/display.cpp
  src/bumper/visual.cpp
  )

add_dependencies(MrsBumperRviz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsBumperRviz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## SPHERE VIZUALIZATION

add_library(MrsSphereRviz
  include/sphere/display.h
  include/sphere/visual.h
  src/sphere/display.cpp
  src/sphere/visual.cpp
  )

add_dependencies(MrsSphereRviz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsSphereRviz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## COVARIANCE

add_library(MrsCovarianceRviz
  include/covariance/property.h
  include/covariance/visual.h
  src/covariance/property.cpp
  src/covariance/visual.cpp
  )

add_dependencies(MrsCovarianceRviz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsCovarianceRviz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## POSE WITH COVARIANCE ARRAY

add_library(MrsPoseWithCovarianceArrayRviz
  include/pose_with_covariance_array/display.h
  src/pose_with_covariance_array/display.cpp
  )

add_dependencies(MrsPoseWithCovarianceArrayRviz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsPoseWithCovarianceArrayRviz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  MrsCovarianceRviz
  )

## RVIZ NAV GOAL

add_library(MrsRvizNavGoal
  src/rviz_interface/nav_goal.cpp
  )

target_link_libraries(MrsRvizNavGoal
  ${catkin_LIBRARIES}
  )

## RVIZ POSE ESTIMATE

add_library(MrsRvizPoseEstimate
  src/rviz_interface/pose_estimate.cpp
  )

target_link_libraries(MrsRvizPoseEstimate
  ${catkin_LIBRARIES}
  )

## RVIZ TRACK ARRAY

add_library(MrsTrackArrayRviz
  include/track_array/display.h
  src/track_array/display.cpp
  )

add_dependencies(MrsTrackArrayRviz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsTrackArrayRviz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## NAMED GOAL TOOL

add_library(MrsNamedGoalTool
  include/named_goal_tool/named_goal_tool.h
  src/named_goal_tool/named_goal_tool.cpp
  )

add_dependencies(MrsNamedGoalTool
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsNamedGoalTool
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch data icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
