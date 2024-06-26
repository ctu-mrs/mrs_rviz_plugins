cmake_minimum_required(VERSION 3.5)
project(mrs_rviz_plugins)

set(CATKIN_DEPENDENCIES
  cmake_modules
  geometry_msgs
  mrs_lib
  mrs_msgs
  nav_msgs
  nodelet
  pluginlib
  roscpp
  rviz
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

# ignore c++17 deprecation warnings for the register storage class specifier because of OGRE
add_definitions("-Wno-register")

set(LIBRARIES
  MrsRvizPlugins_Bumper
  MrsRvizPlugins_Covariance
  MrsRvizPlugins_FastArrow
  MrsRvizPlugins_SmartLine
  MrsRvizPlugins_NavGoal
  MrsRvizPlugins_PoseEstimate
  MrsRvizPlugins_Sphere
  MrsRvizPlugins_PoseWithCovarianceArray
  MrsRvizPlugins_TrackArray
  MrsRvizPlugins_NamedGoalTool
  MrsRvizPlugins_OdomViz
  MrsRvizPlugins_WaypointPlanner
  MrsRvizPlugins_Status
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## BUMPER VIZUALIZATION

add_library(MrsRvizPlugins_Bumper
  include/bumper/display.h
  include/bumper/visual.h
  src/bumper/display.cpp
  src/bumper/visual.cpp
  )

add_dependencies(MrsRvizPlugins_Bumper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_Bumper
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## SPHERE VIZUALIZATION

add_library(MrsRvizPlugins_Sphere
  include/sphere/display.h
  include/sphere/visual.h
  src/sphere/display.cpp
  src/sphere/visual.cpp
  )

add_dependencies(MrsRvizPlugins_Sphere
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_Sphere
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## COVARIANCE

add_library(MrsRvizPlugins_Covariance
  include/covariance/property.h
  include/covariance/visual.h
  src/covariance/property.cpp
  src/covariance/visual.cpp
  )

add_dependencies(MrsRvizPlugins_Covariance
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_Covariance
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## SMART LINE

add_library(MrsRvizPlugins_SmartLine
  include/smart_line/smart_line.h
  src/smart_line/smart_line.cpp
  )

add_dependencies(MrsRvizPlugins_SmartLine
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_SmartLine
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## FAST ARROW

add_library(MrsRvizPlugins_FastArrow
  include/fast_arrow/fast_arrow.h
  src/fast_arrow/fast_arrow.cpp
  )

add_dependencies(MrsRvizPlugins_FastArrow
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_FastArrow
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  MrsRvizPlugins_SmartLine
  )

## POSE WITH COVARIANCE ARRAY

add_library(MrsRvizPlugins_PoseWithCovarianceArray
  include/pose_with_covariance_array/display.h
  src/pose_with_covariance_array/display.cpp
  )

add_dependencies(MrsRvizPlugins_PoseWithCovarianceArray
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_PoseWithCovarianceArray
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  MrsRvizPlugins_Covariance
  MrsRvizPlugins_FastArrow
  )

## RVIZ NAV GOAL

add_library(MrsRvizPlugins_NavGoal
  src/rviz_interface/nav_goal.cpp
  )

target_link_libraries(MrsRvizPlugins_NavGoal
  ${catkin_LIBRARIES}
  )

## RVIZ POSE ESTIMATE

add_library(MrsRvizPlugins_PoseEstimate
  src/rviz_interface/pose_estimate.cpp
  )

target_link_libraries(MrsRvizPlugins_PoseEstimate
  ${catkin_LIBRARIES}
  )

## RVIZ TRACK ARRAY

add_library(MrsRvizPlugins_TrackArray
  include/track_array/display.h
  src/track_array/display.cpp
  )

add_dependencies(MrsRvizPlugins_TrackArray
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_TrackArray
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## NAMED GOAL TOOL

add_library(MrsRvizPlugins_NamedGoalTool
  include/named_goal_tool/named_goal_tool.h
  src/named_goal_tool/named_goal_tool.cpp
  )

add_dependencies(MrsRvizPlugins_NamedGoalTool
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_NamedGoalTool
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## ODOM ViZ

add_library(MrsRvizPlugins_OdomViz
  include/odom_viz/odom_viz.h
  include/odom_viz/visual_entity.h
  src/odom_viz/odom_viz.cpp
  src/odom_viz/visual_entity.cpp
)

add_dependencies(MrsRvizPlugins_OdomViz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(MrsRvizPlugins_OdomViz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

## WAYPOINT PLANNER

add_library(MrsRvizPlugins_WaypointPlanner
  include/waypoint_planner/waypoint_planner.h
  src/waypoint_planner/waypoint_planner.cpp
)

add_dependencies(MrsRvizPlugins_WaypointPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(MrsRvizPlugins_WaypointPlanner
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

## STATUS DISPLAY and CONTOL TOOL

add_library(MrsRvizPlugins_Status
  include/uav_status/status_display.h
  include/uav_status/overlay_utils.h
  include/control/im_server.h
  include/control/control.h
  include/control/drone_entity.h
  include/control/overlay_picker_tool.h
  src/control/im_server.cpp
  src/control/control.cpp
  src/control/drone_entity.cpp
  src/control/overlay_picker_tool.cpp
  src/uav_status/status_display.cpp
  src/uav_status/overlay_utils.cpp
  )

add_dependencies(MrsRvizPlugins_Status
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins_Status
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

install(FILES plugins.xml nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
