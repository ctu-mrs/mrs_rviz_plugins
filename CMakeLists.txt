cmake_minimum_required(VERSION 2.8.3)
project(mrs_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  rviz
  mrs_msgs
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

link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mrs_msgs rviz
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrsRvizPlugins
  # bumper
  include/bumper/display.h
  include/bumper/visual.h
  src/bumper/display.cpp
  src/bumper/visual.cpp
  # covariance
  include/covariance/property.h
  include/covariance/visual.h
  src/covariance/property.cpp
  src/covariance/visual.cpp
  # pose with covariance array
  src/pose_with_covariance_array/display.cpp
  )

add_dependencies(MrsRvizPlugins
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsRvizPlugins
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## Install rules

install(TARGETS
  MrsRvizPlugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES
  plugins.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY media/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
