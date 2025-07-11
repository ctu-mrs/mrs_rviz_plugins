cmake_minimum_required(VERSION 3.5)
project(mrs_rviz_plugins)

# set the correct standards
set(CMAKE_C_STANDARD 99)
# can not use C++17 or higher because of OGRE
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(DEPENDENCIES
  pluginlib
  rviz_common 
  rviz_ogre_vendor
  mrs_msgs
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

# VERY IMPORTANT
# pre-compliation of the headers by QT to generate Q_SLOTS
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/mrs_rviz_plugins/sphere/display.h
  include/mrs_rviz_plugins/sphere/visual.h
)

set(LIBRARIES
  # MrsRvizPlugins_Bumper
  # MrsRvizPlugins_Covariance
  # MrsRvizPlugins_FastArrow
  # MrsRvizPlugins_SmartLine
  # MrsRvizPlugins_NavGoal
  # MrsRvizPlugins_PoseEstimate
  MrsRvizPlugins_Sphere
  # MrsRvizPlugins_PoseWithCovarianceArray
  # MrsRvizPlugins_TrackArray
  # MrsRvizPlugins_NamedGoalTool
  # MrsRvizPlugins_OdomViz
  # MrsRvizPlugins_WaypointPlanner
  # MrsRvizPlugins_Status
  )

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

## SPHERE VIZUALIZATION

add_library(MrsRvizPlugins_Sphere SHARED
  src/sphere/display.cpp
  src/sphere/visual.cpp
  ${MOC_FILES}
  )

target_include_directories(MrsRvizPlugins_Sphere PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(MrsRvizPlugins_Sphere
  pluginlib
  rviz_common
  rviz_ogre_vendor
  mrs_msgs
)

target_link_libraries(MrsRvizPlugins_Sphere
  ${QT_LIBRARIES}
  )

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(rviz_common plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include
  DESTINATION .
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ${LIBRARIES}
  )

ament_export_include_directories(
  include
  )

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  )

ament_export_dependencies(
  ${DEPENDENCIES}
  )

ament_package()