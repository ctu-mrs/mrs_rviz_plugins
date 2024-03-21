#include <coverage_path_planning/stride_method.h>

namespace mrs_rviz_plugins{

// StrideMethod::StrideMethod(rviz::Property* property_container, Ogre::SceneManager* scene_manager){

// }

void StrideMethod::update(mrs_lib::Polygon &new_polygon){

}

void StrideMethod::compute(mrs_lib::Polygon &new_polygon){

}

void StrideMethod::start(){
  ROS_INFO("[StrideMethod]: start called");
}
} // namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StrideMethod, mrs_rviz_plugins::CoverageMethod)