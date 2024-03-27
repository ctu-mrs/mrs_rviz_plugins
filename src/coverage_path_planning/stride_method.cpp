#include <coverage_path_planning/stride_method.h>

#include <rviz/ogre_helpers/axes.h>

namespace mrs_rviz_plugins{

void StrideMethod::update(mrs_lib::Polygon &new_polygon){

}

void StrideMethod::compute(mrs_lib::Polygon &new_polygon){

}

void StrideMethod::setStart(Ogre::Vector3 position){
  
}

void StrideMethod::start(){
  ROS_INFO("[StrideMethod]: start called");

  Ogre::SceneNode* new_node = scene_manager_->createSceneNode();
  root_node_->addChild(new_node);
  new rviz::Axes(scene_manager_, new_node);
}
} // namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StrideMethod, mrs_rviz_plugins::CoverageMethod)