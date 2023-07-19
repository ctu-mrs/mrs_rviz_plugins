#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/mesh_loader.h>

#include "waypoint_planner/waypoint_planner.h"

#include <ros/console.h>

namespace mrs_rviz_plugins
{
WaypointPlanner::WaypointPlanner() {
  shortcut_key_ = 'w';
}

void WaypointPlanner::onInitialize(){
  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);
  setName("Plan way");
  flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";
  if(rviz::loadMeshFromResource( flag_resource_ ).isNull()){
    ROS_ERROR( "PlantFlagTool: failed to load model resource '%s'.", flag_resource_.c_str() );
  }
}

void WaypointPlanner::onPoseSet(double x, double y, double theta){
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
  Ogre::Vector3 position = Ogre::Vector3(x, y, 0);
  node->attachObject(entity);
  node->setVisible(true);
  node->setPosition(position);
  pose_nodes.push_back(node);
}

WaypointPlanner::~WaypointPlanner(){
  for( unsigned i = 0; i < pose_nodes.size(); i++ ){
    scene_manager_->destroySceneNode( pose_nodes[i]);
  }
}


}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::WaypointPlanner, rviz::Tool)
