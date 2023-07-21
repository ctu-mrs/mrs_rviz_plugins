#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/mesh_loader.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include "waypoint_planner/waypoint_planner.h"

#include <ros/console.h>

#define DEFAULT_PROPERTIES_NUM 6
#define READ_ONLY false
#define DEFAULT_TOPIC "/trajectory_generation/path"
#define DEFAULT_DRONE "/uav15"

namespace mrs_rviz_plugins
{
WaypointPlanner::WaypointPlanner() {
  shortcut_key_ = 'w';

  drone_name_property = new rviz::StringProperty("Drone name", DEFAULT_DRONE, "Drone name + topic = service to send path.",
                                            getPropertyContainer(), SLOT(update_topic()), this);
  topic_property = new rviz::StringProperty("Topic", DEFAULT_TOPIC, "Drone name + topic = service to send path.", 
                                            getPropertyContainer(), SLOT(update_topic()), this);
  
  // TODO: update description
  use_heading_property = new rviz::BoolProperty("Use heading", false, "Honestly, no clue.",
      getPropertyContainer());
  fly_now_property = new rviz::BoolProperty("Fly now", true, "If true, the drone will start moving imediatelt after enter.",
      getPropertyContainer());
  stop_at_waypoints_property = new rviz::BoolProperty("Stop at waypoints", false, "If true, the drone will stop at added points.",
      getPropertyContainer());
  loop_property = new rviz::BoolProperty("Loop", false, "If true, drone will fly continuesly untill new path is sent",
      getPropertyContainer());
  
  drone_name_property->setReadOnly(false);
  topic_property->setReadOnly(false);
  use_heading_property->setReadOnly(false);
  fly_now_property->setReadOnly(false);
  stop_at_waypoints_property->setReadOnly(false);
  loop_property->setReadOnly(false);

  // TODO: initialize client by uav_name from `rosservice list` or given by user in tool properties
}

void WaypointPlanner::update_topic(){
  ROS_INFO("update_topic called!");
  client = node_handler.serviceClient<mrs_msgs::PathSrv>(drone_name_property->getStdString() + topic_property->getStdString());
  // pub = node_handler.advertise<geometry_msgs::PoseStamped>(topic_property->getStdString(), 2);
}

void WaypointPlanner::add_property(){
  current_property = new rviz::Property();

  current_point_property = new rviz::VectorProperty("Point: ", Ogre::Vector3::ZERO, QString(), 
      current_property, SLOT(update_position()), this);
  current_point_property->setReadOnly(READ_ONLY);

  current_theta_property = new rviz::FloatProperty("Angle:", 0, QString(), 
      current_property, SLOT(update_position()), this);
  current_theta_property->setReadOnly(READ_ONLY);

  current_property->setName("Position");
  current_property->setReadOnly(READ_ONLY);
  getPropertyContainer()->addChild(current_property);
  point_properties.push_back(current_point_property);
  angle_properties.push_back(current_theta_property);
}

void WaypointPlanner::update_position(){
  // Iterate through every position and update them
  for(int i=0; i<pose_nodes.size(); i++){
    Ogre::Vector3 pos = point_properties[i]->getVector();
    pose_nodes[i]->setPosition(pos);
    // Update is called on creating new position, and seems to be called before onPoseSet.
    // So no need to change it here on initing. If user changes values manually, position will be present and updated.
    if(i<positions.size()){
      positions[i].set_values(pos, angle_properties[i]->getFloat());
    }
  }
}

// Turning the plugin on
void WaypointPlanner::onInitialize(){
  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);
  setName("Plan way");

  flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";
  if(rviz::loadMeshFromResource( flag_resource_ ).isNull()){
    ROS_ERROR( "PlantFlagTool: failed to load model resource '%s'.", flag_resource_.c_str() );
  }

  update_topic();
}

// Choosing the tool
void WaypointPlanner::activate()
{
  for(int i=0; i<pose_nodes.size(); i++){
    pose_nodes[i]->setVisible(true);
  }

  add_property();
}

// Switching to another tool
void WaypointPlanner::deactivate(){
  for(int i=0; i<pose_nodes.size(); i++){
    pose_nodes[i]->setVisible(false);
  }

  if(!current_property){
    return;
  }
  delete current_property;
  current_property = nullptr;
  point_properties.pop_back();
  angle_properties.pop_back();
}

void WaypointPlanner::onPoseSet(double x, double y, double theta){
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
  Ogre::Vector3 position = Ogre::Vector3(x, y, 0);
  // rviz::Arrow* arrow = new rviz::Arrow();
  node->attachObject(entity);
  node->setVisible(true);
  node->setPosition(position);
  pose_nodes.push_back(node);
  
  ROS_INFO("%p", current_point_property);
  current_point_property->setVector(position);
  ROS_INFO("onPoseSet here2");
  current_theta_property->setFloat(theta);
  ROS_INFO("onPoseSet here3");
  add_property();

// TODO: rerad from /uav_name/odometry/main_odom topic
  positions.push_back(WaypointPlanner::Position(x, y, 1, theta));
}

int WaypointPlanner::processMouseEvent(rviz::ViewportMouseEvent& event){
  int res = PoseTool::processMouseEvent(event);
  return res & (~Finished);
}

int WaypointPlanner::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
  PoseTool::processKeyEvent(event, panel);
  if(event->key() == 16777220){
    ROS_INFO("enter press has been received");
    ROS_INFO(" : %s", client.getService().c_str());
    mrs_msgs::PathSrv srv;
    srv.request.path.use_heading = use_heading_property->getBool();
    srv.request.path.fly_now = fly_now_property->getBool();
    srv.request.path.stop_at_waypoints = stop_at_waypoints_property->getBool();
    srv.request.path.loop = loop_property->getBool();

    srv.request.path.points = generate_references();
    // todo: change to /uav18/trajectory_generation/path
    if(client.call(srv)){
      ROS_INFO("Call has been successfull");
    }else{
      ROS_INFO("Call failed: %s", srv.response.message.c_str());
    }
    positions.clear();
    point_properties.clear();
    angle_properties.clear();
    for(int i=0; i<pose_nodes.size(); i++){
      delete pose_nodes[i];
    }
    pose_nodes.clear();
    getPropertyContainer()->removeChildren(DEFAULT_PROPERTIES_NUM, -1);
    add_property();
    // Note: the tool does not exit. Good thing that it doesn't have to XD
    return Render;
  }
  return Render;
}

std::vector<mrs_msgs::Reference> WaypointPlanner::generate_references(){
  std::vector<mrs_msgs::Reference> res(positions.size());
  for(int i=0; i<positions.size(); i++){
    mrs_msgs::Reference ref;
    ref.position.x = positions[i].x;
    ref.position.y = positions[i].y;
    // TODO: is reset to 0 for some reason
    //ref.position.z = positions[i].z;
    ref.position.z = 1;
    ref.heading = positions[i].theta;
    res[i] = ref;
    ROS_INFO(" - %.2f, %.2f, %.2f", ref.position.x, ref.position.y, ref.position.z);
  }
  return res;
}

WaypointPlanner::~WaypointPlanner(){
  for( unsigned i = 0; i < pose_nodes.size(); i++ ){
    scene_manager_->destroySceneNode(pose_nodes[i]);
  }
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::WaypointPlanner, rviz::Tool)
