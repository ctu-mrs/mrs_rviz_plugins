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

#include <stdlib.h>

#define KEY_ENTER 16777220
#define KEY_DELETE 16777223
#define READ_ONLY false
#define DEFAULT_PROPERTIES_NUM 7
#define DEFAULT_TOPIC "trajectory_generation/path"
#define DEFAULT_DRONE "uav15"

namespace mrs_rviz_plugins
{
WaypointPlanner::WaypointPlanner() {
  shortcut_key_ = 'w';
  transformer = mrs_lib::Transformer(node_handler);

  // Add default properties
  drone_name_property = new rviz::StringProperty("Drone name", DEFAULT_DRONE, "Drone name + topic = service to send path.",
                                            getPropertyContainer(), SLOT(update_topic()), this);
  topic_property = new rviz::StringProperty("Topic", DEFAULT_TOPIC, "Drone name + topic = service to send path.", 
                                            getPropertyContainer(), SLOT(update_topic()), this);
  height_offset_property = new rviz::FloatProperty("Height offset", 0.0, "Value to be added to the actual \"z\" coordinate.",
                                            getPropertyContainer());
  use_heading_property = new rviz::BoolProperty("Use heading", true, "If true, drone will rotate according to set heading",
      getPropertyContainer());
  fly_now_property = new rviz::BoolProperty("Fly now", true, "If true, the drone will start moving imediately after enter.",
      getPropertyContainer());
  stop_at_waypoints_property = new rviz::BoolProperty("Stop at waypoints", false, "If true, the drone will stop at added points.",
      getPropertyContainer());
  loop_property = new rviz::BoolProperty("Loop", false, "If true, drone will fly continuously until new path is sent",
      getPropertyContainer());
  
  // Allow changing the values of properties
  drone_name_property->setReadOnly(false);
  topic_property->setReadOnly(false);
  use_heading_property->setReadOnly(false);
  fly_now_property->setReadOnly(false);
  stop_at_waypoints_property->setReadOnly(false);
  loop_property->setReadOnly(false);
}

// Callback on topic change
void WaypointPlanner::update_topic(){
  client = node_handler.serviceClient<mrs_msgs::PathSrv>(std::string("/") + drone_name_property->getStdString() + std::string("/") + topic_property->getStdString());
  status = std::string("Drone name is set to ") + drone_name_property->getStdString();
  setStatus(QString(status.c_str()));
}

// Callback on position change
void WaypointPlanner::update_position(){
  // Iterate through every position and update them
  for(int i=0; i<pose_nodes.size(); i++){
    Ogre::Vector3 pos = point_properties[i]->getVector();
    pose_nodes[i]->setPosition(pos);
    tf2::Quaternion tmp;
    tmp.setRPY(0,0,angle_properties[i]->getFloat());
    pose_nodes[i]->setOrientation(tmp.getW(), tmp.getX(), tmp.getY(), tmp.getZ());
    // Update is called on creating new position, and seems to be called before onPoseSet.
    // So no need to change it here on initing. If user changes values manually, position will be present and updated.
    if(i<positions.size()){
      positions[i].set_values(pos, angle_properties[i]->getFloat());
    }
  }
}

// Saves current_position to the properties. Makes new current_position
void WaypointPlanner::add_property(){
  current_property = new rviz::Property();

  current_point_property = new rviz::VectorProperty("Point: ", Ogre::Vector3::ZERO, QString(), 
      current_property, SLOT(update_position()), this);
  current_point_property->setReadOnly(READ_ONLY);

  current_theta_property = new rviz::FloatProperty("Angle:", 0, QString(), 
      current_property, SLOT(update_position()), this);
  current_theta_property->setReadOnly(READ_ONLY);

  current_property->setName("Position " + QString::number(positions.size() + 1));
  current_property->setReadOnly(READ_ONLY);
  getPropertyContainer()->addChild(current_property);
  point_properties.push_back(current_point_property);
  angle_properties.push_back(current_theta_property);
}

// Turning the plugin on
void WaypointPlanner::onInitialize(){
  setName("Plan way");

  // Set up the arrow
  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);

  // Load the flag model
  flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";
  if(rviz::loadMeshFromResource( flag_resource_ ).isNull()){
    ROS_ERROR( "[Waypoint planner]: failed to load model resource '%s'.", flag_resource_.c_str() );
  }

  // Preparing for searching the drone's name
  XmlRpc::XmlRpcValue req = "/node";
  XmlRpc::XmlRpcValue res;
  XmlRpc::XmlRpcValue pay;
  std::vector<std::string> drone_names;
  ros::master::execute("getSystemState",req ,res ,pay ,true);

  // Search for the drone's name
  std::string state[res[2][2].size()];
  for(int x=0 ; x<res[2][2].size() ; x++){
    std::string name = res[2][2][x][0].toXml().c_str();
    if(name.find("trajectory_generation/path") == std::string::npos){
      continue;
    }
    ROS_INFO("[Waypoint planner]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if(index != std::string::npos){
      name = name.erase(0, index+1);
    }

    index = name.find("/", 1, 1);
    if(index != std::string::npos){
      name = name.erase(index);
    }

    drone_names.push_back(name);
    ROS_INFO("[Waypoint planner]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Set up drone's name
  if(drone_names.size() == 0){
    status = std::string("Warning: No drone was found. Drone name set to ") 
        + std::string(DEFAULT_DRONE);
  } else if(drone_names.size() > 1){
    drone_name_property->setStdString(drone_names[0]);
    status = "Warning: Several drones found. Please, set Drone name property";
  } else{
    drone_name_property->setStdString(drone_names[0]);
    status = std::string("Drone name is set to ") + drone_name_property->getStdString();
  }

  client = node_handler.serviceClient<mrs_msgs::PathSrv>(std::string("/") + 
  drone_name_property->getStdString() + std::string("/") + topic_property->getStdString());
}

// Choosing the tool
void WaypointPlanner::activate()
{
  for(int i=0; i<pose_nodes.size(); i++){
    pose_nodes[i]->setVisible(true);
  }

  setStatus(QString(status.c_str()));

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

// Saves inputted position. Leaves flag model on inputted position.
void WaypointPlanner::onPoseSet(double x, double y, double theta){
  if(is_on_loop){
    status = "Request denied, the program waits for drone to get to the first waypoint";
    setStatus(QString(status.c_str()));
    return;
  }
  arrow_->getSceneNode()->setVisible(false);
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  // Note: after reset the file cannot be found. Even try-catch and loading it 
  // again does not help.
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
  
  Ogre::Vector3 position = Ogre::Vector3(x, y, 0);
  node->attachObject(entity);
  tf2::Quaternion tmp;
  tmp.setRPY(0,0,theta);
  node->rotate(Ogre::Quaternion(tmp.getW(), tmp.getX(), tmp.getY(), tmp.getZ()));
  node->setVisible(true);
  node->setPosition(position);
  pose_nodes.push_back(node);
  
  current_point_property->setVector(position);
  current_theta_property->setFloat(theta);

  positions.push_back(WaypointPlanner::Position(x, y, 1, theta));
  add_property();
}

int WaypointPlanner::processMouseEvent(rviz::ViewportMouseEvent& event){
  int res = PoseTool::processMouseEvent(event);
  return res & (~Finished);
}

void WaypointPlanner::process_loop(){
  is_on_loop = true;
  // Set general attributes
  mrs_msgs::Vec4 srv;
  auto point = generate_references(context_->getFrameManager()->getFixedFrame(), 1);
  srv.request.goal[0] = point[0].position.x;
  srv.request.goal[1] = point[0].position.y;
  srv.request.goal[2] = point[0].position.z;
  srv.request.goal[3] = 0;

  // srv.request.goal[0] = positions[0].x;
  // srv.request.goal[1] = positions[0].y;
  // srv.request.goal[2] = positions[0].z;
  // srv.request.goal[3] = positions[0].theta;
  
  // Temporaly set client to send requests to different service
  client = node_handler.serviceClient<mrs_msgs::Vec4>(std::string("/") + 
  drone_name_property->getStdString() + std::string("/control_manager/goto_fcu"));



  // Make the call
  client.call(srv);
  client = node_handler.serviceClient<mrs_msgs::PathSrv>(std::string("/") + 
  drone_name_property->getStdString() + std::string("/") + topic_property->getStdString());
  if(srv.response.success){
    ROS_INFO("[Waypoint planner]: Looping: initial call processed successfully");
    status = "Looping: initial call processed successfully";
    setStatus(QString(status.c_str()));
  }else{
    ROS_INFO("[Waypoint planner]: Loop: initial call failed: %s", srv.response.message.c_str());
    status = "Looping: initial call failed: " + srv.response.message + " Try checking, if drone name is correct.";
    setStatus(QString(status.c_str()));

    is_on_loop = false;
    return;
  }

  // Wait until drone is on the goal
  std::string drone_name = drone_name_property->getStdString();
  std::string topic = drone_name + std::string("/control_manager/diagnostics");
  bool is_ok = true;;
  while(true){
    auto info_msg = ros::topic::waitForMessage<mrs_msgs::ControlManagerDiagnostics>(topic, node_handler, ros::Duration(1.0));
    if(info_msg.get() == nullptr){
      ROS_INFO("Diagnostics data are not received");
      continue;
    }
    if(!info_msg->flying_normally){
      is_ok = false;
      break;
    }
    if(!info_msg->tracker_status.have_goal){
      break;
    }
  }

  is_on_loop = false;
  if(!is_ok){
    return;
  }

  Position final_pos = Position(positions[0].x, positions[0].y, positions[0].z, positions[0].theta);
  positions.push_back(final_pos);

  send_waypoints();
}

void WaypointPlanner::send_waypoints(){
  // Set general attributes
  mrs_msgs::PathSrv srv;
  // std::string frame_id = "fcu";
  srv.request.path.header.frame_id = "fcu";
  srv.request.path.use_heading = use_heading_property->getBool();
  srv.request.path.fly_now = fly_now_property->getBool();
  srv.request.path.stop_at_waypoints = stop_at_waypoints_property->getBool();
  srv.request.path.loop = loop_property->getBool();

  // Transform positions
  srv.request.path.points = generate_references(context_->getFrameManager()->getFixedFrame(), -1);
  
  // Make the call
  client.call(srv);
  if(srv.response.success){
    ROS_INFO("[Waypoint planner]: Call processed successfully");
    status = "Call processed successfully";
    setStatus(QString(status.c_str()));
  }else{
    ROS_INFO("[Waypoint planner]: Call failed: %s", srv.response.message.c_str());
    status = "Call failed: " + srv.response.message + " Try checking, if drone name is correct.";
    setStatus(QString(status.c_str()));
  }

  // Clean-up
  positions.clear();
  point_properties.clear();
  angle_properties.clear();
  for(int i=0; i<pose_nodes.size(); i++){
    delete pose_nodes[i];
  }
  pose_nodes.clear();
  getPropertyContainer()->removeChildren(DEFAULT_PROPERTIES_NUM, -1);
  add_property();
}

int WaypointPlanner::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
  if(is_on_loop){
    status = "Request denied, the program waits for drone to get to the first waypoint";
    setStatus(QString(status.c_str()));
    return Render;
  }
  PoseTool::processKeyEvent(event, panel);
  ROS_INFO("Received key %d", (int) event->key());
  // Sends added waypoints to service
  if(event->key() == KEY_ENTER){
    if(loop_property->getBool()){
      std::thread t = std::thread([this]{process_loop();});
      t.detach();
    }else{
      send_waypoints();
    }
    
    // Note: the tool does not exit. Good thing that it doesn't have to XD
    return Render;
  }
  if(event->key() == KEY_DELETE && !positions.empty()){
    positions.pop_back();
    std::size_t index = point_properties.size() - 2;
    point_properties.erase(point_properties.begin() + index);
    angle_properties.erase(angle_properties.begin() + index);
    delete pose_nodes.back();
    pose_nodes.pop_back();
    ROS_INFO("Removing index = %d", DEFAULT_PROPERTIES_NUM + positions.size());
    getPropertyContainer()->removeChildren(DEFAULT_PROPERTIES_NUM + positions.size(), 1);
    current_property->setName("Position " + QString::number(positions.size() + 1));
  }
  return Render;
}


std::vector<mrs_msgs::Reference> WaypointPlanner::generate_references(std::string current_frame, int num){
  num = num != -1 ? num : positions.size(); 
  // Note: ref.position is reset to 0 for some reason. Although it does not matter now, it is weird
  std::string drone_name = drone_name_property->getStdString();
  std::string topic = drone_name + std::string("/odometry/odom_main");
  std::vector<mrs_msgs::Reference> res(num);
  
  // Make transformer
  std::string goal_frame = /*"/" +*/ drone_name + "/fcu";
  auto tf = transformer.getTransform(current_frame, goal_frame);
  if(!tf){
    ROS_INFO("[Waypoint planner]: No transformation found. No data sent");
    return(std::vector<mrs_msgs::Reference>{});
  }
  
  // Transforming
  for(int i=0; i<positions.size() && i < num; i++){
    // Setting position in current frame.
    geometry_msgs::Pose pose; 
    pose.position.x = positions[i].x;
    pose.position.y = positions[i].y;
    pose.position.z = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, positions[i].theta);
    pose.orientation.w = quaternion.getW();
    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ(); 
    
    // Transform
    auto point_transformed = transformer.transform(pose, tf.value());
    if(!point_transformed){
      ROS_INFO("[Waypoint planner]: Unable to transform cmd reference from %s to %s at time %.6f. No data sent",
          current_frame.c_str(), goal_frame.c_str(), ros::Time::now().toSec());
      return(std::vector<mrs_msgs::Reference>{});
    }

    // Convert transformed heading from quaternion to XY projection
    pose = point_transformed.value();
    double rotation = mrs_lib::AttitudeConverter(pose.orientation).getHeading();
    mrs_msgs::Reference ref;

    // Save results
    ref.position.x = pose.position.x;
    ref.position.y = pose.position.y;
    ref.position.z += height_offset_property->getFloat();
    ref.heading = rotation;
    res[i] = ref;
    ROS_INFO("[Waypoint planner]: %.2f, %.2f, %.8f", ref.position.x, ref.position.y, ref.position.z);
  }

  ROS_INFO("[Waypoint planner]: All transformation went successfully");
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
