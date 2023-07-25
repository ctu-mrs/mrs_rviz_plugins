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

#define KEY_ENTER 16777220
#define READ_ONLY false
#define DEFAULT_PROPERTIES_NUM 7
#define DEFAULT_TOPIC "trajectory_generation/path"
#define DEFAULT_DRONE "uav15"

// TODO: 
// 
// add license description - nope
// X add rotate the flags
// X Add transformation to rotation // attitude_converter
// X add to the readme
// X read frame_id and set it to the message
// X make the arrow disappear
// X drone name and topic name without "/"
// X add message that service has not been successful
// X height offset

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

// Sends added waypoints to service
int WaypointPlanner::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
  PoseTool::processKeyEvent(event, panel);
  if(event->key() == KEY_ENTER){
    // Set general attributes
    mrs_msgs::PathSrv srv;
    std::string frame_id = "fcu";
    srv.request.path.header.frame_id = frame_id;
    srv.request.path.use_heading = use_heading_property->getBool();
    srv.request.path.fly_now = fly_now_property->getBool();
    srv.request.path.stop_at_waypoints = stop_at_waypoints_property->getBool();
    srv.request.path.loop = loop_property->getBool();

    // Transform positions
    srv.request.path.points = generate_references(context_->getFrameManager()->getFixedFrame());
    
    // Make the call
    if(client.call(srv)){
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
    // Note: the tool does not exit. Good thing that it doesn't have to XD
    return Render;
  }
  return Render;
}

// Transforms positions from current frame to the fcu frame and saves results to vector.
std::vector<mrs_msgs::Reference> WaypointPlanner::generate_references(std::string current_frame){
  // Note: ref.position is reset to 0 for some reason. Although it does not matter now, it is weird
  std::string drone_name = drone_name_property->getStdString();
  std::string topic = drone_name + std::string("/odometry/odom_main");
  std::vector<mrs_msgs::Reference> res(positions.size());
  
  // Make transformer
  std::string goal_frame = /*"/" +*/ drone_name + "/fcu";
  auto tf = transformer.getTransform(current_frame, goal_frame);
  if(!tf){
    ROS_INFO("[Waypoint planner]: No transformation found. No data sent");
    return(std::vector<mrs_msgs::Reference>{});
  }
  
  // Transforming
  for(int i=0; i<positions.size(); i++){
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
