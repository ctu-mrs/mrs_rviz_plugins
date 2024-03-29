#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/axes.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include "waypoint_planner/waypoint_planner.h"

#include <ros/console.h>

#include <stdlib.h>

namespace mrs_rviz_plugins
{
WaypointPlanner::WaypointPlanner() {
  shortcut_key_ = 'w';
  transformer   = mrs_lib::Transformer(node_handler);

  // Add default properties
  drone_name_property    = new rviz::StringProperty("Drone name", DEFAULT_DRONE.c_str(), "Drone name + topic = service to send path.", getPropertyContainer(),
                                                    SLOT(update_topic()), this);
  topic_property         = new rviz::StringProperty("Topic", DEFAULT_TOPIC.c_str(), "Drone name + topic = service to send path.", getPropertyContainer(),
                                                    SLOT(update_topic()), this);
  height_offset_property = new rviz::FloatProperty("Height offset", 0.0, "Value to be added to the actual \"z\" coordinate.", getPropertyContainer());
  use_heading_property   = new rviz::BoolProperty("Use heading", true, "If true, drone will rotate according to set heading", getPropertyContainer());
  fly_now_property       = new rviz::BoolProperty("Fly now", true, "If true, the drone will start moving imediately after enter.", getPropertyContainer());
  stop_at_waypoints_property = new rviz::BoolProperty("Stop at waypoints", false, "If true, the drone will stop at added points.", getPropertyContainer());
  loop_property              = new rviz::BoolProperty("Loop", false, "If true, drone will fly continuously until new path is sent", getPropertyContainer());
  shape_property             = new rviz::EnumProperty("Shape", "Axes", "Shape to show positions.", getPropertyContainer(), SLOT(update_shape()), this);

  // Allow changing the values of properties
  drone_name_property->setReadOnly(false);
  topic_property->setReadOnly(false);
  use_heading_property->setReadOnly(false);
  fly_now_property->setReadOnly(false);
  stop_at_waypoints_property->setReadOnly(false);
  loop_property->setReadOnly(false);
  shape_property->addOption("Axes", 0);
  shape_property->addOption("Arrow", 1);
  is_in_loop = false;
}

// Callback on topic change
void WaypointPlanner::update_topic() {
  client =
      node_handler.serviceClient<mrs_msgs::PathSrv>(std::string("/") + drone_name_property->getStdString() + std::string("/") + topic_property->getStdString());
  status = std::string("Drone name is set to ") + drone_name_property->getStdString();
  setStatus(QString(status.c_str()));
}

// Callback on position change
void WaypointPlanner::update_position() {
  // Iterate through every position and update them
  for (int i = 0; i < pose_nodes.size(); i++) {
    const Ogre::Vector3 pos = point_properties[i]->getVector();
    pose_nodes[i]->setPosition(pos);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle_properties[i]->getFloat());
    pose_nodes[i]->setOrientation(quat.getW(), quat.getX(), quat.getY(), quat.getZ());

    // Update is called on creating new position, and seems to be called before onPoseSet.
    // So no need to change it here on initing. If user changes values manually,
    // position will be present and updated.
    if (i < positions.size()) {
      positions[i].set_values(pos, angle_properties[i]->getFloat());
    }
  }
}

void WaypointPlanner::update_shape() {
  // Awful API: no clearup method
  // Cleaning the ugly way
  for (auto ax : axes) {
    delete ax;
  }
  axes.clear();
  for (auto arrow : arrows) {
    delete arrow;
  }
  arrows.clear();

  if (shape_property->getOptionInt() == 1) {
    for (const auto& node : pose_nodes) {
      rviz::Arrow* arrow = new rviz::Arrow(scene_manager_, node);
      arrow->setDirection(Ogre::Vector3(1, 0, 0));
      arrow->setColor(1, 0.1, 0, 1);
      arrow->set(2, 0.2, 0.6, 0.4);
      arrows.push_back(arrow);
    }
  } else if (shape_property->getOptionInt() == 0) {
    for (const auto& node : pose_nodes) {
      axes.push_back(new rviz::Axes(scene_manager_, node));
    }
  }
}

// Saves current_position to the properties. Makes new current_position
void WaypointPlanner::addProperties() {
  current_property = new rviz::Property();

  current_point_property = new rviz::VectorProperty("Point: ", Ogre::Vector3::ZERO, QString(), current_property, SLOT(update_position()), this);
  current_point_property->setReadOnly(READ_ONLY);

  current_theta_property = new rviz::FloatProperty("Angle: ", 0, QString(), current_property, SLOT(update_position()), this);
  current_theta_property->setReadOnly(READ_ONLY);

  current_property->setName("Position: " + QString::number(positions.size() + 1));
  current_property->setReadOnly(READ_ONLY);
  getPropertyContainer()->addChild(current_property);
  point_properties.push_back(current_point_property);
  angle_properties.push_back(current_theta_property);
}

// Turning the plugin on
void WaypointPlanner::onInitialize() {
  setName("Plan path");

  // Set up the arrow
  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);

  // Preparing for searching the drone's name
  XmlRpc::XmlRpcValue      req = "/node";
  XmlRpc::XmlRpcValue      res;
  XmlRpc::XmlRpcValue      pay;
  std::vector<std::string> drone_names;
  ros::master::execute("getSystemState", req, res, pay, true);

  // Search for the drone's name
  std::string state[res[2][2].size()];
  for (int x = 0; x < res[2][2].size(); x++) {
    std::string name = res[2][2][x][0].toXml().c_str();
    if (name.find("trajectory_generation/path") == std::string::npos) {
      continue;
    }
    ROS_INFO("[Waypoint planner]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    ROS_INFO("[Waypoint planner]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Set up drone's name
  if (drone_names.size() == 0) {

    status = std::string("Warning: No drone was found. Drone name set to: ") + DEFAULT_DRONE;

  } else if (drone_names.size() > 1) {

    drone_name_property->setStdString(drone_names[0]);
    status = "Warning: Several drones found. Please, set drone name property";

  } else {

    drone_name_property->setStdString(drone_names[0]);
    status = std::string("Drone name is set to ") + drone_name_property->getStdString();
  }

  client =
      node_handler.serviceClient<mrs_msgs::PathSrv>(std::string("/") + drone_name_property->getStdString() + std::string("/") + topic_property->getStdString());
}

// Choosing the tool
void WaypointPlanner::activate() {
  for (const auto& node : pose_nodes) {
    node->setVisible(true);
  }

  setStatus(QString(status.c_str()));
  addProperties();
}

// Switching to another tool
void WaypointPlanner::deactivate() {
  for (const auto& node : pose_nodes) {
    node->setVisible(false);
  }

  if (!current_property) {
    return;
  }
  delete current_property;
  current_property = nullptr;
  point_properties.pop_back();
  angle_properties.pop_back();
}

// Saves inputted position. Leaves flag model on inputted position.
void WaypointPlanner::onPoseSet(double x, double y, double theta) {

  if (is_in_loop) {
    status = "Request denied, the program waits for drone to get to the first waypoint";
    setStatus(QString(status.c_str()));
    return;
  }

  // Finding height of the drone:
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  const std::string source_frame = drone_name_property->getStdString() + "/fcu";
  const auto&       tf           = transformer.getTransform(source_frame, context_->getFrameManager()->getFixedFrame());
  if (!tf) {
    setStatus("[Waypoint planner]: No transformation from body frame to current frame found. Z component of visual entity is set to null");
    ROS_INFO("[Waypoint planner]: No transformation found. Z component of visual entity is set to null");
  } else {
    const auto& point_transformed = transformer.transform(pose, tf.value());
    if (!point_transformed) {
      setStatus("[Waypoint planner]: Unable to transform cmd reference from body to current frame => z-axis component of visual entity set to null.");
      ROS_INFO("[Waypoint planner]: Unable to transform cmd reference from %s to %s at time %.6f => z-axis component of visual entity set to null.",
               source_frame.c_str(), context_->getFrameManager()->getFixedFrame().c_str(), ros::Time::now().toSec());
      return;
    } else {
      pose = point_transformed.value();
    }
  }

  arrow_->getSceneNode()->setVisible(false);
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();

  if (shape_property->getOptionInt() == 1) {

    rviz::Arrow* arrow = new rviz::Arrow(scene_manager_, node);
    arrow->setDirection(Ogre::Vector3(1, 0, 0));
    arrow->setColor(1, 0.1, 0, 1);
    arrow->set(2, 0.2, 0.6, 0.4);
    arrows.push_back(arrow);

  } else if (shape_property->getOptionInt() == 0) {

    axes.push_back(new rviz::Axes(scene_manager_, node));
  }

  const Ogre::Vector3 position = Ogre::Vector3(x, y, pose.position.z);

  tf2::Quaternion quat;
  quat.setRPY(0, 0, theta);
  node->rotate(Ogre::Quaternion(quat.getW(), quat.getX(), quat.getY(), quat.getZ()));
  node->setVisible(true);
  node->setPosition(position);
  pose_nodes.push_back(node);

  current_point_property->setVector(position);
  current_theta_property->setFloat(float(theta));

  positions.push_back(WaypointPlanner::Position(float(x), float(y), 1, float(theta)));
  addProperties();
}

int WaypointPlanner::processMouseEvent(rviz::ViewportMouseEvent& event) {
  const int res = PoseTool::processMouseEvent(event);
  return res & (~Finished);
}

int WaypointPlanner::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) {
  if (is_in_loop) {
    status = "Request denied, the program waits for drone to get to the first waypoint";
    setStatus(QString(status.c_str()));
    return Render;
  }
  PoseTool::processKeyEvent(event, panel);
  ROS_INFO("[Waypoint planner]: Received key %d", (int)event->key());
  // Sends added waypoints to service
  if (event->key() == KEY_ENTER) {
    if (loop_property->getBool()) {
      std::thread t = std::thread([this] { processLoop(); });
      t.detach();
    } else {
      sendWaypoints();
    }

    // Note: the tool does not exit. Good thing that it doesn't have to XD
    return Render;
  }
  if (event->key() == KEY_DELETE && !positions.empty()) {
    positions.pop_back();
    std::size_t index = point_properties.size() - 2;
    point_properties.erase(point_properties.begin() + index);
    angle_properties.erase(angle_properties.begin() + index);
    delete pose_nodes.back();
    pose_nodes.pop_back();
    if (!arrows.empty()) {
      delete arrows.back();
      arrows.pop_back();
    }
    if (!axes.empty()) {
      delete axes.back();
      axes.pop_back();
    }
    ROS_INFO("[Waypoint planner]: Removing index = %ld", DEFAULT_PROPERTIES_NUM + positions.size());
    getPropertyContainer()->removeChildren(int(DEFAULT_PROPERTIES_NUM + positions.size()), 1);
    current_property->setName("Position " + QString::number(positions.size() + 1));
  }
  return Render;
}

void WaypointPlanner::processLoop() {
  is_in_loop = true;
  // Set general attributes
  mrs_msgs::Vec4 srv;
  auto           point = generateReferences(context_->getFrameManager()->getFixedFrame(), 1);
  srv.request.goal[0]  = point[0].position.x;
  srv.request.goal[1]  = point[0].position.y;
  srv.request.goal[2]  = point[0].position.z;
  srv.request.goal[3]  = point[0].heading;

  // Temporaly set client to send requests to different service
  client = node_handler.serviceClient<mrs_msgs::Vec4>(std::string("/") + drone_name_property->getStdString() + std::string("/control_manager/goto_fcu"));

  // Make the call
  client.call(srv);
  client =
      node_handler.serviceClient<mrs_msgs::PathSrv>(std::string("/") + drone_name_property->getStdString() + std::string("/") + topic_property->getStdString());
  if (srv.response.success) {
    ROS_INFO("[Waypoint planner]: Looping: initial call processed successfully");
    status = "Looping: initial call processed successfully";
    setStatus(QString(status.c_str()));
  } else {
    ROS_INFO("[Waypoint planner]: Loop: initial call failed: %s", srv.response.message.c_str());
    status = "Looping: initial call failed: " + srv.response.message + " Try checking, if drone name is correct.";
    setStatus(QString(status.c_str()));

    is_in_loop = false;
    return;
  }

  // Wait until drone is on the goal
  std::string drone_name = drone_name_property->getStdString();
  std::string topic      = drone_name + std::string("/control_manager/diagnostics");
  bool        is_ok      = true;
  ;
  while (true) {
    const auto& info_msg = ros::topic::waitForMessage<mrs_msgs::ControlManagerDiagnostics>(topic, node_handler, ros::Duration(1.0));
    if (info_msg.get() == nullptr) {
      ROS_INFO("[Waypoint planner]: Diagnostics data are not received");
      continue;
    }
    if (!info_msg->flying_normally) {
      is_ok = false;
      break;
    }
    if (!info_msg->tracker_status.have_goal) {
      break;
    }
  }

  is_in_loop = false;
  if (!is_ok) {
    return;
  }

  Position final_pos = Position(positions[0].x, positions[0].y, positions[0].z, positions[0].theta);
  positions.push_back(final_pos);

  sendWaypoints();
}

void WaypointPlanner::sendWaypoints() {
  // Set general attributes
  mrs_msgs::PathSrv srv;
  srv.request.path.header.frame_id   = "fcu";
  srv.request.path.use_heading       = use_heading_property->getBool();
  srv.request.path.fly_now           = fly_now_property->getBool();
  srv.request.path.stop_at_waypoints = stop_at_waypoints_property->getBool();
  srv.request.path.loop              = loop_property->getBool();

  // Transform positions
  srv.request.path.points = generateReferences(context_->getFrameManager()->getFixedFrame(), -1);

  // Make the call
  client.call(srv);
  if (srv.response.success) {
    ROS_INFO("[Waypoint planner]: Call processed successfully");
    status = "Call processed successfully";
    setStatus(QString(status.c_str()));
  } else {
    ROS_INFO("[Waypoint planner]: Call failed: %s", srv.response.message.c_str());
    status = "Call failed: " + srv.response.message + " Try checking, if drone name is correct.";
    setStatus(QString(status.c_str()));
  }

  // Clean-up
  positions.clear();
  point_properties.clear();
  angle_properties.clear();
  for (int i = 0; i < pose_nodes.size(); i++) {
    delete pose_nodes[i];
  }
  pose_nodes.clear();
  for (auto ax : axes) {
    delete ax;
  }
  axes.clear();
  for (auto arrow : arrows) {
    delete arrow;
  }
  arrows.clear();
  getPropertyContainer()->removeChildren(DEFAULT_PROPERTIES_NUM, -1);
  addProperties();
}


std::vector<mrs_msgs::Reference> WaypointPlanner::generateReferences(std::string current_frame, int num) {
  num = num != -1 ? num : int(positions.size());

  // Note: ref.position is reset to 0 for some reason. Although it does not matter now, it is weird
  const std::string                drone_name = drone_name_property->getStdString();
  const std::string                topic      = drone_name + std::string("/odometry/odom_main");
  std::vector<mrs_msgs::Reference> res(num);

  // Make transformer
  const std::string goal_frame = drone_name + "/fcu";
  const auto&       tf         = transformer.getTransform(current_frame, goal_frame);
  if (!tf) {
    ROS_INFO("[Waypoint planner]: No transformation found. No data sent");
    return std::vector<mrs_msgs::Reference>{};
  }

  // Transforming
  for (int i = 0; i < positions.size() && i < num; i++) {
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
    const auto& point_transformed = transformer.transform(pose, tf.value());
    if (!point_transformed) {
      ROS_INFO("[Waypoint planner]: Unable to transform cmd reference from %s to %s at time %.6f. No data sent", current_frame.c_str(), goal_frame.c_str(),
               ros::Time::now().toSec());
      return std::vector<mrs_msgs::Reference>{};
    }

    // Convert transformed heading from quaternion to XY projection
    pose                  = point_transformed.value();
    const double rotation = mrs_lib::AttitudeConverter(pose.orientation).getHeading();

    // Save results
    auto& ref      = res[i];
    ref.position.x = pose.position.x;
    ref.position.y = pose.position.y;
    ref.position.z += height_offset_property->getFloat();
    ref.heading = rotation;

    ROS_INFO("[Waypoint planner]: %.2f, %.2f, %.8f", ref.position.x, ref.position.y, ref.position.z);
  }

  ROS_INFO("[Waypoint planner]: All transformations went successfully.");
  return res;
}

WaypointPlanner::~WaypointPlanner() {
  for (const auto& node : pose_nodes) {
    scene_manager_->destroySceneNode(node);
  }
}
}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::WaypointPlanner, rviz::Tool)
