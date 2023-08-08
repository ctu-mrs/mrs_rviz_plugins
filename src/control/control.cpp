#include "control/control.h"

#include <visualization_msgs/InteractiveMarker.h>

#include <rviz/visualization_manager.h>

namespace mrs_rviz_plugins{

ControlTool::ControlTool(){
  shortcut_key_ = 'c';
  server = new ImServer();
}

void ControlTool::onInitialize(){
  rviz::InteractiveMarkerDisplay* dis = new rviz::InteractiveMarkerDisplay();
  dynamic_cast<rviz::VisualizationManager*>(context_)->addDisplay(dis, true);

  dis->setName(QString("Control Display"));
  dis->setTopic(QString("control/update"), QString("visualization_msgs/InteractiveMarkerUpdate"));

  // TODO: /mrs_drone_spawner/diagnostics seems to have that info and it is more efficient
  // Note: it may also be put in timer for example, check a new drone every 3 secs
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
    ROS_INFO("[Control tool]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Initialize all the markers
  for(auto name : drone_names){
    server->addDrone(name);
  }
}

void ControlTool::activate(){
}

void ControlTool::deactivate(){

}

int ControlTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
  
  return Render;
}

}// namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::ControlTool, rviz::Tool)