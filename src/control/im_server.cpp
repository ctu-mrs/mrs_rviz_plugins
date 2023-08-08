#include "control/im_server.h"

using namespace visualization_msgs;

namespace mrs_rviz_plugins{

ImServer::ImServer(){
  server = new interactive_markers::InteractiveMarkerServer("control", "", true);
}

void ImServer::addDrone(const std::string name) {
  if(menu_handlers.find(name) != menu_handlers.end()){
    return;
  }

  // Regular marker
  Marker marker_msg;
  marker_msg.type = Marker::CUBE;
  marker_msg.scale.x = 0.45;
  marker_msg.scale.y = 0.45;
  marker_msg.scale.z = 0.45;
  marker_msg.color.r = 0.5;
  marker_msg.color.g = 0.5;
  marker_msg.color.b = 0.5;
  marker_msg.color.a = 1.0;
  marker_msg.id = 1;
  marker_msg.pose.position.x = 0;
  marker_msg.pose.position.y = 0;
  marker_msg.pose.position.z = 0;

  // Control
  InteractiveMarkerControl control_msg;
  control_msg.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_msg.always_visible = true;
  control_msg.always_visible = true;
  control_msg.independent_marker_orientation = true;
  control_msg.name = "control1";
  control_msg.orientation_mode = InteractiveMarkerControl::INHERIT;
  control_msg.markers.push_back(marker_msg);

  // Interactive marker
  InteractiveMarker int_marker_msg;
  int_marker_msg.header.frame_id = name + "/fcu";
  int_marker_msg.pose.position.y = 0;
  int_marker_msg.scale = 1;
  int_marker_msg.name = name + " marker";
  int_marker_msg.controls.push_back(control_msg);
  server->insert(int_marker_msg);


  // Menu
  interactive_markers::MenuHandler* menu_handler = new interactive_markers::MenuHandler();
  
  // TODO: add entries
  menu_handler->insert("First");
  menu_handler->insert("Second");
  
  
  
  if(!menu_handler->apply(*server, int_marker_msg.name)){
    ROS_INFO("Menu not applied");
  }
  server->applyChanges();
  menu_handlers.insert(std::make_pair(name, menu_handler));
}

} // namespace mrs_rviz_plugins