#ifndef CLIENT_WRAPPER_H
#define CLIENT_WRAPPER_H

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <interactive_markers/interactive_marker_client.h>
#endif

#include <rviz/display_context.h>
#include <rviz/properties/status_list.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/bool_property.h>

#include <rviz/default_plugin/interactive_markers/interactive_marker.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <map>
#include <set>

namespace mrs_rviz_plugins{

class ImServer{
public:
  ImServer();

  void addDrone(const std::string name);

protected:
  void updatePositions();
  interactive_markers::InteractiveMarkerServer* server;
  std::map<std::string, interactive_markers::MenuHandler*> menu_handlers;



}; // class ClientWrapper

}// namespace mrs_rviz_plugins

#endif