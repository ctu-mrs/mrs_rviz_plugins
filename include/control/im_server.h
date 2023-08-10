#ifndef IM_SERVER_H
#define IM_SERVER_H

#include "control/drone_entity.h"

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

#include <ros/ros.h>

#include <QString>
#include <QAction>
#include <QMenu>

#include <map>
#include <set>

namespace mrs_rviz_plugins{

class ImServer : public QObject{

  Q_OBJECT
public:
  ImServer();

  void addDrone(const std::string name);

  boost::shared_ptr<QMenu> getMenu(std::vector<std::string>& names);

protected Q_SLOTS:
  void landNow();
  void landHome();
  void takeoffNow();

protected:
  // Lambda functions are created out of these functions and connected to corresponding actions
  void setConstraints(std::string value);
  void setGains(std::string value);
  void setController(std::string value);
  void setTracker(std::string value);
  void setOdomSource(std::string value);
  void setLatEstimator(std::string value);
  void setAltEstimator(std::string value);
  void setHdgEstimator(std::string value);

  boost::shared_ptr<QMenu> menu;
  QAction* land;
  QAction* land_home;
  QAction* takeoff;
  QMenu* set_constraints;
  QMenu* set_gains;
  QMenu* set_controller;
  QMenu* set_tracker;
  QMenu* set_odom_source;
  QMenu* set_lat_estimator;
  QMenu* set_alt_estimator;
  QMenu* set_hdg_estimator;

  // Note: callbacks of DroneEntity do not work if the instance is not allocated on the heap
  std::vector<DroneEntity*> drones;

  // Drones to be affected by global menu
  std::vector<std::string> selected_drones;

}; // class ClientWrapper

}// namespace mrs_rviz_plugins

#endif