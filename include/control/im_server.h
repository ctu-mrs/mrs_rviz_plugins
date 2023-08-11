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
  void flyForwardSelected();
  void flyBackwardSelected();
  void flyRightSelected();
  void flyLeftSelected();
  void flyUpSelected();
  void flyDownSelected();
  void rotateClockwiseSelected();
  void rotateAntiClockwiseSelected();
  bool select(std::vector<std::string> names);

  boost::shared_ptr<QMenu> getMenu();

protected Q_SLOTS:
  void landNow();
  void landHome();
  void takeoffNow();

protected:

  // Lambda functions are created out of these functions and connected to corresponding actions
  void setConstraints(std::string value);
  void setGains(std::string value);
  void setControllers(std::string value);
  void setTrackers(std::string value);
  void setOdomSources(std::string value);
  void setLatEstimators(std::string value);
  void setAltEstimators(std::string value);
  void setHdgEstimators(std::string value);

  // TODO: maybe delete these attributes from here? (could be loval vars only)
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
  std::map<std::string, DroneEntity*> drones;

  // Drones to be affected by global menu
  std::vector<DroneEntity*> selected_drones;

}; // class ClientWrapper

}// namespace mrs_rviz_plugins

#endif