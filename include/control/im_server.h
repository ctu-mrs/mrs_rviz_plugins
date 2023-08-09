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

  typedef interactive_markers::MenuHandler::EntryHandle MenuEntryHandle;

  interactive_markers::InteractiveMarkerServer* server;
  std::map<std::string, interactive_markers::MenuHandler*> menu_handlers;

  // No method to get entries from menu handler, so we have to save them on inserting
  std::map<std::string, std::vector<MenuEntryHandle>> entries;

  enum EntryIndex{
    LAND = 0,
    LAND_HOME = 1,
    TAKEOFF = 2,
    SET_CONSTRAINTS = 3,
    SET_GAINS = 4,
    SET_CONTROLLER = 5,
    SET_TRACKER = 6,
    SET_ODOM_SOURCE = 7,
    SET_LAT_ESTIMATOR = 8,
    SET_ALT_ESTIMATOR = 9,
    SET_HDG_ESTIMATOR = 10
  };

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

  std::vector<std::string> constraints;
  std::vector<std::string> gains;
  std::vector<std::string> controllers;
  std::vector<std::string> trackers;
  std::vector<std::string> odom_lat_sources;  // Seems to be the same as odom source (https://github.com/ctu-mrs/mrs_uav_status/blob/78f9ee8fce216a810d15d14d7a4e479e2c41d503/src/status.cpp#L872C65-L872C65)
  std::vector<std::string> odom_alt_sources;
  std::vector<std::string> osom_hdg_sources;

}; // class ClientWrapper

}// namespace mrs_rviz_plugins

#endif