#ifndef DRONE_ENTITY_H
#define DRONE_ENTITY_H

#include <ros/ros.h>

#include <mrs_msgs/UavStatus.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <QString>
#include <QAction>
#include <QMenu>

#include <vector>
#include <map>

namespace mrs_rviz_plugins{

class DroneEntity{
public:
  DroneEntity(const std::string name);

  // TODO: optimize the following so they return read-only copy
  std::vector<std::string> getConstraints();
  std::vector<std::string> getGains();
  std::vector<std::string> getConrollers();
  std::vector<std::string> getTrackers();
  std::vector<std::string> getOdomSources();
  std::vector<std::string> getLatEstimators();
  std::vector<std::string> getAltEstimators();
  std::vector<std::string> getHdgEstimators();

  enum EntryIndex{
    LAND              = 0,
    LAND_HOME         = 1,
    TAKEOFF           = 2,
    SET_CONSTRAINT    = 3,
    SET_GAIN          = 4,
    SET_CONTROLLER    = 5,
    SET_TRACKER       = 6,
    SET_ODOM_SOURCE   = 7,
    SET_LAT_ESTIMATOR = 8,
    SET_ALT_ESTIMATOR = 9,
    SET_HDG_ESTIMATOR = 10,
    SIZE              = 11
  };
protected:
  void updateMenu();
  void statusCallback(const mrs_msgs::UavStatusConstPtr& msg);
  // Compares the vectors. If they contain different values, changes current so it is the same as actual
  // Return true if current has been changed, false otherwise.
  bool compareAndUpdate(std::vector<std::string>& current, const std::vector<std::string>& actual);

  //|-------------------- Menu Callbacks --------------------|
  static void land           (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void landHome       (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void takeoff        (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setConstraint  (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setGain        (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setController  (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setTracker     (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setOdomSource  (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setLatEstimator(std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setAltEstimator(std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  static void setHdgEstimator(std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  //|---------------------- Attributes ----------------------|
  std::string name;

  // Menu options
  std::vector<std::string> constraints{};
  std::vector<std::string> gains{};
  std::vector<std::string> controllers{};
  std::vector<std::string> trackers{};
  std::vector<std::string> odom_lat_sources{};  // Seems to be the same as odom source (https://github.com/ctu-mrs/mrs_uav_status/blob/78f9ee8fce216a810d15d14d7a4e479e2c41d503/src/status.cpp#L872C65-L872C65)
  std::vector<std::string> odom_alt_sources{};
  std::vector<std::string> odom_hdg_sources{};

  // No method to get entries from menu handler, so we have to save them on inserting
  typedef interactive_markers::MenuHandler::EntryHandle MenuEntryHandle;
  std::vector<MenuEntryHandle> entries;

  interactive_markers::InteractiveMarkerServer* server;
  interactive_markers::MenuHandler* menu_handler = nullptr;

  ros::NodeHandle nh;
  ros::Subscriber status_subscriber;
  
};//class DroneEntity

}//namespace mrs_rviz_plugins


#endif