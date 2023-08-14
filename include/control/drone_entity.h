#ifndef DRONE_ENTITY_H
#define DRONE_ENTITY_H

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarker.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <mrs_lib/service_client_handler.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/String.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <ros/ros.h>

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
  std::vector<std::string> getControllers();
  std::vector<std::string> getTrackers();
  std::vector<std::string> getOdomSources();
  std::vector<std::string> getLatEstimators();
  std::vector<std::string> getAltEstimators();
  std::vector<std::string> getHdgEstimators();
  bool getNullTracker();

  void setServiceNumCalls(const int value);

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

  // | -------------------- API for ImServer -------------------- |
  bool land           ();
  bool landHome       ();
  bool takeoff        ();
  bool setConstraint  (std::string value);
  bool setGain        (std::string value);
  bool setController  (std::string value);
  bool setTracker     (std::string value);
  bool setOdomSource  (std::string value);
  bool setLatEstimator(std::string value);
  bool setAltEstimator(std::string value);
  bool setHdgEstimator(std::string value);
  bool flyForward();
  bool flyBackward();
  bool flyRight();
  bool flyLeft();
  bool flyUp();
  bool flyDown();
  bool rotateClockwise();
  bool rotateAntiClockwise();

protected:
  void updateMenu();
  void statusCallback(const mrs_msgs::UavStatusConstPtr& msg);
  void newSeviceCallback(const std_msgs::StringConstPtr& msg);

  // Compares the vectors. If they contain different values, changes current so it is the same as actual
  // Return true if current has been changed, false otherwise.
  bool compareAndUpdate(std::vector<std::string>& current, const std::vector<std::string>& actual);

  // | --------------------- Menu Callbacks --------------------- |
  void land           (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void landHome       (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void takeoff        (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setConstraint  (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setGain        (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setController  (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setTracker     (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setOdomSource  (std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setLatEstimator(std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setAltEstimator(std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setHdgEstimator(std::string value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processCustomService(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // | ------------------------ Services ------------------------ |
  mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> service_goto_reference;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> service_land;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> service_land_home;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> service_takeoff;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_constraints;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_gains;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_controller;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_tracker;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_odometry_source;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_lat_estimator;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_alt_estimator;
  mrs_lib::ServiceClientHandler<mrs_msgs::String> service_set_hdg_estimator;
  std::vector<mrs_lib::ServiceClientHandler<std_srvs::Trigger>> custom_services;
  std::vector<std::string> custom_service_names;
  int service_num_calls = 20;
  double service_delay  = 0.1;
  
  // | -------------------------- Menu -------------------------- |
  std::vector<std::string> constraints{};
  std::vector<std::string> gains{};
  std::vector<std::string> controllers{};
  std::vector<std::string> trackers{};
  std::vector<std::string> odom_lat_sources{};  // Seems to be the same as odom source (https://github.com/ctu-mrs/mrs_uav_status/blob/78f9ee8fce216a810d15d14d7a4e479e2c41d503/src/status.cpp#L872C65-L872C65)
  std::vector<std::string> odom_alt_sources{};
  std::vector<std::string> odom_hdg_sources{};
  bool null_tracker; // Responsible for showing Land vs Takeoff

  // No method to get entries from menu handler, so we have to save them on inserting
  typedef interactive_markers::MenuHandler::EntryHandle MenuEntryHandle;
  std::vector<MenuEntryHandle> entries;

  interactive_markers::InteractiveMarkerServer* server;
  interactive_markers::MenuHandler* menu_handler = nullptr;

  // | ----------------------- Attributes ----------------------- |
  std::string name;

  ros::NodeHandle nh;
  ros::Subscriber status_subscriber;
  ros::Subscriber custom_services_subsrciber;
  
};//class DroneEntity

}//namespace mrs_rviz_plugins


#endif