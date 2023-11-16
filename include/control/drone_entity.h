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
#include <mrs_msgs/HwApiPositionCmd.h>
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

namespace mrs_rviz_plugins
{

class DroneEntity {
public:
  DroneEntity(const std::string& name);
  ~DroneEntity();

  std::vector<std::string> getConstraints();
  std::vector<std::string> getGains();
  std::vector<std::string> getControllers();
  std::vector<std::string> getTrackers();
  std::vector<std::string> getOdomSources();
  std::vector<std::string> getLatEstimators();
  std::vector<std::string> getAltEstimators();
  std::vector<std::string> getHdgEstimators();
  bool                     getNullTracker();

  void setServiceNumCalls(const int value);

  enum EntryIndex
  {
    LAND           = 0,
    LAND_HOME      = 1,
    TAKEOFF        = 2,
    SET_CONSTRAINT = 3,
    SET_GAIN       = 4,
    SET_CONTROLLER = 5,
    SET_TRACKER    = 6,
    SET_ESTIMATOR  = 7,
    SIZE           = 8,
  };

  // | -------------------- API for ImServer -------------------- |
  bool land();
  bool landHome();
  bool takeoff();
  bool setConstraint(const std::string& value);
  bool setGain(const std::string& value);
  bool setController(const std::string& value);
  bool setTracker(const std::string& value);
  bool setEstimator(const std::string& value);
  bool flyForward(bool global_mode_on);
  bool flyBackward(bool global_mode_on);
  bool flyRight(bool global_mode_on);
  bool flyLeft(bool global_mode_on);
  bool flyUp();
  bool flyDown();
  bool rotateClockwise();
  bool rotateAntiClockwise();

protected:
  // Makes menu correspond to the current state of uav
  // (land/takeoff options, custom services)
  void updateMenu();

  // Compares the vectors. If they contain different values, changes current so it is the same as actual
  // Return true if current has been changed, false otherwise.
  bool compareAndUpdate(std::vector<std::string>& current, const std::vector<std::string>& actual);

  // | --------------------- Menu Callbacks --------------------- |
  void land(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void landHome(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void takeoff(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void setConstraint(const std::string& value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void setGain(const std::string& value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void setController(const std::string& value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void setTracker(const std::string& value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void setEstimator(const std::string& value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void processCustomService(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // | --------------------- State Callbacks -------------------- |
  void statusCallback(const mrs_msgs::UavStatusConstPtr& msg);
  void newSeviceCallback(const std_msgs::StringConstPtr& msg);
  void positionCmdCallback(const mrs_msgs::HwApiPositionCmdConstPtr& msg);

  // | ------------------------ Services ------------------------ |
  mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>  service_goto_reference;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>              service_land;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>              service_land_home;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>              service_takeoff;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>               service_set_constraints;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>               service_set_gains;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>               service_set_controller;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>               service_set_tracker;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>               service_set_estimator;
  std::vector<mrs_lib::ServiceClientHandler<std_srvs::Trigger>> custom_services;
  std::vector<std::string>                                      custom_service_names;
  int                                                           service_num_calls = 20;
  double                                                        service_delay     = 0.1;

  // | -------------------------- Menu -------------------------- |
  std::vector<std::string> constraints{};
  std::vector<std::string> gains{};
  std::vector<std::string> controllers{};
  std::vector<std::string> trackers{};
  std::vector<std::string>
      odom_lat_sources{};  // Seems to be the same as odom source
                           // (https://github.com/ctu-mrs/mrs_uav_status/blob/78f9ee8fce216a810d15d14d7a4e479e2c41d503/src/status.cpp#L872C65-L872C65)
  std::vector<std::string> odom_alt_sources{};
  std::vector<std::string> odom_hdg_sources{};


  // | ----------------------- Attributes ----------------------- |
  std::string                name;
  bool                       null_tracker;  // Responsible for showing Land vs Takeoff
  mrs_msgs::HwApiPositionCmd last_position;

  // No method to get entries from menu handler, so we have to save them on inserting
  typedef interactive_markers::MenuHandler::EntryHandle MenuEntryHandle;
  std::vector<MenuEntryHandle>                          entries;

  interactive_markers::InteractiveMarkerServer* server       = nullptr;
  interactive_markers::MenuHandler*             menu_handler = nullptr;

  ros::NodeHandle nh;
  ros::Subscriber status_subscriber;
  ros::Subscriber custom_services_subsrciber;
  ros::Subscriber position_cmd_subscriber;

};  // class DroneEntity

}  // namespace mrs_rviz_plugins


#endif
