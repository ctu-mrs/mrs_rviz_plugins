#ifndef MRS_STATUS_DISPLAY_H
#define MRS_STATUS_DISPLAY_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <QStaticText>
#include <QPainter>
#include <QColor>
#include <QTextStream>

#include <rviz/properties/bool_property.h>

#include "uav_status/overlay_utils.h"

#include <mrs_msgs/CustomTopic.h>
#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>

#include <rviz/message_filter_display.h>

#define NORMAL 100
#define FIELD 101
#define GREEN 102
#define RED 103
#define YELLOW 104

namespace mrs_rviz_plugins
{

class StatusDisplay : public rviz::Display {
Q_OBJECT

public:
  StatusDisplay();
  void onInitialize() override;

  // A helper to clear this display back to the initial state.
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

  rviz::BoolProperty* rosnode_shitlist_property;

private Q_SLOTS:
  void nameUpdate();
  void tmpUpdate();
private:
  // Helper function
  QColor getColor(int code){
    if(code == NORMAL) return NO_COLOR;
    if(code == GREEN) return NO_COLOR;
    if(code == RED)   return RED_COLOR;
    return YELLOW_COLOR;
  }

  // Subscriber callback
  void uavStatusCb(const mrs_msgs::UavStatusConstPtr& msg);

  // New message processing methods
  void processControlManager(const mrs_msgs::UavStatusConstPtr& msg);
  void processOdometry(const mrs_msgs::UavStatusConstPtr& msg);
  void processGeneralInfo(const mrs_msgs::UavStatusConstPtr& msg);
  void processMavros(const mrs_msgs::UavStatusConstPtr& msg);
  void processCustomTopics(const mrs_msgs::UavStatusConstPtr& msg);
  void processCustomStrings(const mrs_msgs::UavStatusConstPtr& msg);

  // Drawing methods
  void drawControlManager();
  void drawOdometry();
  void drawGeneralInfo();
  void drawMavros();
  void drawCustomTopicRates();
  void drawCustomStrings();
  
  // Subscribers
  ros::Subscriber uav_status_sub;
  ros::Subscriber gain_manager_sub;
  ros::Subscriber constraint_manager_sub;
  
  // Properties
  rviz::StringProperty* uav_name_property;
  rviz::BoolProperty* contol_manager_property;
  rviz::BoolProperty* odometry_property;
  rviz::BoolProperty* computer_load_property;
  rviz::BoolProperty* mavros_state_property;
  rviz::BoolProperty* first_unknown_property;
  rviz::BoolProperty* second_unknown_property;
  rviz::IntProperty*  debug_property;

  // Individual overlays
  jsk_rviz_plugins::OverlayObject::Ptr contol_manager_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr odometry_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr general_info_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr mavros_state_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr topic_rates_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr custom_strings_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr rosnode_shitlist_overlay;

  // Controller data:
  double       avg_controller_rate;
  bool         null_tracker;
  double       controller_rate;
  std::string  curr_controller  = "!NO DATA!";
  std::string  curr_tracker     = "!NO DATA!";
  std::string  curr_gains;
  std::string  curr_constraints;
  bool         callbacks_enabled;
  bool         has_goal;
  bool         cm_update_required = true;

  // Odometry data
  double avg_odom_rate;
  double color;
  double heading;
  double state_x;
  double state_y;
  double state_z;
  double cmd_x;
  double cmd_y;
  double cmd_z;
  double cmd_hdg;
  std::string odom_frame = "!NO DATA!";
  std::string curr_estimator_hori = "!NO DATA!";
  std::string curr_estimator_vert = "!NO DATA!";
  std::string curr_estimator_hdg = "!NO DATA!";
  bool odom_update_required = true;

  // General info
  double cpu_load;
  double cpu_freq;
  double ram_free;
  double total_ram;
  double disk_free;
  bool comp_state_update_required = true;

  // Mavros
  double      mavros_rate = 0;
  double      state_rate;
  double      cmd_rate;
  double      battery_rate;
  bool        mavros_gps_ok;
  bool        armed;
  std::string mode;
  double      battery_volt;
  double      battery_curr;
  double      battery_wh_drained;
  double      thrust;
  double      mass_estimate;
  double      mass_set;
  double      gps_qual;
  double      mag_norm;
  double      mag_norm_rate;
  bool        mavros_update_required;

  // Custom topics
  std::vector<mrs_msgs::CustomTopic> custom_topic_vec;
  bool topics_update_required;

  // Custom string outputs
  std::vector<std::string> custom_string_vec;
  bool string_update_required;


  ros::NodeHandle nh;

  // | --------------------- Default values --------------------- |
  const QColor RED_COLOR      = QColor(255, 0, 0, 255);
  const QColor YELLOW_COLOR   = QColor(255, 255, 0, 255);
  const QColor NO_COLOR       = QColor(0, 0, 0, 0);
};

} // namespace mrs_rviz_plugins


#endif