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

#include <mrs_msgs/NodeCpuLoad.h>
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

#define CM_INDEX 0
#define ODOM_INDEX 0
#define GEN_INFO_INDEX 1
#define MAVROS_INDEX 1
#define TOPIC_RATE_INDEX 2
#define CUSTOM_STR_INDEX 3
#define NODE_STATS_INDEX 4

namespace mrs_rviz_plugins
{

class StatusDisplay : public rviz::Display {
Q_OBJECT

public:
  StatusDisplay();
  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

  // A helper to clear this display back to the initial state.
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

  void setPosition(int x, int y);
  bool isInRegion(int x, int y);
  void movePosition(int x, int y);
  int getX() { return  display_pos_x; }
  int getY() { return  display_pos_y; }

  rviz::BoolProperty* rosnode_shitlist_property;

private Q_SLOTS:
  void nameUpdate();
  void topLineUpdate();
  void controlManagerUpdate();
  void odometryUpdate();
  void computerLoadUpdate();
  void mavrosStateUpdate();
  void topicRatesUpdate();
  void customStrUpdate();
  void nodeStatsUpdate();

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
  void processTopLine(const mrs_msgs::UavStatusConstPtr& msg);
  void processControlManager(const mrs_msgs::UavStatusConstPtr& msg);
  void processOdometry(const mrs_msgs::UavStatusConstPtr& msg);
  void processGeneralInfo(const mrs_msgs::UavStatusConstPtr& msg);
  void processMavros(const mrs_msgs::UavStatusConstPtr& msg);
  void processCustomTopics(const mrs_msgs::UavStatusConstPtr& msg);
  void processCustomStrings(const mrs_msgs::UavStatusConstPtr& msg);
  void processNodeStats(const mrs_msgs::UavStatusConstPtr& msg);

  // Drawing methods
  void drawTopLine();
  void drawControlManager();
  void drawOdometry();
  void drawGeneralInfo();
  void drawMavros();
  void drawCustomTopicRates();
  void drawCustomStrings();
  void drawNodeStats();
  
  // Subscribers
  ros::Subscriber uav_status_sub;
  ros::Subscriber gain_manager_sub;
  ros::Subscriber constraint_manager_sub;
  
  // Properties
  rviz::EditableEnumProperty* uav_name_property;
  rviz::BoolProperty* top_line_property;
  rviz::BoolProperty* control_manager_property;
  rviz::BoolProperty* odometry_property;
  rviz::BoolProperty* computer_load_property;
  rviz::BoolProperty* mavros_state_property;
  rviz::BoolProperty* topic_rates_property;
  rviz::BoolProperty* custom_str_property;
  rviz::BoolProperty* node_stats_property;
  rviz::IntProperty*  debug_property;

  // Individual overlays
  jsk_rviz_plugins::OverlayObject::Ptr top_line_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr contol_manager_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr odometry_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr general_info_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr mavros_state_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr topic_rates_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr custom_strings_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr rosnode_stats_overlay;

  // Top line data:
  std::string uav_name;
  std::string uav_type;
  std::string nato_name;
  bool collision_avoidance_enabled;
  bool avoiding_collision;
  bool automatic_start_can_takeoff;
  int num_other_uavs;
  int secs_flown;
  bool top_line_update_required = true;

  // Controller data:
  double       avg_controller_rate;
  bool         null_tracker;
  double       controller_rate = 0;
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

  // Node stats
  mrs_msgs::NodeCpuLoad node_cpu_load_vec;
  double cpu_load_total;
  bool node_stats_update_required;


  ros::NodeHandle nh;
  static int display_number;
  int id; 

  // | --------------------- Default values --------------------- |
  const QColor RED_COLOR      = QColor(255, 0, 0, 255);
  const QColor YELLOW_COLOR   = QColor(255, 255, 0, 255);
  const QColor NO_COLOR       = QColor(0, 0, 0, 0);

  // | ---------------------- Layout data ----------------------- |
  std::vector<bool> present_columns{true, true, true, true, false};
  int display_pos_x     = 0;
  int display_pos_y     = 0;
  int cm_pos_y          = 23;
  int odom_pos_y        = 86;
  int gen_info_pos_x    = 233;
  int gen_info_pos_y    = 23;
  int mavros_pos_x      = 233;
  int mavros_pos_y      = 86;
  int topic_rate_pos_x  = 466;
  int topic_rate_pos_y  = 23;
  int custom_str_pos_x  = 699;
  int custom_str_pos_y  = 0;
  int custom_str_height = 206;
  int node_stats_pos_x  = 932;
  int node_stats_pos_y  = 0;
  int node_stats_height = 206;

  bool global_update_required = true;
};

} // namespace mrs_rviz_plugins


#endif