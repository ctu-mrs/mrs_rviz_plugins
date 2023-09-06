#ifndef MRS_STATUS_DISPLAY_H
#define MRS_STATUS_DISPLAY_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <utility>
#include <algorithm>
#include <unordered_map>

#include <QTextStream>
#include <QStaticText>
#include <QPainter>
#include <QColor>

#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/properties/property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/message_filter_display.h>

#include "uav_status/overlay_utils.h"

#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/NodeCpuLoad.h>
#include <mrs_msgs/CustomTopic.h>
#include <mrs_msgs/UavStatus.h>


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
  ~StatusDisplay();
  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

  // A helper to clear this display back to the initial state.
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

  // Methods for OverlayPickerTool
  void setPosition(const int x, const int y);
  bool isInRegion(const int x, const int y);
  void movePosition(const int x, const int y);

  int getX() {
    return display_pos_x;
  }
  int getY() {
    return display_pos_y;
  }

private Q_SLOTS:
  // Property change callbacks
  void nameUpdate();
  void colorFgUpdate();
  void colorBgUpdate();
  void topLineUpdate();
  void controlManagerUpdate();
  void odometryUpdate();
  void computerLoadUpdate();
  void mavrosStateUpdate();
  void topicRatesUpdate();
  void customStrUpdate();
  void nodeStatsUpdate();

private:
  // Helper functions
  std::pair<int, int> getSpawnCoordinates(rviz::Property* property);
  std::pair<int, int> getBottomLine();
  // Meant to take "Global Options" property only!
  void                setTextColor(rviz::Property* property);
  bool                getIsInited() {
    return is_inited;
  }
  QColor getColor(const int code) {
    if (code == NORMAL)
      return NO_COLOR;
    if (code == GREEN)
      return GREEN_COLOR;
    if (code == RED)
      return RED_COLOR;
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

  // Properties
  rviz::EditableEnumProperty* uav_name_property;
  rviz::ColorProperty*        text_color_property;
  rviz::ColorProperty*        bg_color_property;
  rviz::BoolProperty*         top_line_property;
  rviz::BoolProperty*         control_manager_property;
  rviz::BoolProperty*         odometry_property;
  rviz::BoolProperty*         computer_load_property;
  rviz::BoolProperty*         mavros_state_property;
  rviz::BoolProperty*         topic_rates_property;
  rviz::BoolProperty*         custom_str_property;
  rviz::BoolProperty*         node_stats_property;

  // Individual overlays
  jsk_rviz_plugins::OverlayObject::Ptr top_line_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr contol_manager_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr odometry_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr general_info_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr mavros_state_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr topic_rates_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr custom_strings_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr rosnode_stats_overlay;

  // | --------------------- UavStatus data --------------------- |
  // Top line data:
  std::string uav_name                    = "";
  std::string uav_type                    = "";
  std::string nato_name                   = "";
  bool        collision_avoidance_enabled = false;
  bool        avoiding_collision          = false;
  bool        automatic_start_can_takeoff = false;
  int         num_other_uavs              = 0;
  int         secs_flown                  = 0;
  bool        top_line_update_required    = true;

  // Controller data:
  double      avg_controller_rate = 0;
  bool        null_tracker        = true;
  double      controller_rate     = 0;
  std::string curr_controller     = "!NO DATA!";
  std::string curr_tracker        = "!NO DATA!";
  std::string curr_gains          = "";
  std::string curr_constraints    = "";
  bool        callbacks_enabled   = false;
  bool        has_goal            = false;
  bool        cm_update_required  = true;

  // Odometry data
  double      avg_odom_rate        = 0;
  double      color                = 0;
  double      heading              = 0;
  double      state_x              = 0;
  double      state_y              = 0;
  double      state_z              = 0;
  double      cmd_x                = 0;
  double      cmd_y                = 0;
  double      cmd_z                = 0;
  double      cmd_hdg              = 0;
  std::string odom_frame           = "!NO DATA!";
  std::string curr_estimator_hori  = "!NO DATA!";
  std::string curr_estimator_vert  = "!NO DATA!";
  std::string curr_estimator_hdg   = "!NO DATA!";
  bool        odom_update_required = true;

  // General info
  double cpu_load                   = 0;
  double cpu_freq                   = 0;
  double ram_free                   = 0;
  double total_ram                  = 0;
  double disk_free                  = 0;
  bool   comp_state_update_required = true;

  // Mavros
  double      mavros_rate            = 0;
  double      state_rate             = 0;
  double      cmd_rate               = 0;
  double      battery_rate           = 0;
  bool        mavros_gps_ok          = false;
  bool        armed                  = false;
  std::string mode                   = "";
  double      battery_volt           = 0;
  double      battery_curr           = 0;
  double      battery_wh_drained     = 0;
  double      thrust                 = 0;
  double      mass_estimate          = 0;
  double      mass_set               = 0;
  double      gps_qual               = 0;
  double      mag_norm               = 0;
  double      mag_norm_rate          = 0;
  bool        mavros_update_required = true;

  // Custom topics
  std::vector<mrs_msgs::CustomTopic> custom_topic_vec;
  bool                               topics_update_required = true;

  // Custom string outputs
  std::vector<std::string> custom_string_vec;
  bool                     string_update_required = true;

  // Node stats
  mrs_msgs::NodeCpuLoad node_cpu_load_vec;
  double                cpu_load_total             = 0;
  bool                  node_stats_update_required = true;

  // | ----------------------- Attributes ----------------------- |
  ros::NodeHandle nh;
  ros::Subscriber uav_status_sub;
  QColor          bg_color = QColor(0, 0, 0, 100);
  QColor          fg_color = QColor(25, 255, 240, 255);
  bool            is_inited = false;
  static int      display_number;
  int             id;
  std::string     last_uav_name;
  static std::unordered_map<std::string, bool> taken_uavs;

  // | --------------------- Default values --------------------- |
  const QColor RED_COLOR    = QColor(255, 0, 0, 255);
  const QColor YELLOW_COLOR = QColor(255, 255, 0, 255);
  const QColor GREEN_COLOR  = QColor(0, 255, 0, 255);
  const QColor NO_COLOR     = QColor(0, 0, 0, 0);

  // | ---------------------- Layout data ----------------------- |
  std::vector<bool> present_columns{true, true, true, true, false};

  int  display_pos_x          = 0;
  int  display_pos_y          = 0;
  int  cm_pos_y               = 23;
  int  odom_pos_y             = 86;
  int  gen_info_pos_x         = 233;
  int  gen_info_pos_y         = 23;
  int  mavros_pos_x           = 233;
  int  mavros_pos_y           = 86;
  int  topic_rate_pos_x       = 466;
  int  topic_rate_pos_y       = 23;
  int  custom_str_pos_x       = 699;
  int  custom_str_pos_y       = 0;
  int  custom_str_height      = 206;
  int  node_stats_pos_x       = 932;
  int  node_stats_pos_y       = 0;
  int  node_stats_height      = 206;
  bool global_update_required = true;
};

}  // namespace mrs_rviz_plugins


#endif
