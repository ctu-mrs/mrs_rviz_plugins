#ifndef MRS_STATUS_DISPLAY_H
#define MRS_STATUS_DISPLAY_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <QStaticText>
#include <QPainter>
#include <QColor>

#include <rviz/properties/bool_property.h>

#include "uav_status/overlay_utils.h"

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>


#include <rviz/message_filter_display.h>

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

  // Subscriber callbacks
  void uavStatusCb(const mrs_msgs::UavStatusConstPtr& msg);

  // New message processing methods
  void processControlManager(const mrs_msgs::UavStatusConstPtr& msg);

  // Drawing methods
  void drawControlManager();

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
  jsk_rviz_plugins::OverlayObject::Ptr computer_load_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr mavros_state_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr first_unknown_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr second_unknown_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr rosnode_shitlist_overlay;

  // Controller data:
  double       avg_rate;
  bool         null_tracker;
  double       rate;
  std::string  curr_controller  = "!NO DATA!";
  std::string  curr_tracker     = "!NO DATA!";
  std::string  curr_gains;
  std::string  curr_constraints;
  bool         callbacks_enabled;
  bool         has_goal;
  bool         cm_update_required = true;


  mrs_msgs::UavStatus last_uav_status;

  ros::NodeHandle nh;
};

} // namespace mrs_rviz_plugins


#endif