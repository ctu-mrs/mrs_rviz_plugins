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

  rviz::BoolProperty* rosnode_shitlist_property;

private:
  
  // Properties
  rviz::BoolProperty* contol_manager_property;
  rviz::BoolProperty* odometry_property;
  rviz::BoolProperty* computer_load_property;
  rviz::BoolProperty* mavros_state_property;
  rviz::BoolProperty* first_unknown_property;
  rviz::BoolProperty* second_unknown_property;

  // Individual overlays
  jsk_rviz_plugins::OverlayObject::Ptr contol_manager_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr odometry_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr computer_load_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr mavros_state_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr first_unknown_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr second_unknown_overlay;
  jsk_rviz_plugins::OverlayObject::Ptr rosnode_shitlist_overlay;

  std::string active_controller; // These 2 can be found control_manager/diagnostics 
  std::string active_tracker;

  // Note: gain can be found gain_manager/diagnostics
  // Note: constraint can be found constraint_manager/diagnostics
};

} // namespace mrs_rviz_plugins


#endif