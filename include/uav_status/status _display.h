#ifndef MRS_STATUS_DISPLAY_H
#define MRS_STATUS_DISPLAY_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include "uav_status/overlay_utils.h"

#include <mrs_msgs/UavStatus.h>
#include <rviz/message_filter_display.h>

namespace mrs_rviz_plugins
{

class StatusDisplay : rviz::MessageFilterDisplay<mrs_msgs::UavStatus> {
Q_OBJECT;

public:
  StatusDisplay();
  void onInitialize() override;

  // A helper to clear this display back to the initial state.
  void reset() override;

// protected Q_SLOTS: 
//   // General
//   void onKeepChanged();

//   // Position
//   void onPositionChanged();
//   void onShapeChanged();
//   void onPoseColorChanged();
//   void onPoseParamsChanged();
//   void onAxesParamsChanged();

//   // Velocity
//   void onVelocityChanged();
//   void onVelColorChanged();
//   void onVelParamsChanged();

// protected:
//   void onDisable() override;
//   void onEnable() override;

private:
  void processMessage(const mrs_msgs::UavStatus::ConstPtr& msg) override;
  
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
}

} // namespace mrs_rviz_plugins


#endif