#include "uav_status/status _display.h"

namespace mrs_rviz_plugins
{
  StatusDisplay::StatusDisplay(){

    // Compilation must stop here:
    something wrong

    // And here:
    ROS_INFO("alskdf", 4)
  }
  
  void StatusDisplay::onInitialize(){
    overlay.reset(new jsk_rviz_plugins::OverlayObject("Some OverlayObject"));
    overlay->show();

    jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, bg_color_);
    QPainter painter( &Hud );

    QStaticText static_text ("Hello World!");
    painter.drawStaticText(0, 0, static_text);
  }

  void StatusDisplay::reset(){
  }

  void StatusDisplay::processMessage(const mrs_msgs::UavStatus::ConstPtr& msg) {

  }
}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StatusDisplay, rviz::Display)