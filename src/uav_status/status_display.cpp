#include "uav_status/status_display.h"

namespace mrs_rviz_plugins
{
  StatusDisplay::StatusDisplay(){
    contol_manager_property   = new rviz::BoolProperty("Control manager",   true,  "Show control manager data",        this);
    odometry_property         = new rviz::BoolProperty("Odometry",          true,  "Show odometry data",               this);
    computer_load_property    = new rviz::BoolProperty("Computer load",     true,  "Show computer load data",          this);
    mavros_state_property     = new rviz::BoolProperty("Mavros state",      false, "Show mavros state data",           this);
    first_unknown_property    = new rviz::BoolProperty("Unknown 1",         false, "Show somethin, idk anythin 1",     this);
    second_unknown_property   = new rviz::BoolProperty("Unknown 2",         false, "Show somethin, idk anythin 2",     this);
    rosnode_shitlist_property = new rviz::BoolProperty("ROS Node Shitlist", false, "Show rosnodes and their workload", this);


  }
  
  void StatusDisplay::onInitialize(){

    // Control manager overlay
    contol_manager_overlay.reset(new jsk_rviz_plugins::OverlayObject("Some OverlayObject"));
    contol_manager_overlay->updateTextureSize(200, 150);
    contol_manager_overlay->setPosition(0, 0);
    contol_manager_overlay->show();

    jsk_rviz_plugins::ScopedPixelBuffer buffer = contol_manager_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    // QColor fg_color_ = QColor(25, 255, 240, 255);

    QImage hud = buffer.getQImage(*contol_manager_overlay, bg_color_);
    QPainter painter(&hud);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(QColor(25, 255, 240, 255), 2, Qt::SolidLine));

    QStaticText static_text = QStaticText("Hello World!");
    painter.drawStaticText(0, 0, static_text);

    contol_manager_overlay->setDimensions(contol_manager_overlay->getTextureWidth(), contol_manager_overlay->getTextureHeight());
    ROS_INFO("Inited");
  }

  void StatusDisplay::reset(){
  }

}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StatusDisplay, rviz::Display)