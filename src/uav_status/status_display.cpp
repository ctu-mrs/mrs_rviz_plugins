#include "uav_status/status_display.h"

namespace mrs_rviz_plugins
{
  StatusDisplay::StatusDisplay(){
    uav_name_property         = new rviz::StringProperty("Uav name",       "uav1", "Uav name to show status data",     this, SLOT(nameUpdate()), this);
    contol_manager_property   = new rviz::BoolProperty("Control manager",   true,  "Show control manager data",        this);
    odometry_property         = new rviz::BoolProperty("Odometry",          true,  "Show odometry data",               this);
    computer_load_property    = new rviz::BoolProperty("Computer load",     true,  "Show computer load data",          this);
    mavros_state_property     = new rviz::BoolProperty("Mavros state",      false, "Show mavros state data",           this);
    first_unknown_property    = new rviz::BoolProperty("Unknown 1",         false, "Show somethin, idk anythin 1",     this);
    second_unknown_property   = new rviz::BoolProperty("Unknown 2",         false, "Show somethin, idk anythin 2",     this);
    rosnode_shitlist_property = new rviz::BoolProperty("ROS Node Shitlist", false, "Show rosnodes and their workload", this);
    debug_property            = new rviz::IntProperty("number", 10, "hehe", this, SLOT(tmpUpdate()), this);


    nh = ros::NodeHandle();
  }
  
  void StatusDisplay::onInitialize(){

    contol_manager_overlay.reset(new jsk_rviz_plugins::OverlayObject("Control manager overlay"));
    
    uav_status_sub = nh.subscribe(uav_name_property->getStdString() + "mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Inited");
  }

  void StatusDisplay::update(float wall_dt, float ros_dt){
    if(!cm_update_required){
      return;
    }

    drawControlManager();
  }

  void StatusDisplay::drawControlManager() {
    // Control manager overlay
    contol_manager_overlay->updateTextureSize(230, 60);
    contol_manager_overlay->setPosition(0, 0);
    contol_manager_overlay->show();

    jsk_rviz_plugins::ScopedPixelBuffer buffer = contol_manager_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*contol_manager_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));

    // Main row
    QStaticText control_manager_text = QStaticText("Control manager");
    painter.drawStaticText(0, 0, control_manager_text);
    QString tmp;
    tmp.sprintf("%s%.1f Hz", rate >= 10 ? "" : " ", rate);
    QStaticText control_manager_freq_text = QStaticText(tmp);
    painter.drawStaticText(152, 0, control_manager_freq_text);

    // Controller
    if(curr_controller.find("!NO DATA!") != std::string::npos){
      painter.fillRect(0, 26, 80, 13, QColor(255, 0, 0, 255));
    }
    QStaticText controller_data_text = QStaticText(QString("%1/%2").arg(curr_controller.c_str(), curr_gains.c_str()));
    painter.drawStaticText(0, 20, controller_data_text);
    if(!callbacks_enabled){
      painter.fillRect(169, 26, 48, 13, QColor(255, 0, 0, 255));
    }
    QStaticText no_callback_text = QStaticText(QString("%1").arg(callbacks_enabled ? "" : "NO_CB"));
    painter.drawStaticText(171, 20, no_callback_text);

    // Tracker
    if(curr_controller.find("!NO DATA!") != std::string::npos){
      painter.fillRect(0, 46, 80, 13, QColor(255, 0, 0, 255));
    }
    QStaticText tracker_data_text = QStaticText(QString("%1/%2").arg(curr_tracker.c_str(), curr_constraints.c_str()));
    painter.drawStaticText(0, 40, tracker_data_text);
    QStaticText has_goal_text = QStaticText(QString("%1").arg(has_goal ? " FLY" : "IDLE"));
    painter.drawStaticText(180, 40, has_goal_text);

    contol_manager_overlay->setDimensions(contol_manager_overlay->getTextureWidth(), contol_manager_overlay->getTextureHeight());
    cm_update_required = false;
  }


  void StatusDisplay::reset(){
  }


  // Helper function
  template<typename T> bool compareAndUpdate(T& new_value, T& current_value) {
    if(new_value != current_value){
      current_value = new_value;
      return true;
    }
    return false;
  }

  void StatusDisplay::uavStatusCb(const mrs_msgs::UavStatusConstPtr& msg) {
    processControlManager(msg);
  }

  void StatusDisplay::processControlManager(const mrs_msgs::UavStatusConstPtr& msg) {
    bool         new_null_tracker;
    double       new_rate;
    std::string  new_controller;
    std::string  new_tracker;
    std::string  new_gains;
    std::string  new_constraints;
    bool         new_callbacks_enabled;
    bool         new_has_goal;

    new_rate  = msg->control_manager_diag_hz;

    msg->controllers.empty() ? new_controller = "NONE"  : new_controller = msg->controllers[0];
    msg->trackers.empty()    ? new_tracker = "NONE"     : new_tracker = msg->trackers[0];
    msg->gains.empty()       ? new_gains = "NONE"       : new_gains = msg->gains[0];
    msg->constraints.empty() ? new_constraints = "NONE" : new_constraints = msg->constraints[0];

    new_callbacks_enabled = msg->callbacks_enabled;
    new_has_goal          = msg->have_goal;
    new_null_tracker      = msg->null_tracker;

    if(new_null_tracker){
      new_tracker = "NullTracker";
    }

    cm_update_required |= compareAndUpdate(new_null_tracker, null_tracker);
    cm_update_required |= compareAndUpdate(new_rate, rate);
    cm_update_required |= compareAndUpdate(new_controller, curr_controller);
    cm_update_required |= compareAndUpdate(new_tracker, curr_tracker);
    cm_update_required |= compareAndUpdate(new_gains, curr_gains);
    cm_update_required |= compareAndUpdate(new_constraints, curr_constraints);
    cm_update_required |= compareAndUpdate(new_callbacks_enabled, callbacks_enabled);
    cm_update_required |= compareAndUpdate(new_has_goal, has_goal);
  }

  void StatusDisplay::nameUpdate(){
    uav_status_sub = nh.subscribe(uav_name_property->getStdString() + "/mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());
    curr_controller  = "!NO DATA!";
    curr_tracker     = "!NO DATA!";
    curr_gains       = "";
    curr_constraints = "";
    cm_update_required = true;
  }

  void StatusDisplay::tmpUpdate(){
    cm_update_required = true;
  }



}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StatusDisplay, rviz::Display)