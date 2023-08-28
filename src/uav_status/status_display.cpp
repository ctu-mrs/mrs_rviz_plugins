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
    odometry_overlay.reset(new jsk_rviz_plugins::OverlayObject("Odometry overlay"));
    
    uav_status_sub = nh.subscribe(uav_name_property->getStdString() + "mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Inited");
  }

  void StatusDisplay::update(float wall_dt, float ros_dt){
    if(!cm_update_required){
      return;
    }

    drawControlManager();
    drawOdometry();
  }

  void StatusDisplay::drawControlManager() {
    // TODO: if controller_rate == 0 show NO_CONTROLLER and NO_TRACKER


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
    tmp.sprintf("%s%.1f Hz", controller_rate >= 10 ? "" : " ", controller_rate);
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
    if(null_tracker){
      painter.fillRect(0, 46, 100, 13, QColor(255, 0, 0, 255));
    }
    QStaticText tracker_data_text = QStaticText(QString("%1/%2").arg(curr_tracker.c_str(), curr_constraints.c_str()));
    QStaticText has_goal_text = QStaticText(QString("%1").arg(has_goal ? " FLY" : "IDLE"));
    painter.drawStaticText(0, 40, tracker_data_text);
    painter.drawStaticText(180, 40, has_goal_text);

    contol_manager_overlay->setDimensions(contol_manager_overlay->getTextureWidth(), contol_manager_overlay->getTextureHeight());
    cm_update_required = false;
  }

  void StatusDisplay::drawOdometry(){
    // Odometry overlay
    odometry_overlay->updateTextureSize(230, 120);
    odometry_overlay->setPosition(0, 63);
    odometry_overlay->show();

    jsk_rviz_plugins::ScopedPixelBuffer buffer = odometry_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*odometry_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));

    // Main row
    QStaticText odometry_text = QStaticText("Odom");
    painter.drawStaticText(108, 0, odometry_text);
    QString tmp;
    tmp.sprintf("%s%.1f Hz", avg_odom_rate >= 100 ? "" : " ", avg_odom_rate);
    QStaticText control_manager_freq_text = QStaticText(tmp);
    painter.drawStaticText(152, 0, control_manager_freq_text);

    if(avg_odom_rate == 0.0){
      painter.fillRect(0, 5, 80, 13, QColor(255, 0, 0, 255));
      painter.drawStaticText(0, 0, QStaticText("!NO DATA!"));
      odometry_overlay->setDimensions(odometry_overlay->getTextureWidth(), odometry_overlay->getTextureHeight());
      return;
    }

    // XYZ and hdg column
    painter.drawStaticText(0, 20, QStaticText("X"));
    painter.drawStaticText(0, 40, QStaticText("Y"));
    painter.drawStaticText(0, 60, QStaticText("Z"));
    painter.drawStaticText(0, 80, QStaticText("hdg"));

    // XYZ and hdg values
    QString value;
    QTextStream ts = QTextStream(&value);
    ts.setRealNumberPrecision(2);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setFieldAlignment(QTextStream::AlignRight);
    ts.setFieldWidth(8);
    ts.setPadChar(' ');

    value = "";
    ts << state_x;
    painter.drawStaticText(15, 20, QStaticText(value));
    value = "";
    ts << state_y;
    painter.drawStaticText(15, 40, QStaticText(value));
    value = "";
    ts << state_z;
    painter.drawStaticText(15, 60, QStaticText(value));
    value = "";
    ts << heading;
    painter.drawStaticText(15, 80, QStaticText(value));

    // Estimators info
    QString hori = QString("Hori: %1").arg(curr_estimator_hori.c_str());
    QString vert = QString("Vert: %1").arg(curr_estimator_vert.c_str());
    QString head = QString("Head: %1").arg(curr_estimator_hdg.c_str());
    painter.drawStaticText(100, 20, QStaticText(odom_frame.c_str()));
    painter.drawStaticText(100, 40, QStaticText(hori));
    painter.drawStaticText(100, 60, QStaticText(vert));
    painter.drawStaticText(100, 80, QStaticText(head));

    if(!null_tracker){
      double cerr_x   = std::fabs(state_x - cmd_x);
      double cerr_y   = std::fabs(state_y - cmd_y);
      double cerr_z   = std::fabs(state_z - cmd_z);
      double cerr_hdg = std::fabs(heading - cmd_hdg);

      // TODO: make color 0, 0, 0, 0 if everything is alright
      QColor x_warning_color;
      if (cerr_x < 0.5) {
        x_warning_color = QColor(0, 0, 0, 0);
      } else if (cerr_x < 1.0) {
        x_warning_color = QColor(255, 255, 0, 255);
      } else {
        x_warning_color = QColor(255, 0, 0, 255);
      }
      QColor y_warning_color;
      if (cerr_y < 0.5) {
        y_warning_color = QColor(0, 0, 0, 0);
      } else if (cerr_y < 1.0) {
        y_warning_color = QColor(255, 255, 0, 255);
      } else {
        y_warning_color = QColor(255, 0, 0, 255);
      }
      QColor z_warning_color;
      if (cerr_z < 0.5) {
        z_warning_color = QColor(0, 0, 0, 0);
      } else if (cerr_z < 1.0) {
        z_warning_color = QColor(255, 255, 0, 255);
      } else {
        z_warning_color = QColor(255, 0, 0, 255);
      }
      QColor h_warning_color;
      if (cerr_hdg < 0.5) {
        h_warning_color = QColor(0, 0, 0, 0);
      } else if (cerr_hdg < 1.0) {
        h_warning_color = QColor(255, 255, 0, 255);
      } else {
        h_warning_color = QColor(255, 0, 0, 255);
      }

      painter.fillRect(45, 105, 27, 13, x_warning_color);
      painter.fillRect(90, 105, 27, 13, x_warning_color);
      painter.fillRect(135, 105, 27, 13, x_warning_color);
      painter.fillRect(180, 105, 27, 13, x_warning_color);

      QString error;
      error.sprintf("C/E X%.1f Y%.1f Z%.1f H%.1f", cerr_x, cerr_y, cerr_z, cerr_hdg);
      painter.drawStaticText(0, 100, QStaticText(error));
    }


    odometry_overlay->setDimensions(odometry_overlay->getTextureWidth(), odometry_overlay->getTextureHeight());
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
    processOdometry(msg);
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
    cm_update_required |= compareAndUpdate(new_rate, controller_rate);
    cm_update_required |= compareAndUpdate(new_controller, curr_controller);
    cm_update_required |= compareAndUpdate(new_tracker, curr_tracker);
    cm_update_required |= compareAndUpdate(new_gains, curr_gains);
    cm_update_required |= compareAndUpdate(new_constraints, curr_constraints);
    cm_update_required |= compareAndUpdate(new_callbacks_enabled, callbacks_enabled);
    cm_update_required |= compareAndUpdate(new_has_goal, has_goal);
  }

  void StatusDisplay::processOdometry(const mrs_msgs::UavStatusConstPtr& msg) {
    double new_avg_odom_rate;
    double new_color;
    double new_heading;
    double new_state_x;
    double new_state_y;
    double new_state_z;
    double new_cmd_x;
    double new_cmd_y;
    double new_cmd_z;
    double new_cmd_hdg;
    std::string new_odom_frame;
    std::string new_curr_estimator_hori;
    std::string new_curr_estimator_vert;
    std::string new_curr_estimator_hdg;

    ROS_INFO("processOdometry called");

    new_avg_odom_rate   = msg->odom_hz;
    new_color           = msg->odom_color;
    new_heading         = msg->odom_hdg;
    new_state_x         = msg->odom_x;
    new_state_y         = msg->odom_y;
    new_state_z         = msg->odom_z;
    new_odom_frame      = msg->odom_frame;

    new_cmd_x   = msg->cmd_x;
    new_cmd_y   = msg->cmd_y;
    new_cmd_z   = msg->cmd_z;
    new_cmd_hdg = msg->cmd_hdg;

    new_odom_frame = new_odom_frame.substr(new_odom_frame.find("/") + 1);

    msg->odom_estimators_hori.empty() ? new_curr_estimator_hori = "NONE" : new_curr_estimator_hori = msg->odom_estimators_hori[0];
    msg->odom_estimators_vert.empty() ? new_curr_estimator_vert = "NONE" : new_curr_estimator_vert = msg->odom_estimators_vert[0];
    msg->odom_estimators_hdg.empty() ? new_curr_estimator_hdg = "NONE" : new_curr_estimator_hdg = msg->odom_estimators_hdg[0];


    odom_update_required |= compareAndUpdate(new_avg_odom_rate, avg_odom_rate);
    odom_update_required |= compareAndUpdate(new_color, color);
    odom_update_required |= compareAndUpdate(new_heading, heading);
    odom_update_required |= compareAndUpdate(new_state_x, state_x);
    odom_update_required |= compareAndUpdate(new_state_y, state_y);
    odom_update_required |= compareAndUpdate(new_state_z, state_z);
    odom_update_required |= compareAndUpdate(new_cmd_x , cmd_x);
    odom_update_required |= compareAndUpdate(new_cmd_y , cmd_y);
    odom_update_required |= compareAndUpdate(new_cmd_z , cmd_z);
    odom_update_required |= compareAndUpdate(new_cmd_hdg , cmd_hdg);
    odom_update_required |= compareAndUpdate(new_odom_frame, odom_frame);
    odom_update_required |= compareAndUpdate(new_curr_estimator_hori, curr_estimator_hori);
    odom_update_required |= compareAndUpdate(new_curr_estimator_vert, curr_estimator_vert);
    odom_update_required |= compareAndUpdate(new_curr_estimator_hdg, curr_estimator_hdg);
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