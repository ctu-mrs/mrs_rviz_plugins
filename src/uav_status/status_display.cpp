#include "uav_status/status_display.h"

namespace mrs_rviz_plugins
{
  StatusDisplay::StatusDisplay(){
    uav_name_property         = new rviz::StringProperty("Uav name",       "uav22", "Uav name to show status data",     this, SLOT(nameUpdate()), this);
    control_manager_property   = new rviz::BoolProperty("Control manager",   true,  "Show control manager data",        this,  SLOT(controlManagerUpdate()), this);
    odometry_property         = new rviz::BoolProperty("Odometry",          true,  "Show odometry data",               this,  SLOT(odometryUpdate()), this);
    computer_load_property    = new rviz::BoolProperty("Computer load",     true,  "Show computer load data",          this,  SLOT(computerLoadUpdate()), this);
    mavros_state_property     = new rviz::BoolProperty("Mavros state",      false, "Show mavros state data",           this,  SLOT(mavrosStateUpdate()), this);
    topic_rates_property      = new rviz::BoolProperty("Topic rates",       false, "Show somethin, idk anythin 1",     this,  SLOT(topicRatesUpdate()), this);
    custom_str_property       = new rviz::BoolProperty("Custom strings",    false, "Show somethin, idk anythin 2",     this,  SLOT(customStrUpdate()), this);
    node_stats_property       = new rviz::BoolProperty("Node stats list",   false, "Show rosnodes and their workload", this,  SLOT(nodeStatsUpdate()), this);
    
    
    debug_property            = new rviz::IntProperty("number", 10, "hehe", this, SLOT(tmpUpdate()), this);


    nh = ros::NodeHandle();
  }
  
  void StatusDisplay::onInitialize(){

    contol_manager_overlay  .reset(new jsk_rviz_plugins::OverlayObject("Control manager"));
    odometry_overlay        .reset(new jsk_rviz_plugins::OverlayObject("Odometry"));
    general_info_overlay    .reset(new jsk_rviz_plugins::OverlayObject("General info"));
    mavros_state_overlay    .reset(new jsk_rviz_plugins::OverlayObject("Mavros state"));
    topic_rates_overlay     .reset(new jsk_rviz_plugins::OverlayObject("Topic rates"));
    custom_strings_overlay  .reset(new jsk_rviz_plugins::OverlayObject("Custom strings"));
    rosnode_stats_overlay.reset(new jsk_rviz_plugins::OverlayObject("Rosnode shitlist"));

    uav_status_sub = nh.subscribe(uav_name_property->getStdString() + "/mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());
  }

  void StatusDisplay::update(float wall_dt, float ros_dt){

    if(cm_update_required         || global_update_required) drawControlManager();
    if(odom_update_required       || global_update_required) drawOdometry();
    if(comp_state_update_required || global_update_required) drawGeneralInfo();
    if(mavros_update_required     || global_update_required) drawMavros();
    if(topics_update_required     || global_update_required) drawCustomTopicRates();
    if(string_update_required     || global_update_required) drawCustomStrings();
    if(node_stats_update_required || global_update_required) drawNodeStats();

    global_update_required = false;
  }

  void StatusDisplay::drawControlManager() {
    ROS_INFO("updateing cm...");

    // TODO: if controller_rate == 0 show NO_CONTROLLER and NO_TRACKER


    // Control manager overlay
    contol_manager_overlay->updateTextureSize(230, 60);
    contol_manager_overlay->setPosition(display_pos_x, display_pos_y);
    contol_manager_overlay->show(control_manager_property->getBool());
    if(!control_manager_property->getBool()){
      return;
    }

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
      painter.fillRect(0, 26, 80, 13, RED_COLOR);
    }
    QStaticText controller_data_text = QStaticText(QString("%1/%2").arg(curr_controller.c_str(), curr_gains.c_str()));
    painter.drawStaticText(0, 20, controller_data_text);
    if(!callbacks_enabled){
      painter.fillRect(169, 26, 48, 13, RED_COLOR);
    }
    QStaticText no_callback_text = QStaticText(QString("%1").arg(callbacks_enabled ? "" : "NO_CB"));
    painter.drawStaticText(171, 20, no_callback_text);

    // Tracker
    if(curr_controller.find("!NO DATA!") != std::string::npos){
      painter.fillRect(0, 46, 80, 13, RED_COLOR);
    }
    if(null_tracker){
      painter.fillRect(0, 46, 100, 13, RED_COLOR);
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
    odometry_overlay->setPosition(display_pos_x,display_pos_y + odom_pos_y);
    odometry_overlay->show(odometry_property->getBool());
    if(!odometry_property->getBool()){
      return;
    }

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
      painter.fillRect(0, 5, 80, 13, RED_COLOR);
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
        x_warning_color = NO_COLOR;
      } else if (cerr_x < 1.0) {
        x_warning_color = YELLOW_COLOR;
      } else {
        x_warning_color = RED_COLOR;
      }
      QColor y_warning_color;
      if (cerr_y < 0.5) {
        y_warning_color = NO_COLOR;
      } else if (cerr_y < 1.0) {
        y_warning_color = YELLOW_COLOR;
      } else {
        y_warning_color = RED_COLOR;
      }
      QColor z_warning_color;
      if (cerr_z < 0.5) {
        z_warning_color = NO_COLOR;
      } else if (cerr_z < 1.0) {
        z_warning_color = YELLOW_COLOR;
      } else {
        z_warning_color = RED_COLOR;
      }
      QColor h_warning_color;
      if (cerr_hdg < 0.5) {
        h_warning_color = NO_COLOR;
      } else if (cerr_hdg < 1.0) {
        h_warning_color = YELLOW_COLOR;
      } else {
        h_warning_color = RED_COLOR;
      }

      painter.fillRect(45, 105, 27, 13, x_warning_color);
      painter.fillRect(90, 105, 27, 13, x_warning_color);
      painter.fillRect(135, 105, 27, 13, x_warning_color);
      painter.fillRect(180, 105, 27, 13, x_warning_color);

      QString error;
      error.sprintf("C/E X%.1f Y%.1f Z%.1f H%.1f", cerr_x, cerr_y, cerr_z, cerr_hdg);
      painter.drawStaticText(0, 100, QStaticText(error));
    }

    odom_update_required = false;
    odometry_overlay->setDimensions(odometry_overlay->getTextureWidth(), odometry_overlay->getTextureHeight());
  }

  void StatusDisplay::drawGeneralInfo(){
    // Note: colors are 100% implemented

    // General info overlay
    general_info_overlay->updateTextureSize(230, 60);
    general_info_overlay->setPosition(display_pos_x + gen_info_pos_x, display_pos_y);
    general_info_overlay->show(computer_load_property->getBool());
    if(!computer_load_property->getBool()){
      return;
    }

    jsk_rviz_plugins::ScopedPixelBuffer buffer = general_info_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*general_info_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));
 
    // CPU load
    QColor cpu_load_color = NO_COLOR;
    if(cpu_load > 80.0){
      cpu_load_color = RED_COLOR;
    } else if(cpu_load > 60.0){
      cpu_load_color = YELLOW_COLOR;
    }
    painter.fillRect(0, 25, 92, 13, cpu_load_color);
    QString cpu_load_str;
    cpu_load_str.sprintf("CPU: %.1f%%", cpu_load);
    painter.drawStaticText(0, 20, QStaticText(cpu_load_str));

    // CPU frequency
    QString cpu_freq_str;
    cpu_freq_str.sprintf("%.2f GHz", cpu_freq);
    painter.drawStaticText(110, 20, QStaticText(cpu_freq_str));

    // Free RAM
    double used_ram = total_ram - ram_free;
    double ram_ratio = used_ram / total_ram;
    QColor ram_color = NO_COLOR;
    if (ram_ratio > 0.7) {
      ram_color = RED_COLOR;
    } else if (ram_ratio > 0.5) {
      ram_color = YELLOW_COLOR;
    }
    painter.fillRect(0, 45, 92, 13, ram_color);
    QString ram_free_str;
    ram_free_str.sprintf("RAM: %.1f G", ram_free);
    painter.drawStaticText(0, 40, QStaticText(ram_free_str));

    // Free disk
    QColor free_disk_color = NO_COLOR;
    if(disk_free < 100){
      free_disk_color = RED_COLOR;
    } else if(disk_free < 200){
      free_disk_color = YELLOW_COLOR;
    }
    painter.fillRect(109, 45, 118, 13, free_disk_color);
    QString disk_free_str;
    if(disk_free < 10000){
      disk_free_str.sprintf("HDD: %.1f G", disk_free/10);
    } else{
      disk_free_str.sprintf("HDD: %.1f G", disk_free/10000);
    }
    painter.drawStaticText(110, 40, QStaticText(disk_free_str));

    comp_state_update_required = false;    
    general_info_overlay->setDimensions(general_info_overlay->getTextureWidth(), general_info_overlay->getTextureHeight());
  }

  void StatusDisplay::drawMavros(){
    // Note: colors are 100% implemented

    // Mavros overlay
    mavros_state_overlay->updateTextureSize(230, 120);
    mavros_state_overlay->setPosition(display_pos_x + mavros_pos_x, display_pos_y + mavros_pos_y);
    mavros_state_overlay->show(mavros_state_property->getBool());
    if(!mavros_state_property->getBool()){
      return;
    }

    jsk_rviz_plugins::ScopedPixelBuffer buffer = mavros_state_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*mavros_state_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));

    // Main row
    QStaticText mavros_text = QStaticText("Mavros");
    QString tmp;
    tmp.sprintf("%s%.1f Hz", mavros_rate >= 100 ? "" : " ", mavros_rate);
    QStaticText mavros_freq_text = QStaticText(tmp);
    painter.drawStaticText(94, 0, mavros_text);
    painter.drawStaticText(152, 0, mavros_freq_text);
    if(mavros_rate == 0){ // No data
      painter.fillRect(0, 5, 80, 13, RED_COLOR);
      painter.drawStaticText(0, 0, QStaticText("!NO DATA!"));
    }

    // State:
    painter.drawStaticText(0, 20, QStaticText("State:"));
    if(state_rate == 0){
      painter.fillRect(55, 26, 48, 13, RED_COLOR);
      painter.drawStaticText(56, 20, QStaticText("ERROR"));
    } else{
      if(armed){
        painter.drawStaticText(56, 20, QStaticText("ARMED"));
      } else{
        painter.fillRect(55, 26, 75, 13, RED_COLOR);
        painter.drawStaticText(56, 20, QStaticText("DISARMED"));
      }
    }

    // Mode:
    painter.drawStaticText(0, 40, QStaticText("Mode:"));
    if(mode != "OFFBOARD"){
      painter.fillRect(45, 46, 57, 13, RED_COLOR);
    }
    painter.drawStaticText(46, 40, QStaticText(mode.c_str()));

    // Batt:
    painter.drawStaticText(0, 60, QStaticText("Batt:"));
    if(battery_rate == 0){
      painter.fillRect(45, 66, 48, 13, RED_COLOR);
      painter.drawStaticText(46, 60, QStaticText("ERROR"));
    }else{
      double volt_to_show = (battery_volt > 17.0) ? (battery_volt / 6) : (battery_volt / 4);
      if (volt_to_show < 3.6) {
        painter.fillRect(45, 66, 57, 13, RED_COLOR);
      } else if (volt_to_show < 3.7) {
        painter.fillRect(45, 66, 57, 13, YELLOW_COLOR);
      }
      tmp.sprintf("%.2f V  %.2f A", volt_to_show, battery_curr);
      painter.drawStaticText(46, 60, QStaticText(tmp));
    }

    // Drained:
    painter.drawStaticText(0, 80, QStaticText("Drained:"));
    tmp.sprintf("%.1f Wh", battery_wh_drained);
    painter.drawStaticText(72, 80, QStaticText(tmp));

    // Thrst:
    painter.drawStaticText(0, 100, QStaticText("Thrst:"));
    if (thrust > 0.75) {
      painter.fillRect(56, 106, 37, 13, RED_COLOR);
    } else if (thrust > 0.65) {
      painter.fillRect(56, 106, 37, 13, RED_COLOR);
    }
    tmp.sprintf("%.2f", thrust);
    painter.drawStaticText(56, 100, QStaticText(tmp));

    // GPS
    if(!mavros_gps_ok){
      painter.fillRect(159, 26, 56, 13, RED_COLOR);
      painter.drawStaticText(160, 20, QStaticText("NO_GPS"));
    } else{
      painter.drawStaticText(160, 20, QStaticText("GPS_OK"));

      QColor gps_qual_color = RED_COLOR;
      if (gps_qual < 5.0) {
        gps_qual_color = NO_COLOR;
      } else if (gps_qual < 10.0) {
        gps_qual_color = YELLOW_COLOR;      
      }
      painter.fillRect(186, 46, 29, 13, gps_qual_color);
      tmp.sprintf("Q: %.1f", gps_qual);
      painter.drawStaticText(160, 40, QStaticText(tmp));
    }

    // Mass
    double mass_diff = fabs(mass_estimate - mass_set) / mass_set;
    QColor mass_color = NO_COLOR;
    if (mass_diff > 0.3) {
      mass_color = RED_COLOR;
    } else if (mass_diff > 0.2) {
      mass_color = YELLOW_COLOR;
    }
    painter.fillRect(160, 106, 36, 13, mass_color);
    tmp.sprintf("%s%.1f/%s%.1fkg",mass_set >= 10.0 ? "" : " ", mass_set, mass_estimate >= 10.0 ? "" : " ", mass_estimate);
    painter.drawStaticText(115, 100, QStaticText(tmp));

    mavros_update_required = false;
    mavros_state_overlay->setDimensions(mavros_state_overlay->getTextureWidth(), mavros_state_overlay->getTextureHeight());
  }

  void StatusDisplay::drawCustomTopicRates() {
    // Note: all colors are 100% implemented

    // Topic rate overlay
    topic_rates_overlay->updateTextureSize(230, 183);
    topic_rates_overlay->setPosition(display_pos_x + topic_rate_pos_x, display_pos_y);
    topic_rates_overlay->show(topic_rates_property->getBool());
    if(!topic_rates_property->getBool()){
      return;
    }

    jsk_rviz_plugins::ScopedPixelBuffer buffer = topic_rates_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*topic_rates_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));

    // Setting the stream up
    QString frequency;
    QTextStream ts = QTextStream(&frequency);
    ts.setRealNumberPrecision(1);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setFieldAlignment(QTextStream::AlignRight);
    ts.setFieldWidth(10);
    ts.setPadChar(' ');

    // Drawing topics
    for (size_t i = 0; i < custom_topic_vec.size(); i++){
      frequency = "";
      ts << custom_topic_vec[i].topic_hz;

      auto spaces = frequency.count(" ");
      auto width  = 10 - spaces;

      painter.drawStaticText(0, 20*i, QStaticText(custom_topic_vec[i].topic_name.c_str()));
      painter.fillRect(116 + 9*spaces, 20*i+6, 9*width+1, 13, getColor(custom_topic_vec[i].topic_color));
      painter.drawStaticText(117, 20*i, QStaticText(frequency));
      painter.drawStaticText(210, 20*i, QStaticText("Hz"));
    }

    topics_update_required = false;
    topic_rates_overlay->setDimensions(topic_rates_overlay->getTextureWidth(), topic_rates_overlay->getTextureHeight());
  }

  void StatusDisplay::drawCustomStrings() {
    // Note: colors implemeted, but blinking is not

    // Custom string overlay
    custom_strings_overlay->updateTextureSize(230, 183);
    custom_strings_overlay->setPosition(display_pos_x + custom_str_pos_x, display_pos_y);
    custom_strings_overlay->show(custom_str_property->getBool());
    if(!custom_str_property->getBool()){
      return;
    }

    jsk_rviz_plugins::ScopedPixelBuffer buffer = custom_strings_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*custom_strings_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));

    // Drawing strings
    for (size_t i = 0; i < custom_string_vec.size(); i++){
      int tmp_color = NORMAL;
      std::string tmp_display_string = custom_string_vec[i];

      // Set color of the string
      if (tmp_display_string.at(0) == '-') {
        if (tmp_display_string.at(1) == 'r' || tmp_display_string.at(1) == 'R') {
          tmp_color = RED;
        } else if (tmp_display_string.at(1) == 'y' || tmp_display_string.at(1) == 'Y') {
          tmp_color = YELLOW;
        } else if (tmp_display_string.at(1) == 'g' || tmp_display_string.at(1) == 'G') {
          tmp_color = GREEN;
        }
        // If color data are present, delete them 
        if (tmp_color != NORMAL) {
          tmp_display_string.erase(0, 3);
        }
      }

      painter.fillRect(0, 20*i+6, 9*tmp_display_string.length()+1, 13, getColor(tmp_color));
      painter.drawStaticText(0, 20*i, QStaticText(tmp_display_string.c_str()));
    }

    string_update_required = false;
    custom_strings_overlay->setDimensions(custom_strings_overlay->getTextureWidth(), custom_strings_overlay->getTextureHeight());
  }

  void StatusDisplay::drawNodeStats(){
    // Rosnode stats overlay
    rosnode_stats_overlay->updateTextureSize(394, 183);
    rosnode_stats_overlay->setPosition(display_pos_x + node_stats_pos_x, display_pos_y);
    rosnode_stats_overlay->show(node_stats_property->getBool());
    if(!node_stats_property->getBool()){
      return;
    }

    jsk_rviz_plugins::ScopedPixelBuffer buffer = rosnode_stats_overlay->getBuffer();
    QColor bg_color_ = QColor(0,  0,   0,   100);
    QColor fg_color_ = QColor(25, 255, 240, 255);

    // Setting the painter up
    QImage hud = buffer.getQImage(*rosnode_stats_overlay, bg_color_);
    QFont font = QFont("Courier");
    font.setBold(true);
    QPainter painter(&hud);
    painter.setFont(font);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));

    // Main row
    QString tmp;
    tmp.sprintf("%.1f", cpu_load_total);
    painter.drawStaticText(0, 0, QStaticText("ROS Node Shitlist"));
    painter.drawStaticText(285, 0, QStaticText(tmp));
    painter.drawStaticText(345, 0, QStaticText("CPU %"));

    // Drawing stats
    for (size_t i = 0; i < node_cpu_load_vec.node_names.size(); i++){
      painter.drawStaticText(0, (i+1)*20, QStaticText(node_cpu_load_vec.node_names[i].c_str()));

      QColor tmp_color = getColor(GREEN);
      if(node_cpu_load_vec.cpu_loads[i] > 99.9){
        tmp_color = RED_COLOR;
      }else if(node_cpu_load_vec.cpu_loads[i] > 49.9){
        tmp_color = YELLOW_COLOR;
      }

      painter.fillRect(345, 26 + i*20, 46, 13, tmp_color);
      tmp.sprintf("%5.1f", node_cpu_load_vec.cpu_loads[i]);
      painter.drawStaticText(345, 20*(i+1), QStaticText(tmp));
    }

    node_stats_update_required = false;
    rosnode_stats_overlay->setDimensions(rosnode_stats_overlay->getTextureWidth(), rosnode_stats_overlay->getTextureHeight());
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
    processGeneralInfo(msg);
    processMavros(msg);
    processCustomTopics(msg);
    processCustomStrings(msg);
    processNodeStats(msg);
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
    odom_update_required |= compareAndUpdate(new_cmd_x, cmd_x);
    odom_update_required |= compareAndUpdate(new_cmd_y, cmd_y);
    odom_update_required |= compareAndUpdate(new_cmd_z, cmd_z);
    odom_update_required |= compareAndUpdate(new_cmd_hdg, cmd_hdg);
    odom_update_required |= compareAndUpdate(new_odom_frame, odom_frame);
    odom_update_required |= compareAndUpdate(new_curr_estimator_hori, curr_estimator_hori);
    odom_update_required |= compareAndUpdate(new_curr_estimator_vert, curr_estimator_vert);
    odom_update_required |= compareAndUpdate(new_curr_estimator_hdg, curr_estimator_hdg);
  }

  void StatusDisplay::processGeneralInfo(const mrs_msgs::UavStatusConstPtr& msg){
    double new_cpu_load = msg->cpu_load;
    double new_cpu_freq = msg->cpu_ghz;
    double new_ram_free = msg->free_ram;
    double new_total_ram = msg->total_ram;
    double new_disk_free = msg->free_hdd;

    comp_state_update_required |= compareAndUpdate(new_cpu_load, cpu_load);
    comp_state_update_required |= compareAndUpdate(new_cpu_freq, cpu_freq);
    comp_state_update_required |= compareAndUpdate(new_ram_free, ram_free);
    comp_state_update_required |= compareAndUpdate(new_total_ram, total_ram);
    comp_state_update_required |= compareAndUpdate(new_disk_free, disk_free);
  }

  void StatusDisplay::processMavros(const mrs_msgs::UavStatusConstPtr& msg){
    double      new_mavros_rate         = msg->mavros_hz;
    double      new_state_rate          = msg->mavros_state_hz;
    double      new_cmd_rate            = msg->mavros_cmd_hz;
    double      new_battery_rate        = msg->mavros_battery_hz;
    bool        new_mavros_gps_ok       = msg->mavros_gps_ok;
    bool        new_armed               = msg->mavros_armed;
    std::string new_mode                = msg->mavros_mode;
    double      new_battery_volt        = msg->battery_volt;
    double      new_battery_curr        = msg->battery_curr;
    double      new_battery_wh_drained  = msg->battery_wh_drained;
    double      new_thrust              = msg->thrust;
    double      new_mass_estimate       = msg->mass_estimate;
    double      new_mass_set            = msg->mass_set;
    double      new_gps_qual            = msg->mavros_gps_qual;
    // double      new_mag_norm;
    // double      new_mag_norm_rate;

    // mag_norm           = msg->mag_norm;
    // mag_norm_rate      = msg->mag_norm_hz;

    mavros_update_required |= compareAndUpdate(new_mavros_rate, mavros_rate);
    mavros_update_required |= compareAndUpdate(new_state_rate, state_rate);
    mavros_update_required |= compareAndUpdate(new_cmd_rate, cmd_rate);
    mavros_update_required |= compareAndUpdate(new_battery_rate, battery_rate);
    mavros_update_required |= compareAndUpdate(new_mavros_gps_ok, mavros_gps_ok);
    mavros_update_required |= compareAndUpdate(new_armed, armed);
    mavros_update_required |= compareAndUpdate(new_mode, mode);
    mavros_update_required |= compareAndUpdate(new_battery_volt, battery_volt);
    mavros_update_required |= compareAndUpdate(new_battery_curr, battery_curr);
    mavros_update_required |= compareAndUpdate(new_battery_wh_drained, battery_wh_drained);
    mavros_update_required |= compareAndUpdate(new_thrust, thrust);
    mavros_update_required |= compareAndUpdate(new_mass_estimate, mass_estimate);
    mavros_update_required |= compareAndUpdate(new_mass_set, mass_set);
    mavros_update_required |= compareAndUpdate(new_gps_qual, gps_qual);
    // mavros_update_required |= compareAndUpdate(new_mag_norm, mag_norm);
    // mavros_update_required |= compareAndUpdate(new_mag_norm_rate, mag_norm_rate);
  }

  void StatusDisplay::processCustomTopics(const mrs_msgs::UavStatusConstPtr& msg) {
    std::vector<mrs_msgs::CustomTopic> new_custom_topic_vec = msg->custom_topics;

    topics_update_required |= compareAndUpdate(new_custom_topic_vec, custom_topic_vec);
  }

  void StatusDisplay::processCustomStrings(const mrs_msgs::UavStatusConstPtr& msg) {
    std::vector<std::string> new_custom_string_vec = msg->custom_string_outputs;

    topics_update_required |= compareAndUpdate(new_custom_string_vec, custom_string_vec);
  }

  void StatusDisplay::processNodeStats(const mrs_msgs::UavStatusConstPtr& msg) {
    mrs_msgs::NodeCpuLoad new_node_cpu_load_vec = msg->node_cpu_loads ;
    double                new_cpu_load_total = msg->cpu_load_total;


    node_stats_update_required |= compareAndUpdate(new_node_cpu_load_vec, node_cpu_load_vec);
    node_stats_update_required |= compareAndUpdate(new_cpu_load_total, cpu_load_total);
  }

  void StatusDisplay::nameUpdate(){
    // Controller
    uav_status_sub = nh.subscribe(uav_name_property->getStdString() + "/mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());
    curr_controller  = "!NO DATA!";
    curr_tracker     = "!NO DATA!";
    curr_gains       = "";
    curr_constraints = "";
    avg_controller_rate = 0.0;
    cm_update_required = true;

    // Odometry
    odom_frame = "!NO DATA!";
    curr_estimator_hori = "!NO DATA!";
    curr_estimator_vert = "!NO DATA!";
    curr_estimator_hdg = "!NO DATA!";
    avg_odom_rate = 0.0;
    odom_update_required = true;

    // General info
    comp_state_update_required = true;
  }

  void StatusDisplay::controlManagerUpdate() {
    cm_update_required = true;
    present_columns[CM_INDEX] = control_manager_property->getBool() || odometry_property->getBool();
    odometryUpdate();
    computerLoadUpdate();
    return;
  }

  void StatusDisplay::odometryUpdate() {
    odom_update_required = true;
    if(!odometry_property->getBool()){
      if(!control_manager_property->getBool()){
        present_columns[ODOM_INDEX] = false;
      }
      computerLoadUpdate();
      return;
    }

    present_columns[ODOM_INDEX] = true;

    if(control_manager_property->getBool()){
      odom_pos_y = 63;
    } else {
      odom_pos_y = 0;
    }
    computerLoadUpdate();
  }

  void StatusDisplay::computerLoadUpdate() {
    comp_state_update_required = true;
    if(!computer_load_property->getBool()){
      if(!mavros_state_property->getBool()){
        present_columns[GEN_INFO_INDEX] = false;
      }
      mavrosStateUpdate();
      topicRatesUpdate();
      return;
    }

    present_columns[GEN_INFO_INDEX] = true;

    gen_info_pos_x = 0;
    for(int i=0; i<GEN_INFO_INDEX; ++i){
      gen_info_pos_x += present_columns[i] ? 233 : 0;
    }
    mavrosStateUpdate();
  }

  void StatusDisplay::mavrosStateUpdate() {
    mavros_update_required = true;
    if(!mavros_state_property->getBool()){
      if(!computer_load_property->getBool()){
        present_columns[MAVROS_INDEX] = false;
      }
      topicRatesUpdate();
      return;
    }

    present_columns[MAVROS_INDEX] = true;

    mavros_pos_x = 0;
    for(int i=0; i<MAVROS_INDEX; ++i){
      mavros_pos_x += present_columns[i] ? 233 : 0;
    }

    if(computer_load_property->getBool()){
      mavros_pos_y = 63;
    } else {
      mavros_pos_y = 0;
    }
    topicRatesUpdate();
  }

  void StatusDisplay::topicRatesUpdate() {
    topics_update_required = true;
    if(!topic_rates_property->getBool()){
      present_columns[TOPIC_RATE_INDEX] = false;
      customStrUpdate();
      return;
    }

    present_columns[TOPIC_RATE_INDEX] = true;

    topic_rate_pos_x = 0;
    for(int i=0; i<TOPIC_RATE_INDEX; ++i){
      topic_rate_pos_x += present_columns[i] ? 233 : 0;
    }
    customStrUpdate();
  }

  void StatusDisplay::customStrUpdate() {
    string_update_required = true;
    if(!custom_str_property->getBool()){
      present_columns[CUSTOM_STR_INDEX] = false;
      nodeStatsUpdate();
      return;
    }

    present_columns[CUSTOM_STR_INDEX] = true;

    custom_str_pos_x = 0;
    for(int i=0; i<CUSTOM_STR_INDEX; ++i){
      custom_str_pos_x += present_columns[i] ? 233 : 0;
    }
    nodeStatsUpdate();
  }

  void StatusDisplay::nodeStatsUpdate() {
    node_stats_update_required = true;
    if(!node_stats_property->getBool()){
      present_columns[NODE_STATS_INDEX] = false;
      return;
    }

    present_columns[NODE_STATS_INDEX] = true;

    node_stats_pos_x = 0;
    for(int i=0; i<NODE_STATS_INDEX; ++i){
      node_stats_pos_x += present_columns[i] ? 233 : 0;
    }
  }

  void StatusDisplay::tmpUpdate(){
    cm_update_required = true;
  }

  void StatusDisplay::movePosition(int x, int y){
    setPosition(x, y);
  }

  bool StatusDisplay::isInRegion(int x, int y){
    ROS_INFO("isInRegion called. res: %d", (display_pos_y < y && display_pos_y + 183 > y && 
            display_pos_x < x && display_pos_x + 932 + 394 > x));

    return (display_pos_y < y && display_pos_y + 183 > y && 
            display_pos_x < x && display_pos_x + 932 + 394 > x);
  }

}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StatusDisplay, rviz::Display)