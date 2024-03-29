#include "uav_status/status_display.h"
#include <string>

namespace mrs_rviz_plugins
{
int                                   StatusDisplay::display_number = 0;
std::unordered_map<std::string, bool> StatusDisplay::taken_uavs;

StatusDisplay::StatusDisplay() {
  id            = display_number++;
  last_uav_name = "uav1";

  uav_name_property        = new rviz::EditableEnumProperty("UAV name", "uav1", "Uav name to show status data", this, SLOT(nameUpdate()), this);
  top_line_property        = new rviz::BoolProperty("Top Line", true, "Show general data", this, SLOT(topLineUpdate()), this);
  control_manager_property = new rviz::BoolProperty("Control manager", true, "Show control manager data", this, SLOT(controlManagerUpdate()), this);
  odometry_property        = new rviz::BoolProperty("Odometry", true, "Show odometry data", this, SLOT(odometryUpdate()), this);
  computer_load_property   = new rviz::BoolProperty("Computer load", true, "Show computer load data", this, SLOT(computerLoadUpdate()), this);
  hw_api_state_property    = new rviz::BoolProperty("Hw api state", true, "Show hw api state data", this, SLOT(hwApiStateUpdate()), this);
  topic_rates_property     = new rviz::BoolProperty("Topic rates", true, "Show somethin, idk anythin 1", this, SLOT(topicRatesUpdate()), this);
  custom_str_property      = new rviz::BoolProperty("Custom strings", true, "Show somethin, idk anythin 2", this, SLOT(customStrUpdate()), this);
  node_stats_property      = new rviz::BoolProperty("Node stats list", false, "Show rosnodes and their workload", this, SLOT(nodeStatsUpdate()), this);
  text_color_property      = new rviz::ColorProperty("Text color", fg_color, "Color of displayed text", this, SLOT(colorFgUpdate()), this);
  bg_color_property        = new rviz::ColorProperty("Background color", bg_color, "Color of background of the text", this, SLOT(colorBgUpdate()), this);

  nh = ros::NodeHandle();
}

StatusDisplay::~StatusDisplay() {
  taken_uavs[uav_name_property->getStdString()] = false;
}

void StatusDisplay::onInitialize() {
  top_line_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Top line") + std::to_string(id)));
  contol_manager_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Control manager") + std::to_string(id)));
  odometry_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Odometry") + std::to_string(id)));
  general_info_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("General info") + std::to_string(id)));
  hw_api_state_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Hw api state") + std::to_string(id)));
  topic_rates_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Topic rates") + std::to_string(id)));
  custom_strings_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Custom strings") + std::to_string(id)));
  rosnode_stats_overlay.reset(new jsk_rviz_plugins::OverlayObject(std::string("Rosnode cpu usage") + std::to_string(id)));

  uav_status_sub =
      nh.subscribe(uav_name_property->getStdString() + "/mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());

  // Preparing for searching the drone's name
  XmlRpc::XmlRpcValue      req = "/node";
  XmlRpc::XmlRpcValue      res;
  XmlRpc::XmlRpcValue      pay;
  std::vector<std::string> drone_names;
  ros::master::execute("getSystemState", req, res, pay, true);

  // Search for the drone's name
  std::string state[res[2][2].size()];
  for (int x = 0; x < res[2][2].size(); x++) {
    std::string name = res[2][2][x][0].toXml().c_str();
    if (name.find("trajectory_generation/path") == std::string::npos) {
      continue;
    }
    ROS_INFO("[UAV Status]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    uav_name_property->addOptionStd(name);
    if (taken_uavs.find(name) == taken_uavs.end()) {
      taken_uavs[name] = false;
      ROS_INFO("[UAV Status]: %s was added to global drone names", name.c_str());
    }
    ROS_INFO("[UAV Status]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Find the first occurrence of a false value
  std::string first_available_uav;
  bool        found = false;

  for (const auto& pair : taken_uavs) {
    if (!pair.second) {
      first_available_uav = pair.first;
      found               = true;
      break;
    }
  }

  ROS_INFO("Available uav was %sfound", found ? "" : "not ");
  if (found) {
    uav_name_property->setStdString(first_available_uav);
    taken_uavs[first_available_uav] = true;
  }

  // Searching for initial coordinates
  rviz::DisplayGroup* display_group = context_->getRootDisplayGroup();
  std::pair<int, int> point         = getSpawnCoordinates(dynamic_cast<rviz::Property*>(display_group));
  display_pos_x                     = point.first;
  display_pos_y                     = point.second;

  is_inited = true;
}

// Meant to take "Global Options" property only!
void StatusDisplay::setTextColor(rviz::Property* property) {
  rviz::ColorProperty* color_property = nullptr;
  try {
    color_property = dynamic_cast<rviz::ColorProperty*>(property);
  }
  catch (const std::bad_cast& e) {
    color_property = nullptr;
  }

  if (color_property) {
    int curr_r;
    int curr_g;
    int curr_b;
    color_property->getColor().getRgb(&curr_r, &curr_g, &curr_b);
    int grayscale = (255 - curr_r + 255 - curr_g + 255 - curr_b) / 3;

    text_color_property->setColor(QColor(255 - curr_r, 255 - curr_g, 255 - curr_b, 255));
    bg_color_property->setColor(QColor(grayscale, grayscale, grayscale, 100));
  } else {
    for (int i = 0; i < property->numChildren(); i++) {
      setTextColor(property->childAt(i));
    }
  }
}

std::pair<int, int> StatusDisplay::getSpawnCoordinates(rviz::Property* property) {
  rviz::DisplayGroup* display_group  = nullptr;
  StatusDisplay*      status_display = nullptr;

  if (property->getNameStd() == "Global Options") {
    for (int i = 0; i < property->numChildren(); i++) {
      setTextColor(property->childAt(i));
    }
  }

  try {
    display_group = dynamic_cast<rviz::DisplayGroup*>(property);
  }
  catch (const std::bad_cast& e) {
    display_group = nullptr;
  }

  try {
    status_display = dynamic_cast<StatusDisplay*>(property);
  }
  catch (const std::bad_cast& e) {
    status_display = nullptr;
  }

  if (display_group) {
    std::pair<int, int> res = std::make_pair(0, 0);
    for (int i = 0; i < display_group->numChildren(); i++) {
      std::pair<int, int> current = getSpawnCoordinates(display_group->childAt(i));
      // Taking the biggest from both pairs:
      res = std::make_pair(std::max(current.first, res.first), std::max(current.second, res.second));
    }
    return res;
  }

  if (status_display && status_display->getIsInited()) {
    return status_display->getBottomLine();
  }

  return std::make_pair(0, 0);
}

void StatusDisplay::update(float wall_dt, float ros_dt) {
  if (!isEnabled()) {
    return;
  }

  if (top_line_update_required || global_update_required)
    drawTopLine();
  if (cm_update_required || global_update_required)
    drawControlManager();
  if (odom_update_required || global_update_required)
    drawOdometry();
  if (comp_state_update_required || global_update_required)
    drawGeneralInfo();
  if (hw_api_state_update_required || global_update_required)
    drawHwApiState();
  if (topics_update_required || global_update_required)
    drawCustomTopicRates();
  if (string_update_required || global_update_required)
    drawCustomStrings();
  if (node_stats_update_required || global_update_required)
    drawNodeStats();

  global_update_required = false;
}

void StatusDisplay::drawTopLine() {
  // Control manager overlay
  top_line_overlay->updateTextureSize(581, 20);
  top_line_overlay->setPosition(display_pos_x, display_pos_y);
  top_line_overlay->show(top_line_property->getBool());

  if (!top_line_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = top_line_overlay->getBuffer();

  QImage hud  = buffer.getQImage(*top_line_overlay, bg_color);
  QFont  font = QFont("DejaVu Sans Mono");

  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  QString     tmp;
  std::string avoiding_text;
  if (collision_avoidance_enabled && avoiding_collision) {
    avoiding_text = "!! AVOIDING COLLISION !!";
  } else if (collision_avoidance_enabled && !avoiding_collision) {
    avoiding_text = "COL AVOID ENABLED";
  } else {
    avoiding_text = "COL AVOID DISABLED";
  }
  tmp.sprintf("ToF: %3d:%02d %s %s   %s  UAVs:%d", secs_flown / 60, secs_flown % 60, uav_name.c_str(), uav_type.c_str(),
              avoiding_text.c_str(), num_other_uavs);
  painter.drawStaticText(0, 0, QStaticText(tmp));

  top_line_overlay->setDimensions(top_line_overlay->getTextureWidth(), top_line_overlay->getTextureHeight());
  top_line_update_required = false;
}

void StatusDisplay::drawControlManager() {
  // Control manager overlay
  contol_manager_overlay->updateTextureSize(230, 60);
  contol_manager_overlay->setPosition(display_pos_x, display_pos_y + cm_pos_y);
  contol_manager_overlay->show(control_manager_property->getBool());

  if (!control_manager_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = contol_manager_overlay->getBuffer();
  QImage                              hud    = buffer.getQImage(*contol_manager_overlay, bg_color);
  QFont                               font   = QFont("DejaVu Sans Mono");
  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // Main row
  const QStaticText control_manager_text = QStaticText("Control manager");
  painter.drawStaticText(0, 0, control_manager_text);

  QString tmp;
  tmp.sprintf("%s%.1f Hz", controller_rate >= 10 ? "" : " ", controller_rate);
  QStaticText control_manager_freq_text = QStaticText(tmp);
  painter.drawStaticText(152, 0, control_manager_freq_text);

  // Controller
  QColor  controller_color = NO_COLOR;
  QString controller_text;
  if (controller_rate == 0) {
    controller_color = RED_COLOR;
    controller_text  = "NO CONTROLLER";
  } else {
    if (curr_controller.find("!NO DATA!") != std::string::npos) {
      controller_color = RED_COLOR;
    }
    controller_text = QString("%1/%2").arg(curr_controller.c_str(), curr_gains.c_str());
  }
  QRect controller_rect = painter.boundingRect(0, 20, 0, 0, Qt::AlignLeft, controller_text);
  painter.fillRect(controller_rect, controller_color);
  painter.drawText(controller_rect, Qt::AlignLeft, controller_text);

  QColor  callbacks_color = NO_COLOR;
  QString callbacks_text  = "";
  if (!callbacks_enabled) {
    callbacks_color = RED_COLOR;
    callbacks_text  = "NO_CB";
  }
  QRect callback_rect = painter.boundingRect(0, 40, 0, 0, Qt::AlignLeft, callbacks_text);
  painter.fillRect(callback_rect, callbacks_color);
  painter.drawText(callback_rect, Qt::AlignLeft, callbacks_text);

  // Tracker
  QColor  tracker_color = NO_COLOR;
  QString tracker_text;
  if (controller_rate == 0) {

    tracker_color = RED_COLOR;
    tracker_text  = "NO_TRACKER";

  } else {

    if (curr_controller.find("!NO DATA!") != std::string::npos || null_tracker) {
      tracker_color = RED_COLOR;
    }

    tracker_text = QString("%1/%2").arg(curr_tracker.c_str(), curr_constraints.c_str());
  }
  QRect tracker_rect = painter.boundingRect(0, 40, 0, 0, Qt::AlignLeft, tracker_text);
  painter.fillRect(tracker_rect, tracker_color);
  painter.drawText(tracker_rect, Qt::AlignLeft, tracker_text);

  const QStaticText has_goal_text = QStaticText(QString("%1").arg(has_goal ? " FLY" : "IDLE"));
  painter.drawStaticText(180, 40, has_goal_text);

  contol_manager_overlay->setDimensions(contol_manager_overlay->getTextureWidth(), contol_manager_overlay->getTextureHeight());
  cm_update_required = false;
}

void StatusDisplay::drawOdometry() {
  // Odometry overlay
  odometry_overlay->updateTextureSize(230, 120);
  odometry_overlay->setPosition(display_pos_x, display_pos_y + odom_pos_y);
  odometry_overlay->show(odometry_property->getBool());

  if (!odometry_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = odometry_overlay->getBuffer();

  QImage hud  = buffer.getQImage(*odometry_overlay, bg_color);
  QFont  font = QFont("DejaVu Sans Mono");

  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // Main row
  const QStaticText odometry_text = QStaticText("Odom");
  painter.drawStaticText(108, 0, odometry_text);

  QString tmp;
  tmp.sprintf("%s%.1f Hz", avg_odom_rate >= 100 ? "" : " ", avg_odom_rate);
  QStaticText control_manager_freq_text = QStaticText(tmp);
  painter.drawStaticText(152, 0, control_manager_freq_text);

  if (avg_odom_rate == 0.0) {
    QRect no_data_rect = painter.boundingRect(0, 0, 0, 0, Qt::AlignLeft, "!NO DATA!");
    painter.fillRect(no_data_rect, RED_COLOR);
    painter.drawText(no_data_rect, Qt::AlignLeft, "!NO DATA!");
    odometry_overlay->setDimensions(odometry_overlay->getTextureWidth(), odometry_overlay->getTextureHeight());
    return;
  }

  // XYZ and hdg column
  painter.drawStaticText(0, 20, QStaticText("X"));
  painter.drawStaticText(0, 40, QStaticText("Y"));
  painter.drawStaticText(0, 60, QStaticText("Z"));
  painter.drawStaticText(0, 80, QStaticText("hdg"));

  // XYZ and hdg values
  QString     value;
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
  QString estimator = QString("%1").arg(curr_estimator.c_str());
  painter.drawStaticText(100, 20, QStaticText(odom_frame.c_str()));
  painter.drawStaticText(100, 40, QStaticText(estimator));

  if (!null_tracker) {
    const double cerr_x   = std::fabs(state_x - cmd_x);
    const double cerr_y   = std::fabs(state_y - cmd_y);
    const double cerr_z   = std::fabs(state_z - cmd_z);
    const double cerr_hdg = std::fabs(heading - cmd_hdg);

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

    QString x_err_str;
    QString y_err_str;
    QString z_err_str;
    QString h_err_str;
    x_err_str.sprintf("%.1f", cerr_x);
    y_err_str.sprintf("%.1f", cerr_y);
    z_err_str.sprintf("%.1f", cerr_z);
    h_err_str.sprintf("%.1f", cerr_hdg);

    // Printing constant string and saving coordinates for changeable data
    QRect tmp_rect = painter.boundingRect(0, 100, 0, 0, Qt::AlignLeft, "C/E X");
    painter.drawText(tmp_rect, Qt::AlignLeft, "C/E X");
    QRect x_err_rect = painter.boundingRect(tmp_rect.right(), 100, 0, 0, Qt::AlignLeft, x_err_str);

    tmp_rect = painter.boundingRect(x_err_rect.right(), 100, 0, 0, Qt::AlignLeft, " Y");
    painter.drawText(tmp_rect, Qt::AlignLeft, " Y");
    QRect y_err_rect = painter.boundingRect(tmp_rect.right(), 100, 0, 0, Qt::AlignLeft, y_err_str);

    tmp_rect = painter.boundingRect(y_err_rect.right(), 100, 0, 0, Qt::AlignLeft, " Z");
    painter.drawText(tmp_rect, Qt::AlignLeft, " Z");
    QRect z_err_rect = painter.boundingRect(tmp_rect.right(), 100, 0, 0, Qt::AlignLeft, z_err_str);

    tmp_rect = painter.boundingRect(z_err_rect.right(), 100, 0, 0, Qt::AlignLeft, " H");
    painter.drawText(tmp_rect, Qt::AlignLeft, " H");
    QRect h_err_rect = painter.boundingRect(tmp_rect.right(), 100, 0, 0, Qt::AlignLeft, h_err_str);

    // Printing changeable data
    painter.fillRect(x_err_rect, x_warning_color);
    painter.fillRect(y_err_rect, y_warning_color);
    painter.fillRect(z_err_rect, z_warning_color);
    painter.fillRect(h_err_rect, h_warning_color);

    painter.drawText(x_err_rect, Qt::AlignLeft, x_err_str);
    painter.drawText(y_err_rect, Qt::AlignLeft, y_err_str);
    painter.drawText(z_err_rect, Qt::AlignLeft, z_err_str);
    painter.drawText(h_err_rect, Qt::AlignLeft, h_err_str);
  }

  odom_update_required = false;
  odometry_overlay->setDimensions(odometry_overlay->getTextureWidth(), odometry_overlay->getTextureHeight());
}

void StatusDisplay::drawGeneralInfo() {
  // General info overlay
  general_info_overlay->updateTextureSize(230, 60);
  general_info_overlay->setPosition(display_pos_x + gen_info_pos_x, display_pos_y + gen_info_pos_y);
  general_info_overlay->show(computer_load_property->getBool());

  if (!computer_load_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = general_info_overlay->getBuffer();

  QImage hud  = buffer.getQImage(*general_info_overlay, bg_color);
  QFont  font = QFont("DejaVu Sans Mono");

  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // CPU load
  QColor cpu_load_color = NO_COLOR;
  if (cpu_load > 80.0) {
    cpu_load_color = RED_COLOR;
  } else if (cpu_load > 60.0) {
    cpu_load_color = YELLOW_COLOR;
  }
  QString cpu_load_str;
  cpu_load_str.sprintf("CPU: %.1f%%", cpu_load);
  QRect cpu_load_rect = painter.boundingRect(0, 20, 0, 0, Qt::AlignLeft, cpu_load_str);
  painter.fillRect(cpu_load_rect, cpu_load_color);
  painter.drawText(cpu_load_rect, Qt::AlignLeft, cpu_load_str);

  // CPU frequency
  QString cpu_freq_str;
  cpu_freq_str.sprintf("%.2f GHz", cpu_freq);
  painter.drawStaticText(110, 20, QStaticText(cpu_freq_str));

  // Free RAM
  const double used_ram  = total_ram - ram_free;
  const double ram_ratio = used_ram / total_ram;
  QColor       ram_color = NO_COLOR;
  if (ram_ratio > 0.7) {
    ram_color = RED_COLOR;
  } else if (ram_ratio > 0.5) {
    ram_color = YELLOW_COLOR;
  }
  QString ram_free_str;
  ram_free_str.sprintf("RAM: %.1f G", ram_free);
  QRect ram_rect = painter.boundingRect(0, 40, 0, 0, Qt::AlignLeft, ram_free_str);
  painter.fillRect(ram_rect, ram_color);
  painter.drawText(ram_rect, ram_free_str);

  // Free disk
  QColor free_disk_color = NO_COLOR;
  if (disk_free < 100) {
    free_disk_color = RED_COLOR;
  } else if (disk_free < 200) {
    free_disk_color = YELLOW_COLOR;
  }
  QString disk_free_str;
  if (disk_free < 10000) {
    disk_free_str.sprintf("HDD: %.1f G", disk_free / 10);
  } else {
    disk_free_str.sprintf("HDD: %.1f G", disk_free / 10000);
  }
  QRect disk_free_rect = painter.boundingRect(110, 40, 0, 0, Qt::AlignLeft, disk_free_str);
  painter.fillRect(disk_free_rect, free_disk_color);
  painter.drawText(disk_free_rect, disk_free_str);

  comp_state_update_required = false;
  general_info_overlay->setDimensions(general_info_overlay->getTextureWidth(), general_info_overlay->getTextureHeight());
}

void StatusDisplay::drawHwApiState() {
  // Hw api overlay
  hw_api_state_overlay->updateTextureSize(230, 120);
  hw_api_state_overlay->setPosition(display_pos_x + hw_api_pos_x, display_pos_y + hw_api_pos_y);
  hw_api_state_overlay->show(hw_api_state_property->getBool());

  if (!hw_api_state_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = hw_api_state_overlay->getBuffer();
  QImage                              hud    = buffer.getQImage(*hw_api_state_overlay, bg_color);
  QFont                               font   = QFont("DejaVu Sans Mono");
  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // Main row
  QStaticText mavros_text = QStaticText("Mavros");
  QString     tmp;
  tmp.sprintf("%s%.1f Hz", hw_api_rate >= 100 ? "" : " ", hw_api_rate);
  QStaticText hw_api_freq_text = QStaticText(tmp);
  painter.drawStaticText(94, 0, mavros_text);
  painter.drawStaticText(152, 0, hw_api_freq_text);

  if (hw_api_rate == 0) {  // No data
    QRect no_data_rect = painter.boundingRect(0, 0, 0, 0, Qt::AlignLeft, "!NO DATA!");
    painter.fillRect(no_data_rect, RED_COLOR);
    painter.drawText(no_data_rect, "!NO DATA!");
  }

  // State:
  QRect state_rect = painter.boundingRect(0, 20, 0, 0, Qt::AlignLeft, "State: ");
  painter.drawStaticText(0, 20, QStaticText("State: "));
  if (hw_api_state_rate == 0) {
    QRect error_rect = painter.boundingRect(state_rect.right(), 20, 0, 0, Qt::AlignLeft, "ERROR");
    painter.fillRect(error_rect, RED_COLOR);
    painter.drawText(error_rect, "ERROR");

  } else {
    if (hw_api_armed) {
      painter.drawStaticText(state_rect.right(), 20, QStaticText("ARMED"));
    } else {
      QRect error_rect = painter.boundingRect(state_rect.right(), 20, 0, 0, Qt::AlignLeft, "DISARMED");
      painter.fillRect(error_rect, RED_COLOR);
      painter.drawText(error_rect, "DISARMED");
    }
  }

  // Mode:
  QRect mode_rect = painter.boundingRect(0, 40, 0, 0, Qt::AlignLeft, "Mode: ");
  painter.drawStaticText(0, 40, QStaticText("Mode: "));
  if (hw_api_mode != "OFFBOARD") {
    painter.fillRect(painter.boundingRect(mode_rect.right(), 40, 0, 0, Qt::AlignLeft, QString(hw_api_mode.c_str())), RED_COLOR);
  }
  painter.drawStaticText(mode_rect.right(), 40, QStaticText(hw_api_mode.c_str()));

  // Batt:
  QRect batt_rect = painter.boundingRect(0, 60, 0, 0, Qt::AlignLeft, "Batt: ");
  painter.drawStaticText(0, 60, QStaticText("Batt: "));
  if (hw_api_battery_rate == 0) {

    QRect error_rect = painter.boundingRect(batt_rect.right(), 60, 0, 0, Qt::AlignLeft, "ERROR");
    painter.fillRect(error_rect, RED_COLOR);
    painter.drawStaticText(batt_rect.right(), 60, QStaticText("ERROR"));

  } else {

    const double volt_to_show = (battery_volt > 17.0) ? (battery_volt / 6) : (battery_volt / 4);

    QString volt_str;
    QString curr_str;
    volt_str.sprintf("%.2f V", volt_to_show);
    curr_str.sprintf("  %.2f A", battery_curr);

    if (volt_to_show < 3.6) {
      painter.fillRect(painter.boundingRect(batt_rect.right(), 60, 0, 0, Qt::AlignLeft, volt_str), RED_COLOR);
    } else if (volt_to_show < 3.7) {
      painter.fillRect(painter.boundingRect(batt_rect.right(), 60, 0, 0, Qt::AlignLeft, volt_str), YELLOW_COLOR);
    }

    tmp.sprintf("%.2f V  %.2f A", volt_to_show, battery_curr);
    painter.drawStaticText(batt_rect.right(), 60, QStaticText(tmp));
  }

  // Drained:
  QRect drained_rect = painter.boundingRect(0, 80, 0, 0, Qt::AlignLeft, "Drained: ");
  painter.drawStaticText(0, 80, QStaticText("Drained: "));
  tmp.sprintf("%.1f Wh", battery_wh_drained);
  painter.drawStaticText(drained_rect.right(), 80, QStaticText(tmp));

  // Thrst:
  QRect thrst_rect = painter.boundingRect(0, 100, 0, 0, Qt::AlignLeft, "Thrst: ");
  painter.drawStaticText(0, 100, QStaticText("Thrst: "));
  tmp.sprintf("%.2f", thrust);
  QRect thrst_value_rect = painter.boundingRect(thrst_rect.right(), 100, 0, 0, Qt::AlignLeft, tmp);
  if (thrust > 0.75) {
    painter.fillRect(thrst_value_rect, RED_COLOR);
  } else if (thrust > 0.65) {
    painter.fillRect(thrst_value_rect, YELLOW_COLOR);
  }
  painter.drawStaticText(thrst_rect.right(), 100, QStaticText(tmp));

  // GNSS
  if (!hw_api_gnss_ok) {
    QRect error_rect = painter.boundingRect(160, 20, 0, 0, Qt::AlignLeft, "NO_GPS");
    painter.fillRect(error_rect, RED_COLOR);
    painter.drawText(error_rect, Qt::AlignLeft, "NO_GPS");

  } else {
    painter.drawStaticText(160, 20, QStaticText("GPS_OK"));

    QColor gps_qual_color = RED_COLOR;
    if (hw_api_gnss_qual < 5.0) {
      gps_qual_color = NO_COLOR;
    } else if (hw_api_gnss_qual < 10.0) {
      gps_qual_color = YELLOW_COLOR;
    }

    tmp.sprintf("Q: %.1f", hw_api_gnss_qual);
    QRect qual_rect = painter.boundingRect(160, 40, 0, 0, Qt::AlignLeft, tmp);
    painter.fillRect(qual_rect, gps_qual_color);
    painter.drawText(qual_rect, Qt::AlignLeft, tmp);
  }

  // Mass
  const double mass_diff  = fabs(mass_estimate - mass_set) / mass_set;
  QColor       mass_color = NO_COLOR;
  if (mass_diff > 0.3) {
    mass_color = RED_COLOR;
  } else if (mass_diff > 0.2) {
    mass_color = YELLOW_COLOR;
  }
  tmp.sprintf("%.1f/", mass_set);
  QRect mass_set_rect = painter.boundingRect(115, 100, 0, 0, Qt::AlignLeft, tmp);
  painter.drawText(mass_set_rect, Qt::AlignLeft, tmp);

  tmp.sprintf("%.1f", mass_estimate);
  QRect mass_estim_rect = painter.boundingRect(mass_set_rect.right(), 100, 0, 0, Qt::AlignLeft, tmp);
  painter.fillRect(mass_estim_rect, mass_color);
  painter.drawText(mass_estim_rect, Qt::AlignLeft, tmp);
  painter.drawStaticText(mass_estim_rect.right(), 100, QStaticText("kg"));

  hw_api_state_update_required = false;
  hw_api_state_overlay->setDimensions(hw_api_state_overlay->getTextureWidth(), hw_api_state_overlay->getTextureHeight());
}

void StatusDisplay::drawCustomTopicRates() {
  // Topic rate overlay
  topic_rates_overlay->updateTextureSize(230, 183);
  topic_rates_overlay->setPosition(display_pos_x + topic_rate_pos_x, display_pos_y + topic_rate_pos_y);
  topic_rates_overlay->show(topic_rates_property->getBool());

  if (!topic_rates_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = topic_rates_overlay->getBuffer();

  QImage hud  = buffer.getQImage(*topic_rates_overlay, bg_color);
  QFont  font = QFont("DejaVu Sans Mono");

  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // Drawing topics
  QString frequency;
  for (size_t i = 0; i < custom_topic_vec.size(); i++) {
    frequency.sprintf("%.1f Hz", custom_topic_vec[i].topic_hz);

    painter.drawStaticText(0, 20 * i, QStaticText(custom_topic_vec[i].topic_name.c_str()));

    QRect freq_rect = painter.boundingRect(225, 20 * i, 0, 0, Qt::AlignRight, frequency);
    painter.fillRect(freq_rect, getColor(custom_topic_vec[i].topic_color));
    painter.drawText(freq_rect, Qt::AlignRight, frequency);
  }

  topics_update_required = false;
  topic_rates_overlay->setDimensions(topic_rates_overlay->getTextureWidth(), topic_rates_overlay->getTextureHeight());
}

void StatusDisplay::drawCustomStrings() {
  // Custom string overlay
  custom_strings_overlay->updateTextureSize(230, custom_str_height);
  custom_strings_overlay->setPosition(display_pos_x + custom_str_pos_x, display_pos_y + custom_str_pos_y);
  custom_strings_overlay->show(custom_str_property->getBool());

  if (!custom_str_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = custom_strings_overlay->getBuffer();

  QImage hud  = buffer.getQImage(*custom_strings_overlay, bg_color);
  QFont  font = QFont("DejaVu Sans Mono");

  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // Drawing strings
  for (size_t i = 0; i < custom_string_vec.size(); i++) {
    int         tmp_color          = NORMAL;
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

    QRect rect = painter.boundingRect(0, 20 * i, 0, 0, Qt::AlignLeft, tmp_display_string.c_str());
    painter.fillRect(rect, getColor(tmp_color));
    painter.drawText(rect, tmp_display_string.c_str());
  }

  string_update_required = false;
  custom_strings_overlay->setDimensions(custom_strings_overlay->getTextureWidth(), custom_strings_overlay->getTextureHeight());
}

void StatusDisplay::drawNodeStats() {
  // Rosnode stats overlay
  rosnode_stats_overlay->updateTextureSize(394, node_stats_height);
  rosnode_stats_overlay->setPosition(display_pos_x + node_stats_pos_x, display_pos_y + node_stats_pos_y);
  rosnode_stats_overlay->show(node_stats_property->getBool());

  if (!node_stats_property->getBool()) {
    return;
  }

  // Setting the painter up
  jsk_rviz_plugins::ScopedPixelBuffer buffer = rosnode_stats_overlay->getBuffer();

  QImage hud  = buffer.getQImage(*rosnode_stats_overlay, bg_color);
  QFont  font = QFont("DejaVu Sans Mono");

  font.setBold(true);
  QPainter painter(&hud);
  painter.setFont(font);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color, 2, Qt::SolidLine));

  // Main row
  QString tmp;
  tmp.sprintf("%.1f", cpu_load_total);
  painter.drawStaticText(0, 0, QStaticText("ROS Node CPU usage"));
  painter.drawStaticText(285, 0, QStaticText(tmp));
  painter.drawStaticText(345, 0, QStaticText("CPU %"));

  // Drawing stats
  for (size_t i = 0; i < node_cpu_load_vec.node_names.size(); i++) {
    painter.drawStaticText(0, (i + 1) * 20, QStaticText(node_cpu_load_vec.node_names[i].c_str()));

    QColor tmp_color = getColor(GREEN);
    if (node_cpu_load_vec.cpu_loads[i] > 99.9) {
      tmp_color = RED_COLOR;
    } else if (node_cpu_load_vec.cpu_loads[i] > 49.9) {
      tmp_color = YELLOW_COLOR;
    }

    QString load_text;
    load_text.sprintf("%3.1f", node_cpu_load_vec.cpu_loads[i]);
    QRect load_rect = painter.boundingRect(390, (i + 1) * 20, 0, 0, Qt::AlignRight, load_text);
    painter.fillRect(load_rect, tmp_color);
    painter.drawText(load_rect, Qt::AlignRight, load_text);


    // tmp.sprintf("%5.1f", node_cpu_load_vec.cpu_loads[i]);
    // painter.drawStaticText(345, 20 * (i + 1), QStaticText(tmp));
  }

  node_stats_update_required = false;
  rosnode_stats_overlay->setDimensions(rosnode_stats_overlay->getTextureWidth(), rosnode_stats_overlay->getTextureHeight());
}

void StatusDisplay::reset() {
}

// Helper function
template <typename T>
bool compareAndUpdate(T& new_value, T& current_value) {

  if (new_value != current_value) {
    current_value = new_value;
    return true;
  }

  return false;
}

void StatusDisplay::uavStatusCb(const mrs_msgs::UavStatusConstPtr& msg) {
  processTopLine(msg);
  processControlManager(msg);
  processOdometry(msg);
  processGeneralInfo(msg);
  processHwApiState(msg);
  processCustomTopics(msg);
  processCustomStrings(msg);
  processNodeStats(msg);
}

void StatusDisplay::processTopLine(const mrs_msgs::UavStatusConstPtr& msg) {
  std::string new_uav_name                    = msg->uav_name;
  std::string new_uav_type                    = msg->uav_type;
  bool        new_collision_avoidance_enabled = msg->collision_avoidance_enabled;
  bool        new_avoiding_collision          = msg->avoiding_collision;
  bool        new_automatic_start_can_takeoff = msg->automatic_start_can_takeoff;
  int         new_num_other_uavs              = msg->num_other_uavs;
  int         new_secs_flown                  = msg->secs_flown;

  top_line_update_required |= compareAndUpdate(new_uav_name, uav_name);
  top_line_update_required |= compareAndUpdate(new_uav_type, uav_type);
  top_line_update_required |= compareAndUpdate(new_collision_avoidance_enabled, collision_avoidance_enabled);
  top_line_update_required |= compareAndUpdate(new_avoiding_collision, avoiding_collision);
  top_line_update_required |= compareAndUpdate(new_automatic_start_can_takeoff, automatic_start_can_takeoff);
  top_line_update_required |= compareAndUpdate(new_num_other_uavs, num_other_uavs);
  top_line_update_required |= compareAndUpdate(new_secs_flown, secs_flown);
}

void StatusDisplay::processControlManager(const mrs_msgs::UavStatusConstPtr& msg) {
  bool        new_null_tracker      = msg->null_tracker;
  double      new_rate              = msg->control_manager_diag_hz;
  bool        new_callbacks_enabled = msg->callbacks_enabled;
  bool        new_has_goal          = msg->have_goal;
  std::string new_controller;
  std::string new_tracker;
  std::string new_gains;
  std::string new_constraints;

  msg->controllers.empty() ? new_controller = "NONE" : new_controller = msg->controllers[0];
  msg->trackers.empty() ? new_tracker = "NONE" : new_tracker = msg->trackers[0];
  msg->gains.empty() ? new_gains = "NONE" : new_gains = msg->gains[0];
  msg->constraints.empty() ? new_constraints = "NONE" : new_constraints = msg->constraints[0];

  if (new_null_tracker) {
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
  double      new_avg_odom_rate = msg->odom_hz;
  double      new_color         = msg->odom_color;
  double      new_heading       = msg->odom_hdg;
  double      new_state_x       = msg->odom_x;
  double      new_state_y       = msg->odom_y;
  double      new_state_z       = msg->odom_z;
  double      new_cmd_x         = msg->cmd_x;
  double      new_cmd_y         = msg->cmd_y;
  double      new_cmd_z         = msg->cmd_z;
  double      new_cmd_hdg       = msg->cmd_hdg;
  std::string new_odom_frame    = msg->odom_frame;
  std::string new_curr_estimator;

  new_odom_frame = new_odom_frame.substr(new_odom_frame.find("/") + 1);

  msg->odom_estimators.empty() ? new_curr_estimator = "NONE" : new_curr_estimator = msg->odom_estimators[0];

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
  odom_update_required |= compareAndUpdate(new_curr_estimator, curr_estimator);
}

void StatusDisplay::processGeneralInfo(const mrs_msgs::UavStatusConstPtr& msg) {
  double new_cpu_load  = msg->cpu_load;
  double new_cpu_freq  = msg->cpu_ghz;
  double new_ram_free  = msg->free_ram;
  double new_total_ram = msg->total_ram;
  double new_disk_free = msg->free_hdd;

  comp_state_update_required |= compareAndUpdate(new_cpu_load, cpu_load);
  comp_state_update_required |= compareAndUpdate(new_cpu_freq, cpu_freq);
  comp_state_update_required |= compareAndUpdate(new_ram_free, ram_free);
  comp_state_update_required |= compareAndUpdate(new_total_ram, total_ram);
  comp_state_update_required |= compareAndUpdate(new_disk_free, disk_free);
}

void StatusDisplay::processHwApiState(const mrs_msgs::UavStatusConstPtr& msg) {
  double      new_hw_api_rate         = msg->hw_api_hz;
  double      new_hw_api_state_rate   = msg->hw_api_state_hz;
  double      new_hw_api_cmd_rate     = msg->hw_api_cmd_hz;
  double      new_hw_api_battery_rate = msg->hw_api_battery_hz;
  bool        new_hw_api_gnss_ok      = msg->hw_api_gnss_ok;
  bool        new_hw_api_armed        = msg->hw_api_armed;
  std::string new_hw_api_mode         = msg->hw_api_mode;
  double      new_battery_volt        = msg->battery_volt;
  double      new_battery_curr        = msg->battery_curr;
  double      new_battery_wh_drained  = msg->battery_wh_drained;
  double      new_thrust              = msg->thrust;
  double      new_mass_estimate       = msg->mass_estimate;
  double      new_mass_set            = msg->mass_set;
  double      new_hw_api_gnss_qual    = msg->hw_api_gnss_qual;

  hw_api_state_update_required |= compareAndUpdate(new_hw_api_rate, hw_api_rate);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_state_rate, hw_api_state_rate);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_cmd_rate, hw_api_cmd_rate);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_battery_rate, hw_api_battery_rate);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_gnss_ok, hw_api_gnss_ok);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_armed, hw_api_armed);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_mode, hw_api_mode);
  hw_api_state_update_required |= compareAndUpdate(new_battery_volt, battery_volt);
  hw_api_state_update_required |= compareAndUpdate(new_battery_curr, battery_curr);
  hw_api_state_update_required |= compareAndUpdate(new_battery_wh_drained, battery_wh_drained);
  hw_api_state_update_required |= compareAndUpdate(new_thrust, thrust);
  hw_api_state_update_required |= compareAndUpdate(new_mass_estimate, mass_estimate);
  hw_api_state_update_required |= compareAndUpdate(new_mass_set, mass_set);
  hw_api_state_update_required |= compareAndUpdate(new_hw_api_gnss_qual, hw_api_gnss_qual);
}

void StatusDisplay::processCustomTopics(const mrs_msgs::UavStatusConstPtr& msg) {
  std::vector<mrs_msgs::CustomTopic> new_custom_topic_vec = msg->custom_topics;

  topics_update_required |= compareAndUpdate(new_custom_topic_vec, custom_topic_vec);
}

void StatusDisplay::processCustomStrings(const mrs_msgs::UavStatusConstPtr& msg) {
  std::vector<std::string> new_custom_string_vec = msg->custom_string_outputs;

  string_update_required |= compareAndUpdate(new_custom_string_vec, custom_string_vec);
}

void StatusDisplay::processNodeStats(const mrs_msgs::UavStatusConstPtr& msg) {
  mrs_msgs::NodeCpuLoad new_node_cpu_load_vec = msg->node_cpu_loads;
  double                new_cpu_load_total    = msg->cpu_load_total;

  node_stats_update_required |= compareAndUpdate(new_node_cpu_load_vec, node_cpu_load_vec);
  node_stats_update_required |= compareAndUpdate(new_cpu_load_total, cpu_load_total);
}

void StatusDisplay::nameUpdate() {
  if (taken_uavs.find(last_uav_name) != taken_uavs.end()) {
    taken_uavs[last_uav_name] = false;
  }
  if (taken_uavs.find(uav_name_property->getStdString()) != taken_uavs.end()) {
    taken_uavs[last_uav_name] = true;
  }
  last_uav_name = uav_name_property->getStdString();

  uav_status_sub =
      nh.subscribe(uav_name_property->getStdString() + "/mrs_uav_status/uav_status", 10, &StatusDisplay::uavStatusCb, this, ros::TransportHints().tcpNoDelay());

  // Controller
  curr_controller     = "!NO DATA!";
  curr_tracker        = "!NO DATA!";
  curr_gains          = "";
  curr_constraints    = "";
  avg_controller_rate = 0.0;
  cm_update_required  = true;

  // Odometry
  odom_frame           = "!NO DATA!";
  curr_estimator  = "!NO DATA!";
  avg_odom_rate        = 0.0;
  odom_update_required = true;

  // General info
  comp_state_update_required = true;
}

void StatusDisplay::colorFgUpdate() {
  fg_color               = text_color_property->getColor();
  global_update_required = true;
}

void StatusDisplay::colorBgUpdate() {
  bg_color               = bg_color_property->getColor();
  global_update_required = true;
  bg_color.setAlpha(100);
}

void StatusDisplay::topLineUpdate() {
  top_line_update_required = true;
  controlManagerUpdate();
}

void StatusDisplay::controlManagerUpdate() {
  cm_update_required        = true;
  present_columns[CM_INDEX] = control_manager_property->getBool() || odometry_property->getBool();
  cm_pos_y                  = top_line_property->getBool() ? 23 : 0;
  odometryUpdate();
  computerLoadUpdate();
  return;
}

void StatusDisplay::odometryUpdate() {
  odom_update_required = true;
  if (!odometry_property->getBool()) {

    if (!control_manager_property->getBool()) {
      present_columns[ODOM_INDEX] = false;
    }

    computerLoadUpdate();
    return;
  }

  present_columns[ODOM_INDEX] = true;

  odom_pos_y = 0;
  odom_pos_y += top_line_property->getBool() ? 23 : 0;
  odom_pos_y += control_manager_property->getBool() ? 63 : 0;

  computerLoadUpdate();
}

void StatusDisplay::computerLoadUpdate() {
  comp_state_update_required = true;
  if (!computer_load_property->getBool()) {

    if (!hw_api_state_property->getBool()) {
      present_columns[GEN_INFO_INDEX] = false;
    }

    hwApiStateUpdate();
    topicRatesUpdate();
    return;
  }

  present_columns[GEN_INFO_INDEX] = true;

  gen_info_pos_x = 0;
  for (int i = 0; i < GEN_INFO_INDEX; ++i) {
    gen_info_pos_x += present_columns[i] ? 233 : 0;
  }
  gen_info_pos_y = top_line_property->getBool() ? 23 : 0;
  hwApiStateUpdate();
}

void StatusDisplay::hwApiStateUpdate() {
  hw_api_state_update_required = true;
  if (!hw_api_state_property->getBool()) {

    if (!computer_load_property->getBool()) {
      present_columns[HW_API_STATE_INDEX] = false;
    }

    topicRatesUpdate();
    return;
  }

  present_columns[HW_API_STATE_INDEX] = true;

  hw_api_pos_x = 0;
  for (int i = 0; i < HW_API_STATE_INDEX; ++i) {
    hw_api_pos_x += present_columns[i] ? 233 : 0;
  }

  hw_api_pos_y = 0;
  hw_api_pos_y += top_line_property->getBool() ? 23 : 0;
  hw_api_pos_y += computer_load_property->getBool() ? 63 : 0;
  topicRatesUpdate();
}

void StatusDisplay::topicRatesUpdate() {
  topics_update_required = true;
  if (!topic_rates_property->getBool()) {
    present_columns[TOPIC_RATE_INDEX] = false;
    customStrUpdate();
    return;
  }

  present_columns[TOPIC_RATE_INDEX] = true;

  topic_rate_pos_x = 0;
  topic_rate_pos_y = top_line_property->getBool() ? 23 : 0;
  for (int i = 0; i < TOPIC_RATE_INDEX; ++i) {
    topic_rate_pos_x += present_columns[i] ? 233 : 0;
  }

  customStrUpdate();
}

void StatusDisplay::customStrUpdate() {
  string_update_required = true;
  if (!custom_str_property->getBool()) {
    present_columns[CUSTOM_STR_INDEX] = false;
    nodeStatsUpdate();
    return;
  }

  present_columns[CUSTOM_STR_INDEX] = true;

  int col_num      = 0;
  custom_str_pos_x = 0;
  for (int i = 0; i < CUSTOM_STR_INDEX; ++i) {
    custom_str_pos_x += present_columns[i] ? 233 : 0;
    col_num += present_columns[i] ? 1 : 0;
  }

  if (col_num < CUSTOM_STR_INDEX) {
    custom_str_pos_y  = top_line_property->getBool() ? 23 : 0;
    custom_str_height = 183;
  } else {
    custom_str_pos_y  = 0;
    custom_str_height = top_line_property->getBool() ? 206 : 183;
  }

  nodeStatsUpdate();
}

void StatusDisplay::nodeStatsUpdate() {
  node_stats_update_required = true;
  if (!node_stats_property->getBool()) {
    present_columns[NODE_STATS_INDEX] = false;
    return;
  }

  present_columns[NODE_STATS_INDEX] = true;

  int col_num      = 1;
  node_stats_pos_x = 0;
  for (int i = 0; i < NODE_STATS_INDEX; ++i) {
    node_stats_pos_x += present_columns[i] ? 233 : 0;
    col_num += present_columns[i] ? 1 : 0;
  }

  if (col_num < NODE_STATS_INDEX) {
    node_stats_pos_y  = top_line_property->getBool() ? 23 : 0;
    node_stats_height = 183;
  } else {
    node_stats_pos_y  = 0;
    node_stats_height = top_line_property->getBool() ? 206 : 183;
  }
}

void StatusDisplay::movePosition(const int x, const int y) {
  setPosition(x, y);
}

void StatusDisplay::setPosition(const int x, const int y) {
  display_pos_x = x;
  display_pos_y = y;
  topLineUpdate();
}

std::pair<int, int> StatusDisplay::getBottomLine() {
  if (!isEnabled() || !is_inited) {
    return std::make_pair(0, 0);
  }
  int right  = 0;
  int bottom = 0;
  for (int i = 0; i <= NODE_STATS_INDEX; ++i) {
    right += present_columns[i] ? 233 : 0;
  }
  bottom += top_line_property->getBool() ? 23 : 0;
  bottom += right > 0 ? 186 : 23;
  right += node_stats_property->getBool() ? 394 : 0;

  return std::make_pair(display_pos_x, display_pos_y + bottom + 3);
}

bool StatusDisplay::isInRegion(const int x, const int y) {
  if (!isEnabled()) {
    return false;
  }
  int right  = 0;
  int bottom = 0;
  for (int i = 0; i <= NODE_STATS_INDEX; ++i) {
    right += present_columns[i] ? 233 : 0;
  }
  bottom += top_line_property->getBool() ? 23 : 0;
  bottom += right > 0 ? 186 : 23;
  right += node_stats_property->getBool() ? 394 : 0;
  if (right == 0 && top_line_property->getBool()) {
    right = 581;
  }

  return (display_pos_y < y && display_pos_y + bottom > y && display_pos_x < x && display_pos_x + right > x);
}

void StatusDisplay::onEnable() {
  top_line_overlay->show();
  contol_manager_overlay->show();
  odometry_overlay->show();
  general_info_overlay->show();
  hw_api_state_overlay->show();
  topic_rates_overlay->show();
  custom_strings_overlay->show();
  rosnode_stats_overlay->show();
  global_update_required = true;
}

void StatusDisplay::onDisable() {
  top_line_overlay->hide();
  contol_manager_overlay->hide();
  odometry_overlay->hide();
  general_info_overlay->hide();
  hw_api_state_overlay->hide();
  topic_rates_overlay->hide();
  custom_strings_overlay->hide();
  rosnode_stats_overlay->hide();
  global_update_required = true;
}

}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StatusDisplay, rviz::Display)
