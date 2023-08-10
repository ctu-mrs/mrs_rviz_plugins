#include "control/im_server.h"

using namespace visualization_msgs;

namespace mrs_rviz_plugins{

ImServer::ImServer(){
  // Create instances
  menu.reset(new QMenu());
  land              = new QAction("Land",            menu.get());
  land_home         = new QAction("Land Home",       menu.get());
  takeoff           = new QAction("Takeoff",         menu.get());
  set_constraints   = new QMenu("Set Constraints",   menu.get());
  set_gains         = new QMenu("Set Gains",         menu.get());
  set_controller    = new QMenu("Set Controller",    menu.get());
  set_tracker       = new QMenu("Set Tracker",       menu.get());
  set_odom_source   = new QMenu("Set Odom Source",   menu.get());
  set_lat_estimator = new QMenu("Set Lat Estimator", menu.get());
  set_alt_estimator = new QMenu("Set Alt Estimator", menu.get());
  set_hdg_estimator = new QMenu("Set Hdg Estimator", menu.get());

  // Connect signals
  connect(land, &QAction::triggered, this, &ImServer::landNow);
  connect(land_home, &QAction::triggered, this, &ImServer::landHome);
  connect(takeoff, &QAction::triggered, this, &ImServer::takeoffNow);
}

// TODO: add timer to check for new drones
void ImServer::addDrone(const std::string name) {
  drones.push_back(new DroneEntity(name));
}

boost::shared_ptr<QMenu> ImServer::getMenu(std::vector<std::string>& drone_names) {
  selected_drones = drone_names;
  // Note: sub-menus are defined on topic /uav_name/mrs_uav_status/uav_status
  // One more note: actions can be connected through lambda functions

  boost::shared_ptr<QMenu> menu;
  menu.reset(new QMenu());

  // TODO: check state of each drone, and if they are different, do not put Land and Takeoff
  menu->addAction(land);
  menu->addAction(land_home);
  menu->addAction(takeoff);

  menu->addMenu(set_constraints);
  return menu;
}

void ImServer::landNow() {
  ROS_INFO("Land called.");
}
void ImServer::landHome() {
  ROS_INFO("Land Home called.");
}
void ImServer::takeoffNow() {
  ROS_INFO("Takeoff called.");
}
void ImServer::setConstraints(std::string value) {
  ROS_INFO("Set Constraints called. Value: %s", value.c_str());
}
void ImServer::setGains(std::string value) {
  ROS_INFO("Set Gains called. Value: %s", value.c_str());
}
void ImServer::setController(std::string value) {
  ROS_INFO("Set Controller called. Value: %s", value.c_str());
}
void ImServer::setTracker(std::string value) {
  ROS_INFO("Set Tracker called. Value: %s", value.c_str());
}
void ImServer::setOdomSource(std::string value) {
  ROS_INFO("Set Odom Soutce called. Value: %s", value.c_str());
}
void ImServer::setLatEstimator(std::string value) {
  ROS_INFO("Set Lat Estimator called. Value: %s", value.c_str());
}
void ImServer::setAltEstimator(std::string value) {
  ROS_INFO("Set Alt Estimator called. Value: %s", value.c_str());
}
void ImServer::setHdgEstimator(std::string value) {
  ROS_INFO("Set Hdg Estimator called. Value: %s", value.c_str());
}

} // namespace mrs_rviz_plugins