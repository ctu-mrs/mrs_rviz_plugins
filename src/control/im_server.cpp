#include "control/im_server.h"
#include <algorithm>

using namespace visualization_msgs;

namespace mrs_rviz_plugins{

ImServer::ImServer(){
  
}

// TODO: add timer to check for new drones
void ImServer::addDrone(const std::string name) {
  ROS_INFO("drone added %s", name.c_str());
  drones.insert(std::make_pair(name, new DroneEntity(name)));
}

// If element of "possible" is not in "present", deletes it
void chooseOptions(std::vector<std::string>& possible, const std::vector<std::string>& present){
  for(auto option_iter = possible.begin(); option_iter != possible.end(); ){
    auto iter = std::find(present.begin(), present.end(), *option_iter);
    if(iter == present.end()){
      option_iter = possible.erase(option_iter);
    }else{
      ++option_iter;
    }
  }
}

boost::shared_ptr<QMenu> ImServer::getMenu(std::vector<std::string>& drone_names) {
  selected_drones.clear();
  for(std::string name : drone_names){

    ROS_INFO("drone selected: %s", name.c_str());
    auto drone = drones.find(name);
    if(drone == drones.end()){
      ROS_ERROR("[Control tool]: Selected drone has not been found among existing drones");
      selected_drones.clear();
      return (boost::shared_ptr<QMenu>());
    }
    selected_drones.push_back(drone->second);
  }

  if(selected_drones.empty()){
    ROS_INFO("No drone has been selected");
    return (boost::shared_ptr<QMenu>());
  }

  // Note: sub-menus are defined on topic /uav_name/mrs_uav_status/uav_status
  // One more note: actions can be connected through lambda functions

  // Create basic instances
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

  // TODO: check state of each drone, and if they are different, do not put Land and Takeoff
  menu->addAction(land);
  menu->addAction(land_home);
  menu->addAction(takeoff);

  // Filter the options that are not present in every selected drone
  ROS_INFO("Started filtering");
  DroneEntity* drone = selected_drones[0];

  std::vector<std::string> constraint_options        = drone->getConstraints();
  std::vector<std::string> gain_options              = drone->getGains();
  std::vector<std::string> controller_options        = drone->getControllers();
  std::vector<std::string> tracker_options           = drone->getTrackers();
  std::vector<std::string> odom_source_options       = drone->getOdomSources();
  std::vector<std::string> odom_lat_estimtor_options = drone->getLatEstimators();
  std::vector<std::string> odom_alt_estimtor_options = drone->getAltEstimators();
  std::vector<std::string> odom_hdg_estimtor_options = drone->getHdgEstimators();
  std::vector<std::string> present_options;

  for(auto& selected_drone : selected_drones){
    present_options = selected_drone->getConstraints();
    chooseOptions(constraint_options, present_options);
    
    present_options = selected_drone->getGains();
    chooseOptions(gain_options, present_options);

    present_options = selected_drone->getControllers();
    chooseOptions(controller_options, present_options);

    present_options = selected_drone->getTrackers();
    chooseOptions(tracker_options, present_options);

    present_options = selected_drone->getOdomSources();
    chooseOptions(odom_source_options, present_options);

    present_options = selected_drone->getLatEstimators();
    chooseOptions(odom_lat_estimtor_options, present_options);

    present_options = selected_drone->getAltEstimators();
    chooseOptions(odom_alt_estimtor_options, present_options);

    present_options = selected_drone->getHdgEstimators();
    chooseOptions(odom_hdg_estimtor_options, present_options);
  }

  // Set menu actions
  ROS_INFO("Started setting menu actions");
  for(auto option : constraint_options){
    QAction* action = new QAction(option.c_str(), set_constraints);
    connect(action, &QAction::triggered, this, [this, option](){setConstraints(option);});
    set_constraints->addAction(action);
  }
  menu->addMenu(set_constraints);

  for(auto option : gain_options){
    QAction* action = new QAction(option.c_str(), set_gains);
    connect(action, &QAction::triggered, this, [this, option](){setGains(option);});
    set_gains->addAction(action);
  }
  menu->addMenu(set_gains);

  for(auto option : controller_options){
    QAction* action = new QAction(option.c_str(), set_controller);
    connect(action, &QAction::triggered, this, [this, option](){setController(option);});
    set_controller->addAction(action);
  }
  menu->addMenu(set_controller);

  for(auto option : tracker_options){
    QAction* action = new QAction(option.c_str(), set_tracker);
    connect(action, &QAction::triggered, this, [this, option](){setTracker(option);});
    set_tracker->addAction(action);
  }
  menu->addMenu(set_tracker);

  for(auto option : odom_source_options){
    QAction* action = new QAction(option.c_str(), set_odom_source);
    connect(action, &QAction::triggered, this, [this, option](){setOdomSource(option);});
    set_odom_source->addAction(action);
  }
  menu->addMenu(set_odom_source);

  for(auto option : odom_lat_estimtor_options){
    QAction* action = new QAction(option.c_str(), set_lat_estimator);
    connect(action, &QAction::triggered, this, [this, option](){setLatEstimator(option);});
    set_lat_estimator->addAction(action);
  }
  menu->addMenu(set_lat_estimator);

  for(auto option : odom_alt_estimtor_options){
    QAction* action = new QAction(option.c_str(), set_alt_estimator);
    connect(action, &QAction::triggered, this, [this, option](){setAltEstimator(option);});
    set_alt_estimator->addAction(action);
  }
  menu->addMenu(set_alt_estimator);

  for(auto option : odom_hdg_estimtor_options){
    QAction* action = new QAction(option.c_str(), set_hdg_estimator);
    connect(action, &QAction::triggered, this, [this, option](){setHdgEstimator(option);});
    set_hdg_estimator->addAction(action);
  }
  menu->addMenu(set_hdg_estimator);

  ROS_INFO("Menu made");
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