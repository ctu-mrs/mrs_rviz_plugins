#include "control/im_server.h"
#include <algorithm>

using namespace visualization_msgs;

namespace mrs_rviz_plugins{

ImServer::ImServer(){
  nh = ros::NodeHandle();
  checkNewDrones(ros::TimerEvent());
  check_new_drones = nh.createTimer(ros::Duration(3.0), &ImServer::checkNewDrones, this);
}

ImServer::~ImServer(){
  for(auto drone_ptr : drones){
    delete drone_ptr.second;
  }
  drones.clear();
}

void ImServer::addDrone(const std::string& name) {
  drones.insert(std::make_pair(name, new DroneEntity(name)));
}

void ImServer::checkNewDrones(const ros::TimerEvent&){
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
    // "control_manager/diagnostics" does not work for some reason
    if (name.find("trajectory_generation/path") == std::string::npos) {
      continue;
    }

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    state[x] = name;
  }

  for(const std::string& uav_name : drone_names){
    if(drones.find(uav_name) == drones.end()){
      addDrone(uav_name);
    }
  }

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

bool ImServer::select(const std::vector<std::string>& names) {
  selected_drones.clear();
  for(const std::string& name : names){
    auto drone = drones.find(name);
    if(drone == drones.end()){
      ROS_ERROR("[Control tool]: Selected drone is not found among existing drones");
      selected_drones.clear();
      return false;
    }
    selected_drones.push_back(drone->second);
  }
  return true;
}

boost::shared_ptr<QMenu> ImServer::getMenu() {
  if(selected_drones.empty()){
    ROS_INFO("[Control tool]: No drone has been selected");
    return (boost::shared_ptr<QMenu>());
  }

  // Create basic instances
  menu.reset(new QMenu());
  QAction* land              = new QAction("Land",            menu.get());
  QAction* land_home       = new QAction("Land Home",       menu.get());
  QAction* takeoff         = new QAction("Takeoff",         menu.get());
  QMenu* set_constraints   = new QMenu("Set Constraints",   menu.get());
  QMenu* set_gains         = new QMenu("Set Gains",         menu.get());
  QMenu* set_controller    = new QMenu("Set Controller",    menu.get());
  QMenu* set_tracker       = new QMenu("Set Tracker",       menu.get());
  QMenu* set_odom_source   = new QMenu("Set Odom Source",   menu.get());
  QMenu* set_lat_estimator = new QMenu("Set Lat Estimator", menu.get());
  QMenu* set_alt_estimator = new QMenu("Set Alt Estimator", menu.get());
  QMenu* set_hdg_estimator = new QMenu("Set Hdg Estimator", menu.get());

  // Connect signals
  connect(land, &QAction::triggered, this, &ImServer::landNow);
  connect(land_home, &QAction::triggered, this, &ImServer::landHome);
  connect(takeoff, &QAction::triggered, this, &ImServer::takeoffNow);

  // Check state of each drone, and if they are different, do not put Land and Takeoff
  const bool first_value = selected_drones[0]->getNullTracker();
  bool is_same = true;
  for(auto& selected_drone : selected_drones){
    is_same &= first_value == selected_drone->getNullTracker();
  }
  if(is_same && first_value){
    menu->addAction(takeoff);
  } else if(is_same && !first_value){
    menu->addAction(land);
    menu->addAction(land_home);
  }

  // Filter the options that are not present in every selected drone
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
  for(const auto& option : constraint_options){
    QAction* action = new QAction(option.c_str(), set_constraints);
    connect(action, &QAction::triggered, this, [this, option](){setConstraints(option);});
    set_constraints->addAction(action);
  }
  menu->addMenu(set_constraints);

  for(const auto& option : gain_options){
    QAction* action = new QAction(option.c_str(), set_gains);
    connect(action, &QAction::triggered, this, [this, option](){setGains(option);});
    set_gains->addAction(action);
  }
  menu->addMenu(set_gains);

  for(const auto& option : controller_options){
    QAction* action = new QAction(option.c_str(), set_controller);
    connect(action, &QAction::triggered, this, [this, option](){setControllers(option);});
    set_controller->addAction(action);
  }
  menu->addMenu(set_controller);

  for(const auto& option : tracker_options){
    QAction* action = new QAction(option.c_str(), set_tracker);
    connect(action, &QAction::triggered, this, [this, option](){setTrackers(option);});
    set_tracker->addAction(action);
  }
  menu->addMenu(set_tracker);

  for(const auto& option : odom_source_options){
    QAction* action = new QAction(option.c_str(), set_odom_source);
    connect(action, &QAction::triggered, this, [this, option](){setOdomSources(option);});
    set_odom_source->addAction(action);
  }
  menu->addMenu(set_odom_source);

  for(const auto& option : odom_lat_estimtor_options){
    QAction* action = new QAction(option.c_str(), set_lat_estimator);
    connect(action, &QAction::triggered, this, [this, option](){setLatEstimators(option);});
    set_lat_estimator->addAction(action);
  }
  menu->addMenu(set_lat_estimator);

  for(const auto& option : odom_alt_estimtor_options){
    QAction* action = new QAction(option.c_str(), set_alt_estimator);
    connect(action, &QAction::triggered, this, [this, option](){setAltEstimators(option);});
    set_alt_estimator->addAction(action);
  }
  menu->addMenu(set_alt_estimator);

  for(const auto& option : odom_hdg_estimtor_options){
    QAction* action = new QAction(option.c_str(), set_hdg_estimator);
    connect(action, &QAction::triggered, this, [this, option](){setHdgEstimators(option);});
    set_hdg_estimator->addAction(action);
  }
  menu->addMenu(set_hdg_estimator);

  return menu;
}

void ImServer::flyForwardSelected(){
  for(const auto& drone : selected_drones){
    drone->flyForward();
  }
}

void ImServer::flyBackwardSelected(){
  for(const auto& drone : selected_drones){
    drone->flyBackward();
  }
}

void ImServer::flyRightSelected(){
  for(const auto& drone : selected_drones){
    drone->flyRight();
  }
}

void ImServer::flyLeftSelected(){
  for(const auto& drone : selected_drones){
    drone->flyLeft();
  }
}

void ImServer::flyUpSelected(){
  for(const auto& drone : selected_drones){
    drone->flyUp();
  }
}

void ImServer::flyDownSelected(){
  for(const auto& drone : selected_drones){
    drone->flyDown();
  }
}

void ImServer::rotateClockwiseSelected(){
  for(const auto& drone : selected_drones){
    drone->rotateClockwise();
  }
}

void ImServer::rotateAntiClockwiseSelected(){
  for(const auto& drone : selected_drones){
    drone->rotateAntiClockwise();
  }
}

void ImServer::landNow() {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->land();
  }
  if(res){
    ROS_INFO("[Control tool]: selected drones landed successfully");
  } else{
    ROS_INFO("[Control toll]: landing failed on one or more drones");
  }
}

void ImServer::landHome() {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->landHome();
  }
  if(res){
    ROS_INFO("[Control tool]: selected drones landed home successfully");
  } else{
    ROS_INFO("[Control toll]: landing home failed on one or more drones");
  }
}

void ImServer::takeoffNow() {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->takeoff();
  }
  if(res){
    ROS_INFO("[Control tool]: selected drones took off successfully");
  } else{
    ROS_INFO("[Control toll]: taking off failed on one or more drones");
  }
}

void ImServer::setConstraints(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setConstraint(value);
  }
  if(res){
    ROS_INFO("[Control tool]: constraints are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting constraints failed on one or more drones");
  }
}

void ImServer::setGains(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setGain(value);
  }
  if(res){
    ROS_INFO("[Control tool]: gains are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting gains failed on one or more drones");
  }
}

void ImServer::setControllers(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setController(value);
  }
  if(res){
    ROS_INFO("[Control tool]: controllers are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting controllers failed on one or more drones");
  }
}

void ImServer::setTrackers(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setTracker(value);
  }
  if(res){
    ROS_INFO("[Control tool]: trackers are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting trackers failed on one or more drones");
  }
}

void ImServer::setOdomSources(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setOdomSource(value);
  }
  if(res){
    ROS_INFO("[Control tool]: odometry sources are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting odomentry sources failed on one or more drones");
  }
}

void ImServer::setLatEstimators(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setLatEstimator(value);
  }
  if(res){
    ROS_INFO("[Control tool]: lat estimators are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting lat estimators failed on one or more drones");
  }
}

void ImServer::setAltEstimators(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setAltEstimator(value);
  }
  if(res){
    ROS_INFO("[Control tool]: alt estimators are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting alt estimators failed on one or more drones");
  }
}

void ImServer::setHdgEstimators(const std::string& value) {
  bool res = true;
  for(const auto& drone : selected_drones){
    res &= drone->setHdgEstimator(value);
  }
  if(res){
    ROS_INFO("[Control tool]: hdg estimators are set successfully");
  } else{
    ROS_INFO("[Control toll]: setting hdg estimators failed on one or more drones");
  }
}

} // namespace mrs_rviz_plugins