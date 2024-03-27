#include "coverage_path_planning/planner_tool.h"
#include "coverage_path_planning/coverage_method.h"

#include <rviz/visualization_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/render_panel.h>

#include <mrs_msgs/GetSafeZoneAtHeight.h>

#include <QMenu>

namespace mrs_rviz_plugins
{
PlannerTool::PlannerTool() : method_loader("mrs_rviz_plugins", "mrs_rviz_plugins::CoverageMethod"){
  ROS_INFO("Constructor called");
  shortcut_key_ = 'p';

  method_property = new rviz::EnumProperty("Used method", "None", "Choose the algorithm to plan the coverage path", getPropertyContainer(), 
                        SLOT(methodChosen()), this);
  height_property = new rviz::FloatProperty("Height", 3.0F, "The height of the flight", getPropertyContainer(), 
                        SLOT(flightChanged()), this);
  drone_name_property = new rviz::EditableEnumProperty("Main Safety area manager", "uav1", "Safety area of this drone will be used to plan the coverage path.", 
                        getPropertyContainer(), SLOT(droneChanged()), this);

  for(auto& name : method_loader.getDeclaredClasses()){
    method_property->addOptionStd(name);
  }

  ROS_INFO("Planner Tool constructed");
}

void PlannerTool::onInitialize(){
  setName("Coverage path");
  root_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

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
    ROS_INFO("[Waypoint planner]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    ROS_INFO("[Waypoint planner]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Set up drone's name
  if (drone_names.size() == 0) {

    setStatus("Warning: No drone was found. Drone name set to: uav1");
    drone_name_property->setString("uav1");

  } else if (drone_names.size() > 1) {

    for(std::string& name : drone_names){
      drone_name_property->addOptionStd(name);
    }
    drone_name_property->setStdString(drone_names[0]);
    setStatus("Warning: Several drones found. Please, set drone name property");

  } else {

    drone_name_property->setStdString(drone_names[0]);
    setStatus("Drone name is set to " + drone_name_property->getString());
  }

  client_ = nh_.serviceClient<mrs_msgs::GetSafeZoneAtHeight>("/" + drone_name_property->getStdString() + 
              "/uav1/safety_area_manager/get_safety_zone_at_height");
}

int PlannerTool::processMouseEvent(rviz::ViewportMouseEvent& event){
  if (event.panel->contextMenuVisible()){
    return Render;
  }

  if(!event.rightDown()){
    return Render;
  }

  rviz::RenderPanel* render_panel = dynamic_cast<rviz::VisualizationManager*>(context_)->getRenderPanel();
  boost::shared_ptr<QMenu> menu;
  menu.reset(new QMenu());

  QAction* add_obstacle = new QAction("Update the polygon ", menu.get());
  connect(add_obstacle, &QAction::triggered, this, &PlannerTool::updatePolygon);

  QAction* save_config = new QAction("Compute path", menu.get());
  connect(save_config, &QAction::triggered, this, &PlannerTool::computePath);

  QAction* load_config = new QAction("Start mission", menu.get());
  connect(load_config, &QAction::triggered, this, &PlannerTool::startMission);

  menu->addAction(add_obstacle);
  menu->addAction(save_config);
  menu->addAction(load_config);
  render_panel->showContextMenu(menu);

  return Render;
}

void PlannerTool::activate() {
  root_node->setVisible(true);
  ROS_INFO("[PlannerTool]: Activated");
}

void PlannerTool::deactivate() {
  root_node->setVisible(false);
  ROS_INFO("[PlannerTool]: Deactivated");
}

void PlannerTool::methodChosen() {
  ROS_INFO("Method has been chosen");
  if(!current_coverage_method){
    scene_manager_->destroySceneNode(root_node);
    root_node = nullptr;
  }

  if(method_property->getString() == "None"){
    return;
  }

  try{
    current_coverage_method = method_loader.createInstance(method_property->getStdString());
  }
  catch (pluginlib::PluginlibException& ex){
    ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
    setStatus("Error: failed to load plugin " + method_property->getString());
    method_property->setString("None");
    return;
  }

  root_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  current_coverage_method->initialize(getPropertyContainer(), scene_manager_, root_node);
}

void PlannerTool::heightChanged(){
  // TODO: implement me!
}

void PlannerTool::updatePolygon(){
  ROS_INFO("update called");


}

void PlannerTool::computePath(){
  ROS_INFO("compute called");
}

void PlannerTool::startMission(){
  ROS_INFO("start called");
}

void PlannerTool::droneChanged(){
  client_ = nh_.serviceClient<mrs_msgs::GetSafeZoneAtHeight>("/" + drone_name_property->getStdString() + 
              "/uav1/safety_area_manager/get_safety_zone_at_height");
}

}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::PlannerTool, rviz::Tool)