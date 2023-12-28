#include "world_manager/world_manager.h"

#include <rviz/viewport_mouse_event.h>
#include <rviz/render_panel.h>
#include <rviz/geometry.h>

#include <mrs_msgs/ReferenceStamped.h>

#include <QMenu>
#include <QApplication>
#include <QFileDialog>
namespace mrs_rviz_plugins
{
WorldManager::WorldManager() {
  // Note: this tool has full functionality of InteractionTool, 
  // so shortcut could be 'i' instead, if there is no InteractionTool in tool bar
  shortcut_key_ = 'e';
}

void WorldManager::onInitialize(){
  InteractionTool::onInitialize();
  setName("World manager");

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
    ROS_INFO("[WorldManager]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    ROS_INFO("[WorldManager]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Set up drone's name
  if (drone_names.size() == 0) {
    setStatus("Warning: No drone was found. Drone name set to: uav1");
  } else {
    for(size_t i=0; i<drone_names.size(); i++){
      rviz::BoolProperty* tmp  = new rviz::BoolProperty(drone_names[i].c_str(), true, "Actions of the tool will affect chosen uav's SafetyAreaManger", getPropertyContainer());
      properties.push_back(tmp);
      add_obstacle_clients.push_back(node_handler.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + drone_names[i] + "/safety_area_manager/add_obstacle"));
      save_config_clients.push_back(node_handler.serviceClient<mrs_msgs::String>("/" + drone_names[i] + "/safety_area_manager/save_world_config"));
    }
    setStatus("Several drones found.");
  }
}

int WorldManager::processMouseEvent(rviz::ViewportMouseEvent& event) {
  int res = InteractionTool::processMouseEvent(event);

  if (event.panel->contextMenuVisible()){
    return res;
  }

  if(!focused_object_.empty()){
    return res;
  }

  if(!event.rightDown()){
    return res;
  }

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection)) {
    return res;
  }

  current_intersection = intersection;

  rviz::RenderPanel* render_panel = dynamic_cast<rviz::VisualizationManager*>(context_)->getRenderPanel();
  boost::shared_ptr<QMenu> menu;
  menu.reset(new QMenu());

  QAction* add_obstacle = new QAction("Add obstacle", menu.get());
  connect(add_obstacle, &QAction::triggered, this, &WorldManager::add_obstacle);

  QAction* save_config = new QAction("Save world config", menu.get());
  connect(save_config, &QAction::triggered, this, &WorldManager::save_config);

  menu->addAction(add_obstacle);
  menu->addAction(save_config);
  render_panel->showContextMenu(menu);

  return res | Render;
}

void WorldManager::add_obstacle() {
  if(properties.size() == 0){
    ROS_WARN("[WorldManager]: No drone has been detected!");
    return;
  }

  ROS_INFO("add obstacle called");

  mrs_msgs::ReferenceStampedSrv srv;
  srv.request.header.frame_id = context_->getFrameManager()->getFixedFrame();
  srv.request.reference.position.x = current_intersection.x;
  srv.request.reference.position.y = current_intersection.y;
  srv.request.reference.position.z = 0; // SafetyAreaManger doesn't condiser z offset

  for(size_t i=0; i<properties.size(); i++){
    if(!properties[i]->getBool()){
      ROS_INFO("Skipping drone %s", properties[i]->getNameStd().c_str());
      continue;
    }
    if(!add_obstacle_clients[i].call(srv)){
      ROS_WARN("[WorldManager]: Could not call add_obstacle service for drone %s", properties[i]->getNameStd().c_str());
      continue;
    }
    if(!srv.response.success){
      ROS_WARN("[WorldManager]: Could not add obstacle for drone %s: %s", properties[i]->getNameStd().c_str(), srv.response.message.c_str());
      continue;
    }
    ROS_INFO("[WorldManager]: Obstacles has been added successfully");
  }
}

void WorldManager::save_config() {
  // Opening explorer window to choose a file
  QWidget* current_widget = QApplication::focusWidget();
  if(current_widget == nullptr){
    ROS_ERROR("[WorldManager]: Current widget is nullptr. Could not open eplorer window");
    return;
  }
  QString filename = QFileDialog::getSaveFileName(current_widget, tr("Save File"), "/home", tr("yaml files (*.yaml)"));
  if(filename.isNull()){
    ROS_WARN("[WorldManager]: File has not been selected. No config has been saved");
    return;
  }

  if(properties.size() == 0){
    ROS_WARN("[WorldManager]: No drones have been detected");
    return;
  }

  // Counting number of chosen drones
  int drone_num = 0;
  for(size_t i=0; i<properties.size(); i++){
    if(properties[i]->getBool()){
      drone_num++;
    }
  }

  // Sending requests to save configs
  for(size_t i=0; i<properties.size(); i++){
    // Skipping unchosen drones
    if(!properties[i]->getBool()){
      ROS_INFO("Skipping drone %s", properties[i]->getNameStd().c_str());
      continue;
    }

    // Correcting filenames to avoid name conflicts 
    mrs_msgs::String srv;
    if(drone_num > 1){
      srv.request.value = filename.toStdString();
      size_t pos = srv.request.value.find(".yaml");
      if(pos == std::string::npos){
        srv.request.value = srv.request.value + "_" + properties[i]->getNameStd();
      } else{
        srv.request.value.insert(pos-1, "_" + properties[i]->getNameStd());
      }
    }

    // Sending requests
    if(!save_config_clients[i].call(srv)){
      ROS_WARN("[WorldManager]: Could not call save_world_config service for drone %s", properties[i]->getNameStd().c_str());
      continue;
    }
    if(!srv.response.success){
      ROS_WARN("[WorldManager]: Could not add obstacle for drone %s: %s", properties[i]->getNameStd().c_str(), srv.response.message.c_str());
      continue;
    }
    ROS_INFO("[WorldManager]: Obstacles has been added successfully");
  }
}

} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::WorldManager, rviz::Tool)