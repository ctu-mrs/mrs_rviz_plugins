#include "coverage_path_planning/planner_tool.h"
#include "coverage_path_planning/coverage_method.h"

#include <rviz/visualization_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/render_panel.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>

#include <mrs_msgs/GetSafeZoneAtHeight.h>

#include <OGRE/OgreEntity.h>

// TODO: include polygon only
// #include <mrs_lib/safety_zone/polygon.h>
#include <mrs_lib/safety_zone/prism.h>

#include <boost/geometry.hpp>

#include <QMenu>

namespace bg = boost::geometry;

namespace mrs_rviz_plugins
{
PlannerTool::PlannerTool() : method_loader("mrs_rviz_plugins", "mrs_rviz_plugins::CoverageMethod"){
  ROS_INFO("Constructor called");
  shortcut_key_ = 'p';
  flag_node_ = nullptr;

  drone_name_property = new rviz::EditableEnumProperty("Main Safety area manager", "uav1", "Safety area of this drone will be used to plan the coverage path.", 
                        getPropertyContainer(), SLOT(droneChanged()), this);
  method_property = new rviz::EnumProperty("Used method", "None", "Choose the algorithm to plan the coverage path", getPropertyContainer(), 
                        SLOT(methodChosen()), this);
  height_property = new rviz::FloatProperty("Height", 3.0F, "The height of the flight", getPropertyContainer(), 
                        SLOT(heightChanged()), this);
  angle_property_ = new rviz::IntProperty("Angle", 90, "Camera's viewing angle", getPropertyContainer(), SLOT(angleChanged()), this);
  overlap_property_ = new rviz::FloatProperty("Overlap", 0.1, "Overlap percentage of adjacent pictures", getPropertyContainer(), SLOT(overlapChanged()), this);
  start_property_   = new rviz::VectorProperty("Start", Ogre::Vector3(), "Start point for the mission", getPropertyContainer(), SLOT(startChanged()), this);

  for(auto& name : method_loader.getDeclaredClasses()){
    method_property->addOptionStd(name);
  }

  angle_property_->setMin(1);
  angle_property_->setMax(179);

  overlap_property_->setMax(1);
  overlap_property_->setMin(0);

  ROS_INFO("Planner Tool constructed");
}

void PlannerTool::onInitialize(){
  setName("Coverage path");
  root_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Prepare mash resource
  flag_path_ = "package://rviz_plugin_tutorials/media/flag.dae";
  if(rviz::loadMeshFromResource(flag_path_).isNull()){
    ROS_ERROR( "PlantFlagTool: failed to load model resource '%s'.", flag_path_.c_str() );
    return;
  }

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
    ROS_INFO("[Coverage Path Planning]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    ROS_INFO("[Coverage Path Planning]: %s was added to drone names", name.c_str());
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
              "/safety_area_manager/get_safety_zone_at_height");
}

void PlannerTool::makeFlag( const Ogre::Vector3& position ){
  if(!flag_node_){
    flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }
  Ogre::Entity* entity = scene_manager_->createEntity(flag_path_);
  if(!entity){
    ROS_ERROR("[Coverage Path Planning]: Could not create entity of flag!");
  }else{
    flag_node_->attachObject( entity );
  }
  flag_node_->setVisible( true );
  flag_node_->setPosition( position );
}

int PlannerTool::processMouseEvent(rviz::ViewportMouseEvent& event){
  int res = rviz::MoveTool::processMouseEvent(event);

  // Infotmation messages
  if(event.shift()){
    setStatus("<b>Left-Click:</b> Move X/Y. <b>Right-Click:</b> Move Z. <b>Mouse Wheel</b> Zoom.");
    return res;
  } else if(event.alt()){
    setStatus("<b>Left-Click:</b> Set start point.");
  } else{
    setStatus("<b>Left-Click:</b> Rotate. <b>Middle-Click:</b> Move X/Y. <b>Mouse Wheel</b> Zoom. <b>Alt:</b> Set start point. <b>Shift:</b> More options.");
  }

  // Setting start point
  if(event.alt() && event.leftUp()){
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, height_property->getFloat());

    if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection)) {
      ROS_ERROR("[Coverage Path Planning]: Intersection was not found");
      return res;
    }

    makeFlag(intersection);
    start_property_->setVector(intersection);
  }

  if (event.panel->contextMenuVisible()){
    return res;
  }

  if(!event.rightUp()){
    return res;
  }

  // Showing menu
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

  return res | Render;
}

void PlannerTool::activate() {
  root_node->setVisible(true);
  ROS_INFO("[PlannerTool]: Activated");
}

void PlannerTool::deactivate() {
  root_node->setVisible(false);
  flag_node_->setVisible(false);
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
  current_coverage_method->setHeight(height_property->getFloat(), false);
  current_coverage_method->setAngle(angle_property_->getInt(), false);
  current_coverage_method->setOverlap(overlap_property_->getFloat(), false);
  updatePolygon();
}

void PlannerTool::updatePolygon(){
  if(!current_coverage_method){
    ROS_WARN("[Coverage Path Planning]: Path planning method has not been selected");
    setStatus("Select a coverage path planning method.");
    return;
  }

  mrs_msgs::GetSafeZoneAtHeight srv;
  srv.request.height = height_property->getFloat();

  if(!client_.call(srv)){
    ROS_WARN("[Coverage Path Planning]: could not call service %s", client_.getService().c_str());
    setStatus("Could not call Safety Area Manager of " + drone_name_property->getString());
    return;
  }

  // Add the border
  mrs_lib::Polygon result = mrs_lib::Polygon();
  for(geometry_msgs::Point32 point : srv.response.safety_zone[0].points){
    mrs_lib::Point2d p{point.x, point.y};
    bg::append(result, p);
  }
  
  // Add obstacles
  if(srv.response.safety_zone.size() > 1){
    bg::interior_rings(result).resize(srv.response.safety_zone.size() - 1);
    
    for(size_t i=1; i < srv.response.safety_zone.size(); i++){
      for(geometry_msgs::Point32 point : srv.response.safety_zone[i].points){
        mrs_lib::Point2d p{point.x, point.y};
        bg::append(result, p, i-1);
      }
    }
  }

  // Correcting the orientation of the polygon (vertices must be conter-clockwise oredered)
  bg::correct(result);
  
  std::string msg;
  if(!bg::is_valid(result, msg)){
    ROS_WARN("[Coverage Path Planning]: The received polygon could not be corrected: %s", msg.c_str());
    setStatus(msg.c_str()); 
    return;
  }

  current_coverage_method->setPolygon(result);
}

void PlannerTool::computePath(){
  ROS_INFO("compute called");
}

void PlannerTool::startMission(){
  ROS_INFO("start called");
}

void PlannerTool::droneChanged(){
  client_ = nh_.serviceClient<mrs_msgs::GetSafeZoneAtHeight>("/" + drone_name_property->getStdString() + 
              "/safety_area_manager/get_safety_zone_at_height");
}

void PlannerTool::angleChanged(){
  if(!current_coverage_method){
    ROS_WARN("[Coverage Path Planning]: Path planning method has not been selected");
    setStatus("Select a coverage path planning method.");
    return;
  }
  current_coverage_method->setAngle(angle_property_->getInt());
}

void PlannerTool::overlapChanged(){
  if(!current_coverage_method){
    ROS_WARN("[Coverage Path Planning]: Path planning method has not been selected");
    setStatus("Select a coverage path planning method.");
    return;
  }
  current_coverage_method->setOverlap(overlap_property_->getFloat());
}

void PlannerTool::heightChanged(){
  if(!current_coverage_method){
    ROS_WARN("[Coverage Path Planning]: Path planning method has not been selected");
    setStatus("Select a coverage path planning method.");
    return;
  }
  current_coverage_method->setHeight(height_property->getFloat());
}

void PlannerTool::startChanged(){
  flag_node_->setPosition(start_property_->getVector());
  if(current_coverage_method){
    current_coverage_method->setStart(start_property_->getVector());
  }
}

}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::PlannerTool, rviz::Tool)