#include "coverage_path_planning/planner_tool.h"
#include "coverage_path_planning/coverage_method.h"


#include <rviz/viewport_mouse_event.h>

namespace mrs_rviz_plugins
{
PlannerTool::PlannerTool() : method_loader("mrs_rviz_plugins", "mrs_rviz_plugins::CoverageMethod"){
  ROS_INFO("Constructor called");
  shortcut_key_ = 'p';

  method_property = new rviz::EnumProperty("Used method", "None", "Choose the algorithm to plan the coverage path", getPropertyContainer(), 
                        SLOT(methodChosen()), this);

  for(auto& name : method_loader.getDeclaredClasses()){
    method_property->addOptionStd(name);
  }

  ROS_INFO("Planner Tool constructed");
}

void PlannerTool::onInitialize(){
  setName("Coverage path");
  root_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

  ROS_INFO("Planner tool inited");
}

int PlannerTool::processMouseEvent(rviz::ViewportMouseEvent& event){

  if(current_coverage_method && event.leftDown()){
    ROS_INFO("[PlannerTool]: leftDown() received");
    current_coverage_method->start();
  }
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

}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::PlannerTool, rviz::Tool)