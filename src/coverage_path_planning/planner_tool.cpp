#include "coverage_path_planning/planner_tool.h"
#include "coverage_path_planning/coverage_method.h"

#include <pluginlib/class_loader.h>

namespace mrs_rviz_plugins
{
PlannerTool::PlannerTool(){
  shortcut_key_ = 'p';

  method_property = new rviz::EnumProperty("Used method", "None", "Choose the algorithm to plan the coverage path", getPropertyContainer(), 
                        SLOT(methodChosen()), this);

  pluginlib::ClassLoader<CoverageMethod> method_loader("mrs_rviz_plugins", "mrs_rviz_plugins::CoverageMethod");

  std::cout << "available classes: "<< method_loader.getDeclaredClasses().size() << std::endl;
  std::string name = method_loader.getDeclaredClasses()[0];
  current_coverage_method = method_loader.createInstance(name);
}

void PlannerTool::onInitialize(){
  setName("Coverage path");
}

int PlannerTool::processMouseEvent(rviz::ViewportMouseEvent& event){
  ROS_INFO("[PlannerTool]: Mouse event received");
  if(current_coverage_method){
    current_coverage_method->start();
  }
  return Render;
}

void PlannerTool::activate() {
  ROS_INFO("[PlannerTool]: Activated");
}

void PlannerTool::deactivate() {
  ROS_INFO("[PlannerTool]: Deactivated");
  
}

}// namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::PlannerTool, rviz::Tool)