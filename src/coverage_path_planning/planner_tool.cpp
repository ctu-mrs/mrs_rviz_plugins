#include "coverage_path_planning/planner_tool.h"

namespace mrs_rviz_plugins
{
PlannerTool::PlannerTool(){
  shortcut_key_ = 'p';

  method_property = new rviz::EnumProperty("Used method", "None", "Choose the algorithm to plan the coverage path", getPropertyContainer(), 
                        SLOT(methodChosen()), this);


}

void PlannerTool::onInitialize(){
  setName("Coverage path");
}

int PlannerTool::processMouseEvent(rviz::ViewportMouseEvent& event){
  ROS_INFO("[PlannerTool]: Mouse event received");
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