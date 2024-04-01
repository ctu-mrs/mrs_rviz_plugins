#ifndef COVERAGE_PATH_PLANNING_TOOL_H
#define COVERAGE_PATH_PLANNING_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <rviz/default_plugin/tools/move_tool.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>

#include <coverage_path_planning/coverage_method.h>

#include <pluginlib/class_loader.h>

namespace mrs_rviz_plugins
{
class PlannerTool : public rviz::MoveTool {
Q_OBJECT 
public:
  PlannerTool();
  void onInitialize() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  void activate() override;
  void deactivate() override;

protected Q_SLOTS:
  void methodChosen();
  void droneChanged();
  void heightChanged();
  void angleChanged();
  void overlapChanged();
  void startChanged();

  void updatePolygon();
  void computePath();
  void startMission();

private:
// Properties
  rviz::IntProperty*                                  angle_property_;
  rviz::EnumProperty*                                 method_property;
  rviz::FloatProperty*                                overlap_property_;
  rviz::FloatProperty*                                height_property;
  rviz::VectorProperty*                               start_property_;
  rviz::EditableEnumProperty*                         drone_name_property;
  
  std::string                                         flag_path_;
  Ogre::SceneNode*                                    flag_node_;
  Ogre::SceneNode*                                    root_node;
  pluginlib::ClassLoader<CoverageMethod>              method_loader;
  boost::shared_ptr<mrs_rviz_plugins::CoverageMethod> current_coverage_method;

  // ROS communication
  ros::NodeHandle nh_;
  ros::ServiceClient client_;

  void makeFlag(const Ogre::Vector3& position);
}; // class PlannerTool
} // namespace mrs_rviz_plugins

#endif