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

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include <mrs_msgs/Path.h>

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

  static std::vector<std::string> getUavNames();

protected Q_SLOTS:
  void methodChosen();
  void droneChanged();
  void heightChanged();
  void angleChanged();
  void overlapChanged();
  void startChanged();

  void loadPath();
  void savePath();
  void updatePolygon();
  void computePath();
  void startMission();

private:
  // Function for convenience
  void makeFlag(const Ogre::Vector3& position);

  // Properties
  rviz::IntProperty*          angle_property_;
  rviz::EnumProperty*         method_property;
  rviz::FloatProperty*        overlap_property_;
  rviz::FloatProperty*        height_property;
  rviz::VectorProperty*       start_property_;
  rviz::EditableEnumProperty* drone_name_property_;

  // ROS communication
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  
  // Other attributes
  std::string                                         flag_path_;
  Ogre::SceneNode*                                    flag_node_;
  Ogre::SceneNode*                                    root_node_;
  mrs_lib::Transformer                                transformer_;
  pluginlib::ClassLoader<CoverageMethod>              method_loader_;
  boost::shared_ptr<mrs_rviz_plugins::CoverageMethod> current_coverage_method_;


}; // class PlannerTool
} // namespace mrs_rviz_plugins

#endif