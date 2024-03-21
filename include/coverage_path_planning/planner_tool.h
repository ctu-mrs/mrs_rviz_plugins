#ifndef COVERAGE_PATH_PLANNING_TOOL_H
#define COVERAGE_PATH_PLANNING_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <rviz/tool.h>
#include <rviz/properties/enum_property.h>

#include <coverage_path_planning/coverage_method.h>

namespace mrs_rviz_plugins
{
class PlannerTool : public rviz::Tool {
Q_OBJECT 
public:
  PlannerTool();
  void onInitialize() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  void activate() override;
  void deactivate() override;

protected Q_SLOTS:
  void methodChosen();

private:
  rviz::EnumProperty* method_property;
  boost::shared_ptr<mrs_rviz_plugins::CoverageMethod> current_coverage_method;

}; // class PlannerTool
} // namespace mrs_rviz_plugins

#endif