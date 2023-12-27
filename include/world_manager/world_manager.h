#ifndef WORLD_MANAGER_TOOL_H
#define WORLD_MANAGER_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <rviz/default_plugin/tools/interaction_tool.h>
#include <rviz/properties/bool_property.h>
#include <rviz/visualization_manager.h>

#include <mrs_msgs/ReferenceStampedSrv.h>

#include <vector>

namespace mrs_rviz_plugins
{
class WorldManager : public rviz::InteractionTool{
Q_OBJECT

public:
  WorldManager();
  void onInitialize() override;
  // void activate() override;
  // void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

protected Q_SLOTS:
  void add_obstacle();

private:
  std::vector<rviz::BoolProperty*> properties;
  ros::NodeHandle                  node_handler;
  std::vector<ros::ServiceClient>  clients;

  Ogre::Vector3 current_intersection;

}; // class WorldManager

} // namespace mrs_rviz_plugins

#endif