#ifndef WAYPOINT_PLANNER_TOOL_H
#define WAYPOINT_PLANNER_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#endif

#include <OGRE/OgreSceneNode.h>

#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/properties/string_property.h>

#include <vector>

namespace mrs_rviz_plugins
{
/* class Arrow; */
/* class DisplayContext; */
/* class StringProperty; */

class WaypointPlanner : public rviz::PoseTool {
  Q_OBJECT
public:
  WaypointPlanner();
  ~WaypointPlanner() override ;
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

// private Q_SLOTS:
//   void updateTopic();
//   void updateName();

private:
  std::string flag_resource_;
  std::vector<Ogre::SceneNode*> pose_nodes;
};


}  // namespace mrs_rviz_plugins

#endif