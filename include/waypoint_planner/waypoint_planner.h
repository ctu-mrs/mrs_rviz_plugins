#ifndef WAYPOINT_PLANNER_TOOL_H
#define WAYPOINT_PLANNER_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#endif

#include <OGRE/OgreSceneNode.h>

#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>

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
  void activate() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

// private Q_SLOTS:
//   void updateTopic();
//   void updateName();

private:
  class Position{
    public:
    float x;
    float y;
    float z;
    float theta;
    Position(float x_, float y_, float z_, float theta_):x(x_), y(y_), z(z_), theta(theta_){}
  };
  std::string flag_resource_;
  std::vector<Ogre::SceneNode*> pose_nodes;
  rviz::Property* current_property;
  rviz::VectorProperty* current_point_property;
  rviz::FloatProperty* current_theta_property;

  std::vector<WaypointPlanner::Position> positions;

};


}  // namespace mrs_rviz_plugins

#endif