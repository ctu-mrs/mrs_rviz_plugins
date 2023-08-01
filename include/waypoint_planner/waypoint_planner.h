#ifndef WAYPOINT_PLANNER_TOOL_H
#define WAYPOINT_PLANNER_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <ros/topic.h>
#include <ros/duration.h>
#include <ros/master.h>

#include <XmlRpcValue.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/render_panel.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Reference.h> 
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>

#include <qevent.h>
#include <qkeysequence.h>

#include <vector>
#include <atomic>
#include <thread>

namespace mrs_rviz_plugins
{
/* class Arrow; */
/* class DisplayContext; */
/* class StringProperty; */

class WaypointPlanner : public rviz::PoseTool {
public:
  WaypointPlanner();
  ~WaypointPlanner() override ;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected:
  void add_property();
  void onPoseSet(double x, double y, double theta) override;

  // Transforms positions from current frame to the fcu frame and saves results to vector.
  // frame_id: frame, that points are set in
  // num: number of required points to transform. -1 will process all available points.
  std::vector<mrs_msgs::Reference> generate_references(std::string frame_id, int num);
  
  void process_loop();
  void send_waypoints();

Q_OBJECT
protected Q_SLOTS:
  void update_topic();
  void update_position();
  void update_shape();

private:
  class Position{
    public:
    float x;
    float y;
    float z;
    float theta;
    Position(float x_, float y_, float z_, float theta_):x(x_), y(y_), z(z_), theta(theta_){}
    void set_values(Ogre::Vector3 pos, double theta_){
      x = pos.x;
      y = pos.y;
      z = pos.z;
      theta = theta_;
    }
  };

  // Added items
  std::vector<Ogre::SceneNode*> pose_nodes;
  std::vector<rviz::VectorProperty*> point_properties;
  std::vector<rviz::FloatProperty*> angle_properties;
  std::vector<rviz::Arrow*> arrows;
  std::vector<rviz::Axes*> axess;

  // Current properties
  rviz::Property* current_property;
  rviz::VectorProperty* current_point_property;
  rviz::FloatProperty* current_theta_property;

  // Initial properties
  rviz::StringProperty* drone_name_property;
  rviz::StringProperty* topic_property;
  rviz::BoolProperty* use_heading_property;
  rviz::BoolProperty* fly_now_property;
  rviz::BoolProperty* stop_at_waypoints_property;
  rviz::BoolProperty* loop_property;
  rviz::FloatProperty* height_offset_property;
  rviz::EnumProperty* shape_property;

  // Communicating through ros
  ros::NodeHandle node_handler;
  ros::ServiceClient client;

  std::vector<WaypointPlanner::Position> positions;
  std::string status;
  mrs_lib::Transformer transformer;
  Ogre::Entity* model;
  std::atomic_bool is_on_loop;
};


}  // namespace mrs_rviz_plugins

#endif