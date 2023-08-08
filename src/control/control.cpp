#include "control/control.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <rviz/properties/status_list.h>

#include <rviz/default_plugin/interactive_markers/interactive_marker.h>

namespace mrs_rviz_plugins{

  ControlTool::ControlTool(){
    shortcut_key_ = 'c';
    ROS_INFO("Status list creating ...");
    status_list   = new rviz::StatusList(QString("Status"), getPropertyContainer());

  }

  void ControlTool::activate(){
    // Regular marker
    visualization_msgs::Marker marker_msg;
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.scale.x = 0.45;
    marker_msg.scale.y = 0.45;
    marker_msg.scale.z = 0.45;
    marker_msg.color.r = 0.5;
    marker_msg.color.g = 0.5;
    marker_msg.color.b = 0.5;
    marker_msg.color.a = 1.0;
    marker_msg.id = 1;
    marker_msg.pose.position.x = 0;
    marker_msg.pose.position.y = 0;
    marker_msg.pose.position.z = 0;

    // Control
    visualization_msgs::InteractiveMarkerControl control_msg;
    control_msg.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control_msg.always_visible = true;
    control_msg.always_visible = true;
    control_msg.independent_marker_orientation = true;
    control_msg.name = "control1";
    // control_msg.orientation = IDENTITY by default
    control_msg.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    control_msg.markers.push_back(marker_msg);

    // Interactive marker
    visualization_msgs::InteractiveMarker int_marker_msg;
    int_marker_msg.header.frame_id = "base_link";
    int_marker_msg.pose.position.y = 0;
    int_marker_msg.scale = 1;
    int_marker_msg.name = "marker1";
    int_marker_msg.controls.push_back(control_msg);

    // Scene node
    Ogre::SceneNode* node = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    node->setPosition(0, 0, 0);

    ROS_INFO("InteractiveMarker");
    rviz::InteractiveMarker* im = new rviz::InteractiveMarker(node, context_);

    ROS_INFO("Control");
    rviz::InteractiveMarkerControl* im_control = new rviz::InteractiveMarkerControl(context_, node, im);

    ROS_INFO("InteractiveMarker process msg");
    im->processMessage(int_marker_msg);
  }

  void ControlTool::deactivate(){

  }

  int ControlTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
    // ROS_INFO("[Control Tool]: Received key %d", (int)event->key());
    // client->onFixedFrameChanged();
    // client->update(1, 2);
    // if(num == 0){
    //     // putInteractiveMarker();
    //     num++;
    // }
    return Render;
  }

}// namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::ControlTool, rviz::Tool)