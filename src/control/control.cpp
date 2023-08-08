#include "control/control.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <rviz/properties/status_list.h>

namespace mrs_rviz_plugins{

  ControlTool::ControlTool(){
    shortcut_key_ = 'c';
    ROS_INFO("Status list creating ...");
    status_list   = new rviz::StatusList(QString("Status"), getPropertyContainer());

  }

  void ControlTool::activate(){
    ROS_INFO("Client wrapper creating ...");
    client = new ClientWrapper(context_, status_list);

    context_->getFixedFrame();


    client->onInitialize();

    client->enable();

    ROS_INFO("Setting topic ...");
    client->setTopic(std::string("/basic_controls/update"));

    ROS_INFO("Constructor called");
  }

  void ControlTool::deactivate(){

  }

  int ControlTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
    ROS_INFO("[Control Tool]: Received key %d", (int)event->key());
    client->onFixedFrameChanged();
    client->update(1, 2);
    if(num == 0){
        // putInteractiveMarker();
        num++;
    }
    return Render;
  }

}// namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::ControlTool, rviz::Tool)