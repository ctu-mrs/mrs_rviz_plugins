#ifndef CONTROL_TOOL_H
#define CONTROL_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <QString>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/render_panel.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/default_plugin/tools/selection_tool.h>

#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_manager.h>

#include "control/im_server.h"

namespace mrs_rviz_plugins{

class ControlTool : public rviz::SelectionTool{

public:
  ControlTool();
  ~ControlTool() override;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int  processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int  processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected:
  // Check if the mouse has moved from one object to another,
  // and update focused_object_ if so. 
  void updateFocus(const rviz::ViewportMouseEvent& event);

  uint64_t last_selection_frame_count_;

  // The object (control) which currently has the mouse focus.
  rviz::InteractiveObjectWPtr focused_object_;
  ImServer* server;

}; // class ControlTool


} // namespace mrs_rviz_plugins

#endif