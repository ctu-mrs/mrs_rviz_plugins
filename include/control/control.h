#ifndef CONTROL_TOOL_H
#define CONTROL_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <QString>
#include <QAction>
#include <QMenu>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/render_panel.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/default_plugin/tools/selection_tool.h>

#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_manager.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>


#include "control/im_server.h"

namespace mrs_rviz_plugins{

class ControlTool : public rviz::SelectionTool{

  Q_OBJECT
public:
  ControlTool();
  ~ControlTool() override;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int  processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int  processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected Q_SLOTS:

protected:
  // Check if the mouse has moved from one object to another,
  // and update focused_object_ if so. 
  void updateFocus(const rviz::ViewportMouseEvent& event);

  std::vector<std::string> findSelectedDroneNames();

  uint64_t last_selection_frame_count_;

  // The object (control) which currently has the mouse focus.
  rviz::InteractiveObjectWPtr focused_object_;
  ImServer* server;

  const int KEY_M = 77;
  const int KEY_W = 87;       const int KEY_H = 72;
  const int KEY_A = 65;       const int KEY_J = 74;
  const int KEY_S = 83;       const int KEY_K = 75;
  const int KEY_D = 68;       const int KEY_L = 76;
  const int KEY_R = 82;       const int KEY_F = 70;
  const int KEY_Q = 81;
  const int KEY_E = 69;

  bool remote_mode_on = false;

}; // class ControlTool


} // namespace mrs_rviz_plugins

#endif