#ifndef CONTROL_TOOL_H
#define CONTROL_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#endif

#include <QString>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/tool.h>

#include "control/client_wrapper.h"

namespace mrs_rviz_plugins{

// Note: if the tool will be selection tool eventually, it's better to inherit from SelectionTool
class ControlTool : public rviz::Tool{

public:
    ControlTool();
    // ~ControlTool() override;
    // void onInitialize() override;
    void activate() override;
    void deactivate() override;
    // int  processMouseEvent(rviz::ViewportMouseEvent& event) override;
    int  processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected:

    int num = 0;

    ClientWrapper* client;
    rviz::StatusList* status_list;


}; // class ControlTool


} // namespace mrs_rviz_plugins

#endif