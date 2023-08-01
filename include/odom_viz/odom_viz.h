
#ifndef ODOM_VIZ_TOOL_H
#define ODOM_VIZ_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
    #include <QObject>
    #include <ros/ros.h>
#endif

#include <nav_msgs/Odometry.h>

#include <OGRE/OgreVector3.h>

#include <QColor>

#include <rviz/message_filter_display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/default_plugin/covariance_property.h>
#include <rviz/default_plugin/covariance_visual.h>

#include "odom_viz/visual_entity.h"
#include <vector>

namespace mrs_rviz_plugins
{
class OdomViz : public rviz::MessageFilterDisplay<nav_msgs::Odometry>
{
Q_OBJECT;
public:

    OdomViz();
    void onInitialize();

    // A helper to clear this display back to the initial state.
    void reset() override;

protected Q_SLOTS:
    // General
    void on_keep_changed();

    // Position
    void on_position_changed();
    void on_shape_changed();
    void on_pose_color_changed();
    void on_pose_params_changed();
    void on_axes_params_changed();

    // Velocity
    void on_velocity_changed();
    void on_vel_color_changed();
    void on_vel_params_changed();

protected:
    void onDisable() override;
    void onEnable() override;

private:
    void processMessage(const nav_msgs::Odometry::ConstPtr& msg);
    // Properties:
    //      General
    rviz::RosTopicProperty* topic_property;
    rviz::IntProperty* keep_property;
    rviz::FloatProperty* pose_tolerance_property;
    rviz::FloatProperty* angle_tolerance_property;

    //      Position
    rviz::BoolProperty* pose_property;
    rviz::EnumProperty* shape_property;
    //          Arrow
    rviz::ColorProperty* pose_color_property;
    rviz::IntProperty*   pose_opacity_property;
    rviz::FloatProperty* pose_shaft_len_property;
    rviz::FloatProperty* pose_shaft_rad_property;
    rviz::FloatProperty* pose_head_len_property;
    rviz::FloatProperty* pose_head_rad_property;
    //          Axes
    rviz::FloatProperty* pose_axes_len_property;
    rviz::FloatProperty* pose_axes_rad_property;

    //      Velocity
    rviz::BoolProperty*  vel_property;
    rviz::ColorProperty* vel_color_property;
    rviz::IntProperty*   vel_opacity_property;
    rviz::FloatProperty* vel_shaft_len_property;
    rviz::FloatProperty* vel_shaft_rad_property;
    rviz::FloatProperty* vel_head_len_property;
    rviz::FloatProperty* vel_head_rad_property;
    rviz::FloatProperty* vel_scale_property;

    //      Covariance
    rviz::CovarianceProperty* covariance_property;

    std::vector<VisualEntity*> entities;
    nav_msgs::Odometry::ConstPtr last_msg;
};
}

#endif