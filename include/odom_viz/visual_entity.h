#ifndef VISUAL_ENTITY_H
#define VUSUAL_ENTITY_H

#include <nav_msgs/Odometry.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreColourValue.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>

#include <QColor>

namespace mrs_rviz_plugins
{
enum PoseType {arrow, axes, invisible};

class VisualEntity{
public:
    VisualEntity(Ogre::SceneManager* scene_manager_);

    ~VisualEntity();

    void set(Ogre::Vector3 point_, Ogre::Vector3 velocity_, 
                Ogre::Quaternion orientation_, float vel_abs_);

    // Position
    void set_pose_arrow_color(QColor color);
    void set_pose_arrow_params(float shaft_length, float shaft_diameter, 
                                float head_length, float head_diameter);
    void set_pose_type(PoseType type);
    void set_axes_params(float len, float rad);

    // Velocity
    void set_vel_arrow_color(QColor color);
    void set_vel_arrow_params(float shaft_length, float shaft_diameter, 
                                float head_length, float head_diameter, 
                                float scale);
    void set_vel_visible(bool value);


    Ogre::Vector3 get_point();
protected:
    // Data:
    PoseType pose_type;
    Ogre::Vector3 point;
    float vel_abs = 1;              // Default value so it's not NaN
    float scale = 1;                // Default value so it's not NaN
    Ogre::Quaternion orientation;
    Ogre::SceneManager* scene_manager;
    Ogre::SceneNode* pose_arrow_scene_node;
    Ogre::SceneNode* vel_arrow_scene_node;
    Ogre::SceneNode* axes_scene_node;

    // Visual:
    rviz::Arrow* pose_arrow;
    rviz::Arrow* vel_arrow;
    rviz::Axes* pose_axes;
};
}// namespace mrs_rviz_plugins



#endif