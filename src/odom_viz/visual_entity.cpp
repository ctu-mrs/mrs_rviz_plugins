#include "odom_viz/visual_entity.h"

#include <ros/console.h>

namespace mrs_rviz_plugins{
VisualEntity::VisualEntity(Ogre::SceneManager* scene_manager_){
    scene_manager = scene_manager_;
    
    pose_arrow_scene_node = scene_manager->getRootSceneNode()->createChildSceneNode();
    vel_arrow_scene_node = scene_manager->getRootSceneNode()->createChildSceneNode();
    axes_scene_node = scene_manager->getRootSceneNode()->createChildSceneNode();

    pose_arrow = new rviz::Arrow(scene_manager, pose_arrow_scene_node);
    pose_arrow->setDirection(Ogre::Vector3(1, 0, 0));
    vel_arrow = new rviz::Arrow(scene_manager, vel_arrow_scene_node);
    pose_axes = new rviz::Axes(scene_manager, axes_scene_node);

    pose_type = PoseType::arrow;
}

VisualEntity::~VisualEntity(){
   scene_manager->destroySceneNode(pose_arrow_scene_node);
   scene_manager->destroySceneNode(axes_scene_node);
   scene_manager->destroySceneNode(vel_arrow_scene_node);
}

void VisualEntity::set(Ogre::Vector3 point_, Ogre::Vector3 velocity_, 
                       Ogre::Quaternion orientation_, float vel_abs_){
    point = point_;
    orientation = orientation_;
    vel_abs = vel_abs_;

    pose_arrow_scene_node->setPosition(point);
    pose_arrow_scene_node->setOrientation(orientation);
    axes_scene_node->setPosition(point);
    axes_scene_node->setOrientation(orientation);

    Ogre::Vector3 velocity = velocity_;
    vel_arrow_scene_node->setPosition(point);
    vel_arrow_scene_node->setDirection(velocity);
    vel_arrow->setScale(Ogre::Vector3(vel_abs*scale, vel_abs*scale, vel_abs*scale));
}

void VisualEntity::set_pose_type(PoseType type){
    pose_type = type;
    if(pose_type == PoseType::arrow){
        axes_scene_node->setVisible(false);
        pose_arrow_scene_node->setVisible(true);
    }
    if(pose_type == PoseType::axes){
        axes_scene_node->setVisible(true);
        pose_arrow_scene_node->setVisible(false);
    }
    if(pose_type == PoseType::invisible){
        axes_scene_node->setVisible(false);
        pose_arrow_scene_node->setVisible(false);
    }
}

Ogre::Vector3 VisualEntity::get_point(){
    return point;
}

void VisualEntity::set_pose_arrow_color(QColor qcolor){
    int r = qcolor.red();
    int g = qcolor.green();
    int b = qcolor.blue();
    int a = qcolor.alpha();

    Ogre::ColourValue color;
    color.setAsRGBA((r << 24) | (g << 16) | (b << 8) | a);
    pose_arrow->setColor(color);
}

void VisualEntity::set_pose_arrow_params(float shaft_length, float shaft_diameter, 
                                         float head_length, float head_diameter){
    pose_arrow->set(shaft_length, shaft_diameter, head_length, head_diameter);
}

void VisualEntity::set_axes_params(float len, float rad){
    pose_axes->set(len, rad);
}

void VisualEntity::set_vel_arrow_color(QColor qcolor){
    int r = qcolor.red();
    int g = qcolor.green();
    int b = qcolor.blue();
    int a = qcolor.alpha();

    Ogre::ColourValue color;
    color.setAsRGBA((r << 24) | (g << 16) | (b << 8) | a);
    vel_arrow->setColor(color);
}

void VisualEntity::set_vel_arrow_params(float shaft_length, float shaft_diameter, 
                                float head_length, float head_diameter, float scale_){
    scale = scale_;
    vel_arrow->set(shaft_length, shaft_diameter, head_length, head_diameter);
    vel_arrow->setScale(Ogre::Vector3(scale*vel_abs, scale*vel_abs, scale*vel_abs));
}

void VisualEntity::set_vel_visible(bool value){
    vel_arrow_scene_node->setVisible(value);
}


} // namespace mrs_rviz_plugins
