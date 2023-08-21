#include "odom_viz/visual_entity.h"

#include <ros/console.h>

namespace mrs_rviz_plugins
{
VisualEntity::VisualEntity(Ogre::SceneManager *scene_manager_) {
  scene_manager = scene_manager_;

  pose_arrow_scene_node = scene_manager->getRootSceneNode()->createChildSceneNode();
  vel_arrow_scene_node  = scene_manager->getRootSceneNode()->createChildSceneNode();
  axes_scene_node       = scene_manager->getRootSceneNode()->createChildSceneNode();

  vel_arrow  = new rviz::Arrow(scene_manager, vel_arrow_scene_node);
  pose_axes  = new rviz::Axes(scene_manager, axes_scene_node);
  pose_arrow = new rviz::Arrow(scene_manager, pose_arrow_scene_node);
  pose_arrow->setDirection(Ogre::Vector3(1, 0, 0));

  pose_type = PoseType::ARROW;
}

VisualEntity::~VisualEntity() {
  scene_manager->destroySceneNode(pose_arrow_scene_node);
  scene_manager->destroySceneNode(axes_scene_node);
  scene_manager->destroySceneNode(vel_arrow_scene_node);
}

void VisualEntity::set(const Ogre::Vector3 &point_, const Ogre::Vector3 &velocity_, const Ogre::Quaternion &orientation_, const float vel_abs_) {
  point       = point_;
  orientation = orientation_;
  vel_abs     = vel_abs_;

  pose_arrow_scene_node->setPosition(point);
  pose_arrow_scene_node->setOrientation(orientation);
  axes_scene_node->setPosition(point);
  axes_scene_node->setOrientation(orientation);

  vel_arrow_scene_node->setPosition(point);
  vel_arrow_scene_node->setDirection(velocity_);
  vel_arrow->setScale(Ogre::Vector3(vel_abs * scale, vel_abs * scale, vel_abs * scale));
}

void VisualEntity::setPoseType(const PoseType &type) {
  pose_type = type;
  if (pose_type == PoseType::ARROW) {

    axes_scene_node->setVisible(false);
    pose_arrow_scene_node->setVisible(true);

  } else if (pose_type == PoseType::AXES) {

    axes_scene_node->setVisible(true);
    pose_arrow_scene_node->setVisible(false);

  } else if (pose_type == PoseType::INVISIBLE) {

    axes_scene_node->setVisible(false);
    pose_arrow_scene_node->setVisible(false);
  }
}

Ogre::Vector3 VisualEntity::getPoint() {
  return point;
}

void VisualEntity::setPoseArrowColor(const QColor &qcolor) {
  const int r = qcolor.red();
  const int g = qcolor.green();
  const int b = qcolor.blue();
  const int a = qcolor.alpha();

  Ogre::ColourValue color;
  color.setAsRGBA((r << 24) | (g << 16) | (b << 8) | a);
  pose_arrow->setColor(color);
}

void VisualEntity::setPoseArrowParams(float shaft_length, float shaft_diameter, float head_length, float head_diameter) {
  pose_arrow->set(shaft_length, shaft_diameter, head_length, head_diameter);
}

void VisualEntity::setAxesParams(float len, float rad) {
  pose_axes->set(len, rad);
}

void VisualEntity::setVelArrowColor(const QColor &qcolor) {
  const int r = qcolor.red();
  const int g = qcolor.green();
  const int b = qcolor.blue();
  const int a = qcolor.alpha();

  Ogre::ColourValue color;
  color.setAsRGBA((r << 24) | (g << 16) | (b << 8) | a);
  vel_arrow->setColor(color);
}

void VisualEntity::setVelArrowParams(float shaft_length, float shaft_diameter, float head_length, float head_diameter, float scale_) {
  scale = scale_;
  vel_arrow->set(shaft_length, shaft_diameter, head_length, head_diameter);
  vel_arrow->setScale(Ogre::Vector3(scale * vel_abs, scale * vel_abs, scale * vel_abs));
}

void VisualEntity::setVelVisible(const bool value) {
  vel_arrow_scene_node->setVisible(value);
}

void VisualEntity::setHasCov(const bool value) {
  has_cov = value;
}

bool VisualEntity::getHasCov() {
  return has_cov;
}

}  // namespace mrs_rviz_plugins
