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
enum PoseType
{
  ARROW,
  AXES,
  INVISIBLE
};

class VisualEntity {
public:
  VisualEntity(Ogre::SceneManager* scene_manager_);

  ~VisualEntity();

  void set(const Ogre::Vector3& point_, const Ogre::Vector3& velocity_, const Ogre::Quaternion& orientation_, const float vel_abs_);

  // Position
  void setPoseArrowColor(const QColor& qcolor);
  void setPoseArrowParams(const float shaft_length, const float shaft_diameter, const float head_length, const float head_diameter);
  void setPoseType(const PoseType& type);
  void setAxesParams(const float len, const float rad);

  // Velocity
  void setVelArrowColor(const QColor& color);
  void setVelArrowParams(const float shaft_length, const float shaft_diameter, const float head_length, const float head_diameter, const float scale);
  void setVelVisible(const bool value);

  // Covariance
  void setHasCov(const bool value);
  bool getHasCov();

  Ogre::Vector3 getPoint();

protected:
  // Data:
  float               vel_abs = 1;  // Default value so it's not NaN
  float               scale   = 1;  // Default value so it's not NaN
  bool                has_cov;      // Whether a CovarianceVisual is associated with this entity
  PoseType            pose_type;
  Ogre::Vector3       point;
  Ogre::Quaternion    orientation;
  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode*    pose_arrow_scene_node;
  Ogre::SceneNode*    vel_arrow_scene_node;
  Ogre::SceneNode*    axes_scene_node;

  // Visual:
  rviz::Arrow* pose_arrow;
  rviz::Arrow* vel_arrow;
  rviz::Axes*  pose_axes;
};
}  // namespace mrs_rviz_plugins


#endif
