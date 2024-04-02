#ifndef COVERAGE_PATH_PLANNING_METHOD_H
#define COVERAGE_PATH_PLANNING_METHOD_H

#include <mrs_lib/safety_zone/polygon.h>
#include <rviz/properties/property.h>
#include <OGRE/OgreSceneManager.h>
#include <QObject>

namespace mrs_rviz_plugins{

class CoverageMethod : public QObject{

public:
  // CoverageMethod(rviz::Property* property_container, Ogre::SceneManager* scene_manager){
  //   property_container_ = property_container;
  //   scene_manager_ = scene_manager;
  // }

  // All the scene nodes created by this plugin must be children of root_node.
  virtual void initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){
    property_container_ = property_container;
    scene_manager_ = scene_manager;
    root_node_ = root_node;
  }

  virtual void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) = 0;

  virtual void setStart(Ogre::Vector3 position) = 0;

  virtual void compute() = 0;

  // Angle must be in interval (0; 180)
  virtual void setAngle(int angle, bool update=true) = 0;

  // overlap percentage is in interval [0; 1]
  virtual void setOverlap(float percentage, bool update=true) = 0;

  virtual void setHeight(float height, bool update=true) = 0;

  virtual void setFrame(std::string new_frame, bool update=true) = 0;

  virtual void start() = 0;

protected:
  // Visualisation
  rviz::Property* property_container_;
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode*    root_node_;

  // Math
  mrs_lib::Polygon current_polygon_;
  Ogre::Vector3 start_position_;
  std::string polygon_frame_;
  std::string current_frame_; 
  float overlap_ = 0.1;
  float height_ = 5;
  int angle_ = 90;
}; // class CoverageMethod

} // namespace mrs_rviz_plugins

#endif