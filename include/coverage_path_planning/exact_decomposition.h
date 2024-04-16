#ifndef EXACT_DECOMPOSITION_METHOD_H
#define EXACT_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/coverage_method.h"

#include <utility>

#include <mrs_msgs/PathSrv.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

namespace mrs_rviz_plugins{

class ExactDecomposition : public CoverageMethod {
Q_OBJECT
public: 
  void initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void setStart(Ogre::Vector3 position) override;

  void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) override;

  void setAngle(int angle, bool update=true) override;

  void setOverlap(float percentage, bool update=true) override;

  void setHeight(float height, bool update=true) override;

  void setFrame(std::string new_frame, bool update=true) override;

  ~ExactDecomposition();

protected Q_SLOTS:
  void boundariesChanged();
  void decompositionChanged();
  void pathChanged();

protected:
  void drawDecomposition(std::vector<mrs_lib::Polygon::ring_type>& polygons);
  void drawRing(mrs_lib::Polygon::ring_type& ring, geometry_msgs::TransformStamped tf, Ogre::SceneNode* node);
  void drawPath(mrs_msgs::PathSrv& path);
  void drawCurrentPolygon();

  Ogre::SceneNode* boundaries_node_ = nullptr;
  Ogre::SceneNode* decomposition_node_ = nullptr;
  Ogre::SceneNode* path_node_ = nullptr;

  rviz::BoolProperty* decomposition_property_;
  rviz::BoolProperty* boundaries_property_;
  rviz::BoolProperty* path_property_;
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif