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
  // |---------------------------- Types ----------------------------|
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;
  typedef mrs_lib::Polygon::ring_type Ring;

  // |------------------------ Visualisation ------------------------|
  void drawDecomposition(std::vector<mrs_lib::Polygon::ring_type>& polygons);
  void drawRing(mrs_lib::Polygon::ring_type& ring, geometry_msgs::TransformStamped tf, Ogre::SceneNode* node);
  void drawPath(mrs_msgs::PathSrv& path);
  void drawCurrentPolygon();

  // |---------------------------- Tools ----------------------------|
  std::vector<mrs_lib::Point2d> getPath(mrs_lib::Point2d p1, mrs_lib::Point2d p2);

  std::vector<mrs_msgs::Reference> fixPath(std::vector<mrs_msgs::Reference>& path);

  // Returns -1 if no path has been found
  float getPathLen(mrs_lib::Point2d p1, mrs_lib::Point2d p2);

  // returns distance from point to infinite line, such that:
  //    1) distance from 2 points have the same sign iff there 
  //       are on the same side from the line
  //    2) abs(distance) is lower for the point that is closer to the line
  double signedDistComparable(Line line, mrs_lib::Point2d point);

  // returns distance from point to infinite line, such that:
  //    1) distance from 2 points have the same sign iff there 
  //       are on the same side from the line
  //    2) abs(distance) is lower for the point that is closer to the line
  double signedDistComparable(Ogre::Vector3 line, mrs_lib::Point2d point);

  // Shifts p1 and p2 towards each other by dist
  Line shrink(mrs_lib::Point2d p1, mrs_lib::Point2d p2, float dist);

  // |-------------------------- Attributes --------------------------|
  Ogre::SceneNode* boundaries_node_ = nullptr;
  Ogre::SceneNode* decomposition_node_ = nullptr;
  Ogre::SceneNode* path_node_ = nullptr;

  rviz::BoolProperty* decomposition_property_;
  rviz::BoolProperty* boundaries_property_;
  rviz::BoolProperty* path_property_;
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif