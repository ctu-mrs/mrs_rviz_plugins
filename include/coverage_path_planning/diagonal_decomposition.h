#ifndef DIAGONAL_DECOMPOSITION_METHOD_H
#define DIAGONAL_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/coverage_method.h"

#include <utility>

namespace mrs_rviz_plugins{

class DiagonalDecomposition : public CoverageMethod {
Q_OBJECT
public: 

  void compute() override;

  void start() override;

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void setStart(Ogre::Vector3 position) override;

  void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) override;

  void setAngle(int angle, bool update=true) override;

  void setOverlap(float percentage, bool update=true) override;

  void setHeight(float height, bool update=true) override;

  void setFrame(std::string new_frame, bool update=true) override;

protected:
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;

  // Makes one iteration of MP3 algorithm
  // TODO: Pure function (?) 
  std::pair<mrs_lib::Polygon, Line> getPartition(mrs_lib::Polygon& border, int index_start);

  // polygon: convex partition with holes
  // diagonal: initial edge that must become a "true" one
  // Returns: ring: the hole that contains endpoint of true diagonal
  //          line: a diagonal that is not intersected by any holes, starts at diagonal[0] 
  // TODO: Pure function (?) 
  std::pair<mrs_lib::Polygon::ring_type, Line> drawTrueDiagonal(mrs_lib::Polygon& polygon, Line diagonal);

  // ang(a, b, c) denotes the angle between 0 and 360 degrees
  // swept by a counterclockwise rotation from line segment ba to line segment bc.
  float ang(mrs_lib::Point2d a, mrs_lib::Point2d b, mrs_lib::Point2d c);

  // void getPolygonBoundaries(mrs_lib::Polygon& poly, float& max_x, float& min_x,float& max_y, float& min_y);
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif