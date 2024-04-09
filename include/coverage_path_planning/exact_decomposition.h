#ifndef EXACT_DECOMPOSITION_METHOD_H
#define EXACT_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/coverage_method.h"

#include <utility>

namespace mrs_rviz_plugins{

class ExactDecomposition : public CoverageMethod {
Q_OBJECT
public: 

  void setStart(Ogre::Vector3 position) override;

  void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) override;

  void setAngle(int angle, bool update=true) override;

  void setOverlap(float percentage, bool update=true) override;

  void setHeight(float height, bool update=true) override;

  void setFrame(std::string new_frame, bool update=true) override;

protected:
  void drawDecomposition(std::vector<mrs_lib::Polygon::ring_type>& polygons);
  void drawRing(mrs_lib::Polygon::ring_type& ring, geometry_msgs::TransformStamped tf, Ogre::SceneNode* node);
  void drawCurrentPolygon();

  Ogre::SceneNode* boundaries_node_ = nullptr;
  Ogre::SceneNode* decomposition_node_ = nullptr;
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif