#ifndef MORSE_DECOMPOSITION_METHOD_H
#define MORSE_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/exact_decomposition.h"

#include <optional>

namespace mrs_rviz_plugins{

// This class uses linear function only
class MorseDecomposition : public ExactDecomposition {
Q_OBJECT
public:

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void compute() override;

  void start() override;

protected:
  typedef mrs_lib::Polygon::ring_type Ring;
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;
  typedef struct{
    Ring partition;
    std::vector<std::vector<mrs_lib::Point2d>> paths;
    int id;
    std::vector<int> adjacent_cells;
    mrs_lib::Point2d crit_point1;
    mrs_lib::Point2d crit_point2;
  } cell_t;

  virtual std::vector<mrs_lib::Point2d> getCriticalPoints(mrs_lib::Point2d start, Ring obstacle, float twist);

  virtual std::vector<cell_t> getDecomposition(mrs_lib::Polygon polygon, 
                                              std::vector<mrs_lib::Point2d> crit_points, 
                                              mrs_lib::Point2d start,
                                              float twist);

  // |--------------------- Tools ---------------------|
  Ogre::Vector3 toLine(mrs_lib::Point2d start, float twist);

  std::optional<mrs_lib::Point2d> getIntersection(Ogre::Vector3 line, Line edge);

  // line is the slice containing the crit_point
  std::optional<Line> getEdge(mrs_lib::Polygon& polygon, mrs_lib::Point2d crit_point, Ogre::Vector3 line);

  double signedDistComparable(Line line, mrs_lib::Point2d point);

  double signedDistComparable(Ogre::Vector3 line, mrs_lib::Point2d point);
}; // class MorseDecomposition
} // namespace mrs_rviz_plugins

#endif // MORSE_DECOMPOSITION_METHOD_H