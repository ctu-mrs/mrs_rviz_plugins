#ifndef MORSE_DECOMPOSITION_METHOD_H
#define MORSE_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/exact_decomposition.h"

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
  typedef struct{
    Ring partition;
    std::vector<std::vector<mrs_lib::Point2d>> paths;
    int id;
    std::vector<int> adjacent_cells;
    mrs_lib::Point2d crit_point1;
    mrs_lib::Point2d crit_point2;
  } cell_t;

  virtual std::vector<mrs_lib::Point2d> getCriticalPoints(mrs_lib::Point2d start, Ring obstacle, float twist);

}; // class MorseDecomposition
} // namespace mrs_rviz_plugins

#endif // MORSE_DECOMPOSITION_METHOD_H