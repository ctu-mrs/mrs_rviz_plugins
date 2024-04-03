#ifndef COVERAGE_PATH_PLANNING_METHOD_STRIDES_H
#define COVERAGE_PATH_PLANNING_METHOD_STRIDES_H

#include "coverage_path_planning/approximate_decomposition.h"

#include <OGRE/OgreVector2.h>

#include <mrs_msgs/PathSrv.h>

#include <rviz/properties/editable_enum_property.h>

namespace mrs_rviz_plugins{
class StrideMethod : public ApproximateDecomposition {
Q_OBJECT
public:
  void compute() override;
  void start() override;

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node);

private:

typedef struct {
  Ogre::Vector2 start;
  Ogre::Vector2 direction;
  size_t len;
} stride_t;

typedef struct {
  bool first = false;
  bool second = false;
  int num = 0;
} limit_t;

  std::vector<Ogre::Vector2> getPathToNextCell(Ogre::Vector2 cur_cell);
  bool isNextToVisited(Ogre::Vector2 cell);
  void addCellToPath(Ogre::Vector2 cell);
  stride_t computeStride(Ogre::Vector2 start, Ogre::Vector2 direction);
  limit_t getLimits(Ogre::Vector2 cell, Ogre::Vector2 direction);

  // Returns false if cell.x and cell.y are valid indices of grid and 
  // corresponding element of grid is neither visited nor valid
  // true otherwise
  bool isLimit(Ogre::Vector2 cell);

  mrs_msgs::PathSrv path_;
  bool is_computed_ = false;
  Ogre::SceneNode* path_node_ = nullptr;

  rviz::IntProperty*          turn_num_property_;
  rviz::EditableEnumProperty* drone_name_property_;

  ros::ServiceClient client_;
};
} // namespace mrs_rviz_plugins

#endif