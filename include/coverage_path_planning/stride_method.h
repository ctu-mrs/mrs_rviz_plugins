#ifndef COVERAGE_PATH_PLANNING_METHOD_STRIDES_H
#define COVERAGE_PATH_PLANNING_METHOD_STRIDES_H

#include <coverage_path_planning/coverage_method.h>

namespace mrs_rviz_plugins{

class StrideMethod : public CoverageMethod {

  // StrideMethod(rviz::Property* property_container, Ogre::SceneManager* scene_manager);

  void update(mrs_lib::Polygon &new_polygon) override;

  void compute(mrs_lib::Polygon &new_polygon) override;

  void start();
};
} // namespace mrs_rviz_plugins

#endif