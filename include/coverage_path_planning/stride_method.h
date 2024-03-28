#ifndef COVERAGE_PATH_PLANNING_METHOD_STRIDES_H
#define COVERAGE_PATH_PLANNING_METHOD_STRIDES_H

#include <coverage_path_planning/coverage_method.h>

#include <rviz/properties/float_property.h>

namespace mrs_rviz_plugins{

class StrideMethod : public CoverageMethod {
public:

  void initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void compute() override;

  void setStart(Ogre::Vector3 position) override;

  void start();

  void setPolygon(mrs_lib::Polygon &new_polygon) override;

  void setAngle(int angle);

  void setOverlap(float percentage);

  void setHeight(float height);

protected:
  void drawGrid();

  typedef struct {
    float x;
    float y;
    bool visited = false;
    bool valid;
  } cell_t;

  std::vector<std::vector<cell_t>> grid;

private:
  Ogre::SceneNode* grid_node_;
};
} // namespace mrs_rviz_plugins

#endif