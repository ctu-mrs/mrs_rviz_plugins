#ifndef COVERAGE_PATH_PLANNING_METHOD_STRIDES_H
#define COVERAGE_PATH_PLANNING_METHOD_STRIDES_H

#include <coverage_path_planning/coverage_method.h>

#include <rviz/properties/int_property.h>

namespace mrs_rviz_plugins{

class StrideMethod : public CoverageMethod {
Q_OBJECT 
public:

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void compute() override;

  void setStart(Ogre::Vector3 position) override;

  void start();

  void setPolygon(mrs_lib::Polygon &new_polygon, bool update=true) override;

  void setAngle(int angle, bool update=true) override;

  void setOverlap(float percentage, bool update=true) override;

  void setHeight(float height, bool update=true) override;

protected Q_SLOTS:
  void twistChanged();

protected:
  void drawGrid();

  typedef struct {
    float x;
    float y;
    bool visited = false;
    bool valid;
  } cell_t;

  std::vector<std::vector<cell_t>> grid;

  rviz::IntProperty* twist_property_;

private:
  Ogre::SceneNode* grid_node_;
};
} // namespace mrs_rviz_plugins

#endif