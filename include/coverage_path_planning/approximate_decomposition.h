#ifndef COVERAGE_PATH_PLANNING_APPROXIMATE_DECOMPOSITION_H
#define COVERAGE_PATH_PLANNING_APPROXIMATE_DECOMPOSITION_H

#include "coverage_path_planning/coverage_method.h"

#include <rviz/properties/int_property.h>

#include <mrs_lib/transformer.h>

#include <ros/ros.h>

namespace mrs_rviz_plugins{

class ApproximateDecomposition : public CoverageMethod {
Q_OBJECT 
public:

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void setStart(Ogre::Vector3 position) override;

  void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) override;

  void setAngle(int angle, bool update=true) override;

  void setOverlap(float percentage, bool update=true) override;

  void setHeight(float height, bool update=true) override;

  void setFrame(std::string new_frame, bool update=true) override;

protected Q_SLOTS:
  void twistChanged();

protected:
  // Fills the attribute *grid_ and visualizes it in Rviz 
  void drawGrid();

  typedef struct {
    float x;
    float y;
    bool visited = false;
    bool valid;
  } cell_t;

  // Functions for convenience
  void getPolygonBoundaries(float& max_x, float& min_x,float& max_y, float& min_y);
  void addWaypoint(geometry_msgs::Pose pose, bool valid);
  cell_t getCell(float min_x, float min_y, 
                 float camera_width, float dist, 
                 float twist_rad, size_t x, size_t y);
  std::optional<geometry_msgs::Pose> transform(cell_t cell, 
                                              float height, 
                                              Ogre::Quaternion quat, 
                                              geometry_msgs::TransformStamped tf);

  // Attributes
  std::vector<std::vector<cell_t>> grid_;

  rviz::IntProperty* twist_property_;
  rviz::IntProperty* cell_num_property_;

  Ogre::SceneNode* grid_node_;

  mrs_lib::Transformer transformer_;
  ros::NodeHandle      nh_;
};
} // namespace mrs_rviz_plugins

#endif