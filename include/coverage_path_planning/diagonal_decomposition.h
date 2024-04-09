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
  typedef struct{
    mrs_lib::Point2d point;
    int ring_index;
    int id;
  } point_t;
  typedef std::pair<point_t, point_t> line_t;
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;
  typedef mrs_lib::Polygon::ring_type Ring;

  // Makes one iteration of MP3 algorithm
  // TODO: Pure function (?) 
  bool getPartition(std::vector<point_t>& border, int index_start, std::vector<point_t>& res_poly, std::pair<point_t, point_t>& res_line);
  
  // Returns true if terminated
  bool getPartitionClockwise(const std::vector<point_t>& border, int index_start, std::vector<point_t>& res);
  bool getPartitionCounterClockwise(const std::vector<point_t>& border, int index_start, std::vector<point_t>& res);

  std::pair<Ring, line_t> drawTrueDiagonal(std::vector<std::vector<point_t>>& holes, line_t diagonal);

  // ang(a, b, c) denotes the angle between 0 and 360 degrees
  // swept by a counterclockwise rotation from line segment ba to line segment bc.
  float ang(mrs_lib::Point2d a, mrs_lib::Point2d b, mrs_lib::Point2d c);
  float ang(point_t a, point_t b, point_t c);

  // TODO: implement 
  float signedDistComparable(Line line, mrs_lib::Point2d point);

  bool fits(mrs_lib::Polygon& main, int start, mrs_lib::Polygon& part);

  point_t getClosestPoint(std::vector<std::vector<point_t>>& holes, point_t point);

  bool equals(point_t a, point_t b);

  bool intersects(std::vector<point_t>& ring1, std::vector<point_t>& ring2);

  bool intersects(std::vector<point_t>& ring, line_t& line);

  std::string printPolygon(std::vector<point_t>& poly);
  std::string printPoint(point_t& point);

  // void getPolygonBoundaries(mrs_lib::Polygon& poly, float& max_x, float& min_x,float& max_y, float& min_y);
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif