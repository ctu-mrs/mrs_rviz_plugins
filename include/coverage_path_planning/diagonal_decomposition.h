#ifndef DIAGONAL_DECOMPOSITION_METHOD_H
#define DIAGONAL_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/exact_decomposition.h"

#include <utility>
#include <optional>
#include <mrs_msgs/PathSrv.h>

namespace mrs_rviz_plugins{

class DiagonalDecomposition : public ExactDecomposition {
Q_OBJECT
public:

  void compute() override;

  void start() override;

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

protected:
  typedef struct{
    mrs_lib::Point2d point;
    int ring_index;
    int id;
  } point_t;
  typedef std::pair<point_t, point_t> line_t;
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;
  typedef mrs_lib::Polygon::ring_type Ring;

  //|------------------- Procedures of MP3 algorithm-------------------|
  // Returns true if terminated
  // TODO: Pure function (?) 
  // Makes one iteration of MP3 algorithm
  bool getPartition(std::vector<point_t>& border, int index_start, std::vector<point_t>& res_poly, std::pair<point_t, point_t>& res_line);
  
  bool getPartitionClockwise(const std::vector<point_t>& border, int index_start, std::vector<point_t>& res);
  bool getPartitionCounterClockwise(const std::vector<point_t>& border, int index_start, std::vector<point_t>& res);

  std::pair<Ring, line_t> drawTrueDiagonal(std::vector<std::vector<point_t>>& holes, line_t diagonal);


  //|----------------- Searching for exhaustive path ------------------|
  typedef struct{
    std::vector<std::pair<mrs_lib::Point2d, mrs_lib::Point2d>> waypoints;
    std::vector<int> adjacent_polygons;
    int polygon_id = 0;
  } cell_t;

  std::vector<cell_t> fillCells(std::vector<std::vector<point_t>>& polygons);
  bool getWaypointPair(std::vector<point_t>& polygon, Ogre::Vector3 sweep_dir, float distance, std::pair<mrs_lib::Point2d, mrs_lib::Point2d>& res);
  std::optional<mrs_lib::Point2d> getIntersection(Ogre::Vector3 line, Line edge);

  std::vector<int> getAdjacentPolygons(std::vector<std::vector<point_t>>& polygons, int index);
  void fixCells(std::vector<DiagonalDecomposition::cell_t>& cells);

  // Returns vector which defines infinite line normal to 
  // sweep direction in sence of the publication (i.e. vector.x vector.y define sweep direction)
  Ogre::Vector3 getSweepDirection(std::vector<point_t>& polygon);
  bool findPath(std::vector<cell_t>& cells, 
                mrs_lib::Point2d prev_point,
                std::set<int> visited, 
                int cur_index, 
                int path_len, 
                std::vector<int>& path,
                float& total_path_len);

  void getStartAndFinish(mrs_lib::Point2d prev_point, 
                        cell_t& cur_cell, 
                        mrs_lib::Point2d& start_point, 
                        mrs_lib::Point2d& finish_point);


  //|------------------------------ Tools------------------------------|
  mrs_msgs::PathSrv genereatePath(std::vector<cell_t>& cells, std::vector<int> path, mrs_lib::Point2d start);

  // ang(a, b, c) denotes the angle between 0 and 360 degrees
  // swept by a counterclockwise rotation from line segment ba to line segment bc.
  float ang(mrs_lib::Point2d a, mrs_lib::Point2d b, mrs_lib::Point2d c);
  float ang(point_t a, point_t b, point_t c);

  float signedDistComparable(Line line, mrs_lib::Point2d point);

  bool fits(mrs_lib::Polygon& main, int start, mrs_lib::Polygon& part);

  point_t getClosestPoint(std::vector<std::vector<point_t>>& holes, point_t point);
  // Point2d getClosestPoint(Line line, Point2d point);

  bool equals(point_t a, point_t b);

  bool intersects(std::vector<point_t>& ring1, std::vector<point_t>& ring2);

  bool intersects(std::vector<point_t>& ring, line_t& line);

  std::string printPolygon(std::vector<point_t>& poly);
  std::string printPoint(point_t& point);
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif