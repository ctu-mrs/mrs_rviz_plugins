#ifndef DIAGONAL_DECOMPOSITION_METHOD_H
#define DIAGONAL_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/exact_decomposition.h"

#include <rviz/properties/editable_enum_property.h>

#include <mrs_msgs/PathSrv.h>

#include <utility>
#include <optional>

namespace mrs_rviz_plugins{

class DiagonalDecomposition : public ExactDecomposition {
Q_OBJECT
public:

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void compute() override;

  void start() override;

protected:
  //|----------------------------  Types ----------------------------|
  // For more derailed description view .cpp file
  typedef struct{
    mrs_lib::Point2d point;
    int ring_index;
    int id;
  } point_t;
  typedef struct{
    std::vector<std::pair<mrs_lib::Point2d, mrs_lib::Point2d>> waypoints;
    std::vector<int> adjacent_polygons;
    int polygon_id = 0;
  } cell_t;
  typedef std::pair<point_t, point_t> line_t;
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;
  typedef mrs_lib::Polygon::ring_type Ring;

  //|------------------- Procedures of MP3 algorithm-------------------|
  
  // Performs whole MP3 algorithm
  std::vector<std::vector<point_t>> decompose(std::vector<point_t> border, std::vector<std::vector<point_t>> holes);
  
  // Searches for the biggest convex partition containg vertex at index_start
  // res_poly: generated partition
  // res_line: diagonal that creates the partition
  // returns true if the generated partition is the whole border
  bool getPartition(std::vector<point_t>& border, int index_start, std::vector<point_t>& res_poly, std::pair<point_t, point_t>& res_line);
  
  // Searches for the biggest convex partition containg vertex at 
  //    index_start and going clockwise.
  // Used by getPartition()
  // res_line: generated partition
  // returns true if the generated partition is the whole border
  bool getPartitionClockwise(const std::vector<point_t>& border, int index_start, std::vector<point_t>& res);

  // Same as getPartitionClockwise but searches in counter-clockwise order
  // always returns false (this method is only called if getPartitionClockwise 
  //      returned false, so whole polygon cannot be generated)
  bool getPartitionCounterClockwise(const std::vector<point_t>& border, int index_start, std::vector<point_t>& res);

  // Searches for a diagonal that is not intersected by any hole. 
  // Part of MP3 algorithm
  line_t drawTrueDiagonal(std::vector<std::vector<point_t>>& holes, line_t diagonal);

  //|----------------- Searching for exhaustive path ------------------|

  // param polygons: partitions of the decomposed polygon
  // returns a vector of cells that represent individual partitions
  std::vector<cell_t> fillCells(std::vector<std::vector<point_t>>& polygons);
  
  // Changes cell_t.waypoints of every cell so that each waypoint.first 
  //    is on one side and each waypoint.second is on the other
  void fixCells(std::vector<DiagonalDecomposition::cell_t>& cells);

  // For given infinite line represented by sweep_direction searches for 
  //    intersections with edges of the polygon.
  // returns true if 2 intersections were found
  bool getWaypointPair(std::vector<point_t>& polygon, Ogre::Vector3 sweep_dir, float distance, std::pair<mrs_lib::Point2d, mrs_lib::Point2d>& res);
  
  // returns intersection point of infinite line and the edge (if any)
  std::optional<mrs_lib::Point2d> getIntersection(Ogre::Vector3 line, Line edge);

  // For given polygons[index] returns vector of indices of polygons that 
  //    share at least 1 common vertex (except the given index)
  std::vector<int> getAdjacentPolygons(std::vector<std::vector<point_t>>& polygons, int index);

  // Returns vector which defines infinite line normal to 
  // sweep direction in sence of the publication (i.e. vector.x vector.y define sweep direction)
  Ogre::Vector3 getSweepDirection(std::vector<point_t>& polygon);

  // DFS algorithm searching for the best permutation of cells
  // If a dead end is reached, continue the search with any of unvisited cells
  // Must allways return true
  bool findPath(std::vector<cell_t>& cells, 
                mrs_lib::Point2d prev_point,
                std::set<int> visited, 
                int cur_index, 
                int path_len, 
                std::vector<int>& path,
                float& total_path_len);

  //|------------------------------ Tools------------------------------|

  // Considering that a UAV is at prev_point, finds the closest point 
  // to start coverage of the cell (start_point) and where the coverage 
  // path finishes (finish_point).
  void getStartAndFinish(mrs_lib::Point2d prev_point, 
                        cell_t& cur_cell, 
                        mrs_lib::Point2d& start_point, 
                        mrs_lib::Point2d& finish_point);
  mrs_msgs::PathSrv genereatePath(std::vector<cell_t>& cells, std::vector<int> path, mrs_lib::Point2d start);

  // ang(a, b, c) denotes the angle between 0 and 360 degrees
  // swept by a counterclockwise rotation from line segment ba to line segment bc.
  double ang(mrs_lib::Point2d a, mrs_lib::Point2d b, mrs_lib::Point2d c);
  
  // Convenient overload of ang()
  double ang(point_t a, point_t b, point_t c);

  // returns distance from point to infinite line, such that:
  //    1) distance from 2 points have the same sign iff there 
  //       are on the same side from the line
  //    2) abs(distance) is lower for the point that is closer to the line
  double signedDistComparable(Line line, mrs_lib::Point2d point);

  point_t getClosestPoint(std::vector<std::vector<point_t>>& holes, point_t point);

  bool equals(point_t a, point_t b);

  // Same as boost::geometry::intersects, but works for custom datatypes
  bool intersects(std::vector<point_t>& ring1, std::vector<point_t>& ring2);

  // Same as boost::geometry::intersects, but works for custom datatypes
  bool intersects(std::vector<point_t>& ring, line_t& line);

  std::string printPolygon(std::vector<point_t>& poly);
  std::string printPoint(point_t& point);

  //|---------------------------- Attributes----------------------------|
  bool is_computed_ = false;
  mrs_msgs::PathSrv path_;
  ros::ServiceClient client_;

  rviz::EditableEnumProperty* drone_name_property_;
  rviz::IntProperty* cell_num_property_;
  rviz::IntProperty* turn_num_property_;
}; // class DiagonalDecomposition
} // namespace mrs_rviz_plugins

#endif