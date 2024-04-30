#ifndef MORSE_DECOMPOSITION_METHOD_H
#define MORSE_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/exact_decomposition.h"

#include <rviz/properties/int_property.h>
#include <rviz/properties/editable_enum_property.h>

#include <optional>

namespace mrs_rviz_plugins{

// This class uses linear function only
class MorseDecomposition : public ExactDecomposition {
Q_OBJECT
public:

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) override;

  void start() override;

  void compute() override;

  std::vector<mrs_msgs::Path> getPath() override;

  void setPath(std::vector<mrs_msgs::Path> paths) override;

  ~MorseDecomposition();

protected:
  //|----------------------------  Types ----------------------------|
  typedef struct{
    Ring partition;
    std::vector<std::vector<mrs_lib::Point2d>> paths;
    int id;
    std::vector<int> adjacent_cells;
    mrs_lib::Point2d crit_point1;
    mrs_lib::Point2d crit_point2;
  } cell_t;
  typedef struct{
    mrs_lib::Point2d point;
    int id = -1;
    int ring_id;
    bool is_new_edge = false;
    bool is_crit_p = false;
    int prev_point;
  } point_t;
  typedef struct {
    point_t p1;
    std::vector<point_t> part1;
    point_t crit_p;
    std::vector<point_t> part2;
    point_t p2;
    bool follow_cp;
  } edge_t;

  //|----------------- Procedures of Morse decomposition -----------------|

  // Computes critical points (extrems) of function f(x) = a*x1 + b*x2 + c restricted
  // to obstacle boundaries. a and b are given by start and twist.
  // This function can be overriden in order to implement Morse decomposition 
  // with another [Morse] function. In that case getEdge() and fillCells()
  // must be overriden too.
  virtual std::vector<point_t> getCriticalPoints(mrs_lib::Point2d start, Ring obstacle, float twist, int ring_id);

  // Returns edge_t corresponding to the critical point. 
  // In this implementation, edge is a straight finite line with ends of boundaries of 
  // the polygon. This corresponds to [Morse] function f(x) = a*x1 + b*x2 + c
  // View README for detailed explanation.
  // This function can be overriden in order to implement Morse decomposition 
  // with another [Morse] function. In that case getCriticalPoints() and fillCells()
  // must be overriden too.
  virtual std::optional<edge_t> getEdge(mrs_lib::Polygon& polygon, point_t crit_point, mrs_lib::Point2d start, float twist);

  // Initialises cell_t.paths and cell_t.adjacent_cells of the given cells.
  // In this implementation paths are boustrophedon coverage paths, which corresponds 
  // to [Morse] function f(x) = a*x1 + b*x2 + c.
  // Inserted paths must cover whole cells and can be reversed versions of one another,
  // since generatePath() does not reverse path on its own
  // This function can be overriden in order to implement Morse decomposition 
  // with another [Morse] function. In that case getEdge() and getCriticalPoints()
  // must be overriden too.
  virtual void fillCells(std::vector<cell_t>& cells, mrs_lib::Point2d start, float twist);

  // Fairly the main function of the class.
  // Given critical points it decomposes the whole polygon into partitions
  // and returns array of cells representing them.
  // 
  // start must be in polygon_frame
  // twist in radians [0, pi]
  std::vector<cell_t> getDecomposition(mrs_lib::Polygon& polygon,
                                      std::vector<point_t>& crit_points, 
                                      mrs_lib::Point2d start,
                                      float twist);

  // Support function for getDecomposition() to prevent it from overblowing (because it already is xd)
  // The function is called once decomposition algorithm runs into critical point
  void moveToNextAtCritPoint(point_t& next_point,
                          bool& is_prev_edge,
                          cell_t& cur_cell,
                          std::vector<std::optional<edge_t>>& edges,
                          std::vector<point_t>& crit_points,
                          std::vector<point_t>& cur_border,
                          std::vector<std::vector<point_t>>& cur_holes, 
                          std::vector<bool>& big_used,
                          std::vector<bool>& first_used,
                          std::vector<bool>& second_used);

  // Support function for getDecomposition() to prevent it from overblowing (because it already is xd)
  // The function is called once decomposition algorithm runs into point where a new edge must start
  void moveToNextAtNewEdge(point_t& next_point,
                          point_t& start_point,
                          bool& is_prev_edge,
                          cell_t& cur_cell,
                          std::vector<std::optional<edge_t>>& edges,
                          std::vector<point_t>& crit_points,
                          std::vector<point_t>& cur_border,
                          std::vector<std::vector<point_t>>& cur_holes, 
                          std::vector<bool>& big_used,
                          std::vector<bool>& first_used,
                          std::vector<bool>& second_used);

  // |------------------------- Generating path -------------------------|

  // Returns sequence of cell indices that produce optimal (in sence of length) path.
  // Implementation is DFS, where non-adjacent cells can be visited only if all adjacent 
  // ones has been visited
  // total_len is sum of lengths of transitions between cells
  // total_len is set to -1 if no path has been found
  std::vector<int> findPath(std::vector<cell_t>& cells, int start_index, mrs_lib::Point2d start_pos, float& total_len);

  // Converts cell sequence into coverage path
  // Note: start must be in polygon_frame_
  mrs_msgs::PathSrv generatePath(std::vector<cell_t>& cells, std::vector<int>& path, mrs_lib::Point2d start);

  std::vector<int> getAdjacentCells(std::vector<cell_t>& cells, int index);

  // Returns index of the path, which start is closest to point 
  // Returns -1 if cell has no path or all paths are empty
  int findClosest(cell_t& cell, mrs_lib::Point2d point);

  // |-------------------------- General tools --------------------------|

  point_t getNext(std::vector<point_t>& border, std::vector<std::vector<point_t>>& holes, point_t& cur);

  point_t getPrev(std::vector<point_t>& border, std::vector<std::vector<point_t>>& holes, point_t& cur);

  bool getWaypointPair(Ring& partition, Ogre::Vector3 sweep_dir, std::pair<mrs_lib::Point2d, mrs_lib::Point2d>& res);

  void pushPoints(Ring& ring, std::vector<point_t>& points);

  Ogre::Vector3 toLine(mrs_lib::Point2d start, float twist);

  std::optional<mrs_lib::Point2d> getIntersection(Ogre::Vector3 line, Line edge);

  //|---------------------------- Attributes----------------------------|
  bool is_computed_ = false;
  mrs_msgs::PathSrv path_;
  ros::ServiceClient client_;

  rviz::IntProperty* twist_property_;
  rviz::EditableEnumProperty* drone_name_property_;
  rviz::IntProperty* cell_num_property_;
  rviz::IntProperty* turn_num_property_;

}; // class MorseDecomposition
} // namespace mrs_rviz_plugins

#endif // MORSE_DECOMPOSITION_METHOD_H