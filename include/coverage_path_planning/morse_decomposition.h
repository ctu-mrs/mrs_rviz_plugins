#ifndef MORSE_DECOMPOSITION_METHOD_H
#define MORSE_DECOMPOSITION_METHOD_H

#include "coverage_path_planning/exact_decomposition.h"

#include <rviz/properties/int_property.h>

#include <optional>

namespace mrs_rviz_plugins{

// This class uses linear function only
class MorseDecomposition : public ExactDecomposition {
Q_OBJECT
public:

  void initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) override;

  void compute() override;

  void start() override;

  void setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update=true) override;

  ~MorseDecomposition();

protected:
  typedef mrs_lib::Polygon::ring_type Ring;
  typedef boost::geometry::model::linestring<mrs_lib::Point2d> Line;
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

  virtual std::vector<point_t> getCriticalPoints(mrs_lib::Point2d start, Ring obstacle, float twist, int ring_id);

  std::vector<cell_t> getDecomposition(mrs_lib::Polygon& polygon, 
                                              std::vector<point_t>& crit_points, 
                                              mrs_lib::Point2d start,
                                              float twist);

  virtual void fillCells(std::vector<cell_t>& cells, mrs_lib::Point2d start, float twist);

  std::vector<int> getAdjacentCells(std::vector<cell_t>& cells, int index);

  // |--------------------- Tools ---------------------|
  point_t getNext(std::vector<point_t>& border, std::vector<std::vector<point_t>>& holes, point_t& cur);
  
  point_t getPrev(std::vector<point_t>& border, std::vector<std::vector<point_t>>& holes, point_t& cur);

  Line shrink(mrs_lib::Point2d p1, mrs_lib::Point2d p2, float dist);

  bool getWaypointPair(Ring& partition, Ogre::Vector3 sweep_dir, std::pair<mrs_lib::Point2d, mrs_lib::Point2d>& res);

  mrs_msgs::PathSrv generatePath(std::vector<cell_t>& cells, std::vector<int>& path, mrs_lib::Point2d start);

  std::vector<int> findPath(std::vector<cell_t>& cells, int start_index, mrs_lib::Point2d start_pos, float& total_len);

  int findClosest(cell_t& cell, mrs_lib::Point2d point);

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

  void pushPoints(Ring& ring, std::vector<point_t>& points);

  Ogre::Vector3 toLine(mrs_lib::Point2d start, float twist);

  std::optional<mrs_lib::Point2d> getIntersection(Ogre::Vector3 line, Line edge);

  double signedDistComparable(Line line, mrs_lib::Point2d point);

  double signedDistComparable(Ogre::Vector3 line, mrs_lib::Point2d point);

// private:

  // line is the slice containing the crit_point
  std::optional<edge_t> getEdge(mrs_lib::Polygon& polygon, point_t crit_point, Ogre::Vector3 line);

  rviz::IntProperty* twist_property_;

}; // class MorseDecomposition
} // namespace mrs_rviz_plugins

#endif // MORSE_DECOMPOSITION_METHOD_H