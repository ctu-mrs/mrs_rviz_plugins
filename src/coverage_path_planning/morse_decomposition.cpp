#include "coverage_path_planning/morse_decomposition.h"
#include "coverage_path_planning/planner_tool.h"

#include <algorithm>
#include <optional>
#include <numeric>
#include <limits>
#include <random>
#include <cmath>
#include <list>

// Uncomment this for useful outputs during computations
// #define DEBUG

namespace bg = boost::geometry;

using Polygon = mrs_lib::Polygon;
using Point2d = mrs_lib::Point2d;
using std::vector;

namespace mrs_rviz_plugins{

  //|------------------------------------------------------------------|
  //|-------------------- Public overriden methods --------------------|
  //|------------------------------------------------------------------|

void MorseDecomposition::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) {
  ExactDecomposition::initialize(property_container, scene_manager, root_node);
  twist_property_ = new rviz::IntProperty("Twist", 0, "Twist of the morse funstion", property_container);
  drone_name_property_ = new rviz::EditableEnumProperty("Uav", "", "Uav used to perform coverage mission", property_container);
  cell_num_property_ = new rviz::IntProperty("Cell number", 0, "Number of cells in current decomposition", property_container);
  turn_num_property_ = new rviz::IntProperty("Turn number", 0, "Number of turns in current path", property_container);

  twist_property_->setMin(0);
  twist_property_->setMax(180);
  cell_num_property_->setReadOnly(true);
  turn_num_property_->setReadOnly(true);

  std::vector<std::string> drone_names = PlannerTool::getUavNames();
  for(auto& name : drone_names){
    drone_name_property_->addOption(name.c_str());
  }

  if(drone_names.size() > 0){
    drone_name_property_->setString(drone_names[0].c_str());
  }else{
    ROS_WARN("[MorseDecomposition]: could not find any uav for coverage mission");
  }
}

// All the vertices of the polygon are shifted with random noise in order to
// avoid edge cases of searching for critical points and constructing edges
void MorseDecomposition::setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update){
  current_polygon_ = new_polygon;
  std::default_random_engine generator;
  float upper = 0.02;
  float lower = -0.02;
  std::uniform_real_distribution<float> distribution(lower, upper);

  for(Point2d& p : current_polygon_.outer()){
    float x = bg::get<0>(p);
    float y = bg::get<1>(p);

    x += distribution(generator);
    y += distribution(generator);

    bg::set<0>(p, x);
    bg::set<1>(p, y);
  }
  current_polygon_.outer().back() = current_polygon_.outer().front();

  auto& holes = bg::interior_rings(current_polygon_);
  for(auto& hole : holes){
    for(Point2d& p : hole){
      float x = bg::get<0>(p);
      float y = bg::get<1>(p);

      x += distribution(generator);
      y += distribution(generator);

      bg::set<0>(p, x);
      bg::set<1>(p, y);
    }
    hole.back() = hole.front();
  }

  ExactDecomposition::setPolygon(frame_id, current_polygon_, update);
}

void MorseDecomposition::start() {
  if(!is_computed_){
    ROS_WARN("[MorseDecomposition]: Could not start the mission. The path has not been computed yet.");
    return;
  }
  client_ = nh_.serviceClient<mrs_msgs::PathSrv>("/" + drone_name_property_->getStdString() + "/trajectory_generation/path");

  // Make the call
  if(!client_.call(path_)){
    ROS_INFO("[MorseDecomposition]: Call failed. Service name: %s", client_.getService().c_str());
    return;
  }
  if (!path_.response.success) {
    ROS_INFO("[MorseDecomposition]: Call failed: %s", path_.response.message.c_str());
    return;
  } 
  ROS_INFO("[MorseDecomposition]: Call processed successfully");
}

void MorseDecomposition::compute() {
  std::string msg;
  if (!bg::is_valid(current_polygon_, msg)){
    ROS_ERROR("[MorseDecomposition]: current polygon is not valid. Either invalid polygon has been recieved or vertices has been shifted inappropriately. Try increase distance between vertices");
    return;
  }

  float twist_rad = (((float)twist_property_->getInt()) / 180) * M_PI;

  geometry_msgs::Point start_p;
  start_p.x = start_position_.x;
  start_p.y = start_position_.y;
  start_p.z = start_position_.z;
  auto transformed = transformer_.transformSingle(current_frame_, start_p, polygon_frame_);
  if(!transformed){
    ROS_ERROR("[MorseDecomposition]: Could not transform start position from %s to %s. Terminating computation.", current_frame_.c_str(), polygon_frame_.c_str());
    return;
  }
  start_p = transformed.value();
  Point2d start{start_p.x, start_p.y};

  vector<point_t> cps = getCriticalPoints(start, current_polygon_.outer(), twist_rad, -1);
  vector<Ring> holes = bg::interior_rings(current_polygon_); 
  for(int i=0; i<holes.size(); i++){
    vector<point_t> tmp = getCriticalPoints(start, holes[i], twist_rad, i);
    cps.insert(cps.end(), tmp.begin(), tmp.end());
  }

  // todo: add crit points into the polygon

  vector<cell_t> decomposition = getDecomposition(current_polygon_, cps, start, twist_rad);
  vector<Ring> decomposition_rings;
  decomposition_rings.reserve(decomposition.size());
  for(cell_t& cell : decomposition){
    decomposition_rings.push_back(cell.partition);
  }
  drawDecomposition(decomposition_rings);
  cell_num_property_->setInt(decomposition.size());

  fillCells(decomposition, start, twist_rad);
  #ifdef DEBUG
  std::cout << "cells are filled\n";
  #endif // DEBUG

  vector<int> optimal_cell_sequence;
  float optimal_len = std::numeric_limits<float>::max();
  bool found = false;
  #ifdef DEBUG
  std::cout << "searching for cell seq\n";
  #endif // DEBUG
  for(int i=0; i<decomposition.size(); i++){
    float cur_len = 0;
    vector<int> cur_path = findPath(decomposition, i, start, cur_len);
    if(cur_len >= 0 && cur_len < optimal_len){
      optimal_len = cur_len;
      optimal_cell_sequence = cur_path;
      found = true;
    }
  }

  if(!found){
    ROS_ERROR("[MorseDecomposition]: Could not find any cell sequence. No path has been computed");
    return;
  }
  #ifdef DEBUG
  std::cout << "cell seq found\n";
  #endif // DEBUG

  path_ = generatePath(decomposition, optimal_cell_sequence, start);
  is_computed_ = true;
  turn_num_property_->setInt(path_.request.path.points.size() - 1);
  #ifdef DEBUG
  std::cout << "path generated\n";
  #endif // DEBUG

  drawPath(path_);
  return;
}

MorseDecomposition::~MorseDecomposition(){
  delete twist_property_;
  delete drone_name_property_;
  delete cell_num_property_;
  delete turn_num_property_;
}

  //|------------------------------------------------------------------|
  //|---------------- Procedures of Morse decomposition ---------------|
  //|------------------------------------------------------------------|

// Source: https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  std::stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

vector<MorseDecomposition::point_t> MorseDecomposition::getCriticalPoints(Point2d start, Ring obstacle, float twist, int ring_id) {
  // 1. Compute parameters of infinite line (the slice)
  Ogre::Vector3 line = toLine(start, twist);

  // 2. Compute values of the (linear) function at each vertex
  vector<float> values(obstacle.size() - 1);
  for(int i=0; i<obstacle.size() - 1; i++){
    values[i] = line.x * bg::get<0>(obstacle[i]) + line.y * bg::get<1>(obstacle[i]) + line.z; // Note: line.z seems to be redundant since we compare all the values afterwards
  }

  // 3. Find values that are greater than previous and next ones.
  //    Points of these values are critical ones.
  vector<point_t> result;
  float prev = values.back();
  float cur  = values[0];
  float next = values[1];
  for(int i=0; i<values.size(); i++){
    if((cur > prev && cur > next) || (cur < prev && cur < next)){
      point_t crit_point;
      crit_point.point = obstacle[i];
      crit_point.id = i;
      crit_point.ring_id = ring_id;
      crit_point.is_crit_p = true;
      result.push_back(crit_point);
      #ifdef DEBUG
      std::cout << "Critical point found: " << bg::wkt(crit_point.point) << std::endl;
      #endif // DEBUG
    }
    prev = cur;
    cur = next;
    next = values[(i+2) % values.size()];
  }

  return result;
}

std::optional<MorseDecomposition::edge_t> MorseDecomposition::getEdge(Polygon& polygon, point_t crit_point, mrs_lib::Point2d start, float twist) {
  Ogre::Vector3 line = toLine(start, twist);
  // 1. Find all the intersections of line with edges of polygon
  vector<point_t> intersections;
  for(int i=0; i<polygon.outer().size() - 1; i++){
    Line edge;
    edge.push_back(polygon.outer()[i]);
    edge.push_back(polygon.outer()[i+1]);

    // Skip intersections with critical point itself
    if(bg::equals(edge[0], crit_point.point) || bg::equals(edge[1], crit_point.point)){
      continue;
    }

    auto intersection = getIntersection(line, edge);
    if(intersection){
      point_t tmp;
      tmp.point = intersection.value();
      tmp.is_new_edge = true;
      tmp.prev_point = i;
      tmp.ring_id = -1;
      tmp.id = -1;
      intersections.push_back(tmp);
    }
  }

  // 2. Find all the intersections of line with holes of polygon
  vector<Ring>& obstacles = bg::interior_rings(polygon);
  for(int j=0; j<obstacles.size(); j++){
    Ring& obstacle = obstacles[j];
    for(int i=0; i<obstacle.size() - 1; i++){
      Line edge;
      edge.push_back(obstacle[i]);
      edge.push_back(obstacle[i+1]);

      // Skip intersections with critical point itself
      if(bg::equals(edge[0], crit_point.point) || bg::equals(edge[1], crit_point.point)){
        continue;
      }

      auto intersection = getIntersection(line, edge);
      if(intersection){
        point_t tmp;
        tmp.point = intersection.value();
        tmp.is_new_edge = true;
        tmp.prev_point = i;
        tmp.ring_id = j;
        tmp.id = -1;
        intersections.push_back(tmp);
      }
    }
  }

  // 3. Compute signed distances from critical point to the intersections
  vector<float> distances(intersections.size());
  Ogre::Vector3 norm_line;
  norm_line.x = line.y;
  norm_line.y = -line.x;
  norm_line.z = - ( norm_line.x * bg::get<0>(crit_point.point)  + norm_line.y * bg::get<1>(crit_point.point) );
  for(int i=0; i<intersections.size(); i++){
    distances[i] = signedDistComparable(norm_line, intersections[i].point);
  }

  if(intersections.size() == 0){
    return std::nullopt;
  }

  // 4. Find closest intersections on both sides of line
  bool found_negative = false;
  bool found_positive = false;
  float closest_negative_dist = - std::numeric_limits<float>::max();
  float closest_positive_dist = std::numeric_limits<float>::max();
  point_t closest_negative_point;
  point_t closest_positive_point;
  for(int i=0; i<distances.size(); i++){
    if(distances[i] > 0){
      if(distances[i] < closest_positive_dist){
        closest_positive_dist = distances[i];
        closest_positive_point = intersections[i];
        found_positive = true;
      }
    } else{
      if(distances[i] > closest_negative_dist){
        closest_negative_dist = distances[i];
        closest_negative_point = intersections[i];
        found_negative = true;
      }
    }
  }

  if(!found_positive || !found_negative){
    return std::nullopt;
  }

  edge_t result;
  result.crit_p = crit_point;
  result.p1 = closest_negative_point;
  result.p2 = closest_positive_point;
  #ifdef DEBUG
  std::cout << "getEdge: crit_point is " << (crit_point.is_crit_p ? "true" : "false") << std::endl;
  std::cout << "getEdge: p1 is " << bg::wkt(result.p1.point) << std::endl;
  std::cout << "getEdge: p1 is " << bg::wkt(result.p2.point) << std::endl;
  #endif // DEBUG

  // 5. Checking if the edge lies within the polygon
  // Note: bg::covered_by() does not work properly in this case,
  //    probably because of inacuracy of float type. 
  Line half1;
  half1.push_back(result.crit_p.point);
  half1.push_back(result.p1.point);
  Line half2;
  half2.push_back(result.crit_p.point);
  half2.push_back(result.p2.point);
  Point2d center1;
  bg::centroid(half1, center1);
  Point2d center2;
  bg::centroid(half1, center2);

  bool is_covered;
  is_covered = bg::within(center1, polygon) && bg::within(center2, polygon);

  if(!is_covered){
    #ifdef DEBUG
    printf("edge (%.7f %.7f)  (%.7f %.7f) is not covered by the polygon\n", bg::get<0>(result.p1.point), bg::get<1>(result.p1.point), bg::get<0>(result.p2.point), bg::get<1>(result.p2.point));
    #endif // DEBUG
    return std::nullopt;
  }
  #ifdef DEBUG
  printf("edge (%.7f %.7f)  (%.7f %.7f) is covered by the polygon\n", bg::get<0>(result.p1.point), bg::get<1>(result.p1.point), bg::get<0>(result.p2.point), bg::get<1>(result.p2.point));
  #endif // DEBUG

  // 6. Define where to go after p1
  Point2d next_point;
  if(crit_point.ring_id == -1){
    next_point = polygon.outer()[crit_point.id + 1];
  } else{
    auto& holes = bg::interior_rings(polygon);
    next_point = holes[crit_point.ring_id][crit_point.id+1];
  }

  Line tmp_line;
  tmp_line.push_back(result.p1.point);
  tmp_line.push_back(result.p2.point);
  result.follow_cp = signedDistComparable(tmp_line, next_point) > 0;
  #ifdef DEBUG
  std::cout << "\t" << (result.follow_cp ? "follows cp" : "does not follow cp") << std::endl;
  #endif // DEBUG

  return result;
}

void MorseDecomposition::fillCells(vector<cell_t>& cells, Point2d start, float twist){
  float angle_rad = (((float)angle_) / 180) * M_PI;
  float camera_width = (std::tan(angle_rad / 2) * height_);
  float distance = camera_width * (1 - overlap_);

  for(int i=0; i<cells.size(); i++){
    cell_t& cell = cells[i];
    #ifdef DEBUG
    std::cout << "\nfilling cell #" << i << std::endl;
    #endif // DEBUG

    cell.id = i;
    cell.adjacent_cells = getAdjacentCells(cells, i);

    // Find best sweep direction
    Ogre::Vector3 line = toLine(cell.crit_point1, twist);
    float value1 = line.x * bg::get<0>(cell.crit_point1) + line.y * bg::get<1>(cell.crit_point1);
    float value2 = line.x * bg::get<0>(cell.crit_point2) + line.y * bg::get<1>(cell.crit_point2);
    if(value2 < value1){
      line = toLine(cell.crit_point2, twist);
    }

    // Find waypoints of coverage path
    vector<std::pair<Point2d, Point2d>> waypoints;
    for(float c=line.z-distance; ; c-=distance){
      line.z = c;
      std::pair<Point2d, Point2d> waypoint_pair;
      if(getWaypointPair(cell.partition, line, waypoint_pair)){
        waypoints.push_back(waypoint_pair);
        #ifdef DEBUG
        std::cout << "found waypoint: " << bg::wkt(waypoint_pair.first) << " " << bg::wkt(waypoint_pair.second) << std::endl;
        #endif // DEBUG
        continue;
      }
      break;
    }

    if(waypoints.size() == 0){
      std::cout << "cell does not have waypoints\n";
      Point2d center;
      bg::centroid(cell.partition, center);
      if(bg::within(center, cell.partition)){
        cell.paths.push_back({center});
      }
      continue;
    }

    // Fix waypoints
    Point2d cur_point = waypoints[0].first;
    for(std::pair<Point2d, Point2d>& pair : waypoints){
      if(bg::comparable_distance(cur_point, pair.first) > bg::comparable_distance(cur_point, pair.second)){
        Point2d tmp = pair.first;
        pair.first = pair.second;
        pair.second = tmp;
      }
      cur_point = pair.first;
    }

    // Shrink waypoints
    for(std::pair<Point2d, Point2d>& pair : waypoints){
      if(bg::distance(pair.first, pair.second) > 2*distance){
        Line shrinked = shrink(pair.first, pair.second, distance);
        pair.first = shrinked[0];
        pair.second = shrinked[1];
      }else{
        float x = (bg::get<0>(pair.first) + bg::get<0>(pair.second)) / 2;
        float y = (bg::get<1>(pair.first) + bg::get<1>(pair.second)) / 2;
        pair.first = Point2d{x, y};
        pair.second = Point2d{x, y};
      }
    }

    // Add paths
    vector<Point2d> path1;
    vector<Point2d> path2;
    path1.reserve(waypoints.size() * 2);
    path2.reserve(waypoints.size() * 2);
    bool is_first = true;
    for(std::pair<Point2d, Point2d>& pair : waypoints){
      if(is_first){
        path1.push_back(pair.first);
        path1.push_back(pair.second);
        path2.push_back(pair.second);
        path2.push_back(pair.first);
      }else{
        path1.push_back(pair.second);
        path1.push_back(pair.first);
        path2.push_back(pair.first);
        path2.push_back(pair.second);
      }
      is_first = !is_first;
    }
    cell.paths.push_back(path1);
    cell.paths.push_back(path2);
    std::reverse(path1.begin(), path1.end());
    std::reverse(path2.begin(), path2.end());
    cell.paths.push_back(path1);
    cell.paths.push_back(path2);
  }
}

vector<MorseDecomposition::cell_t> MorseDecomposition::getDecomposition(Polygon& polygon, vector<MorseDecomposition::point_t>& crit_points, mrs_lib::Point2d start, float twist) {
  // 1. Find all the edges generated by critical points
  vector<std::optional<edge_t>> edges;
  for(point_t& cp : crit_points){
    Ogre::Vector3 line = toLine(cp.point, twist);
    edges.push_back(getEdge(polygon, cp, cp.point, twist));
  }
  #ifdef DEBUG
  std::cout << "edges computed\n";
  #endif // DEBUG

  // 2. Transform polygon into more convenient format
  vector<point_t> cur_border(polygon.outer().size()-1);
  for(int i=0; i<polygon.outer().size() - 1; i++){
    cur_border[i].id = i;
    cur_border[i].ring_id = -1;
    cur_border[i].point = polygon.outer()[i];
  }
  #ifdef DEBUG
  std::cout << "border transformed\n";
  #endif // DEBUG
  vector<Ring> holes = bg::interior_rings(polygon);
  vector<vector<point_t>> cur_holes(holes.size());
  for(int i=0; i<holes.size(); i++){
    vector<point_t> new_hole(holes[i].size() - 1);
    for(int j=0; j<new_hole.size(); j++){
      // filling from the end to define the hole in clockwise order
      // int cur_index = new_hole.size() - j;
      int cur_index = j;
      new_hole[cur_index].id = cur_index;
      new_hole[cur_index].ring_id = i;
      new_hole[cur_index].point = holes[i][cur_index];
    }
    cur_holes[i] = new_hole;
  }
  #ifdef DEBUG
  std::cout << "holes transformed\n";
  #endif // DEBUG

  // 3. Insert points of edges into the border
  vector<point_t> new_border;
  for(int i=0; i<cur_border.size(); i++){
    // Insert current
    new_border.push_back(cur_border[i]);

    // Insert any new points
    Line cur_edge;
    cur_edge.push_back(cur_border[i].point);
    cur_edge.push_back(cur_border[(i+1) % cur_border.size()].point);

    std::vector<point_t> new_points;
    std::vector<float> distances;
    for(auto& new_edge_opt : edges){
      if(!new_edge_opt){
        continue;
      }
      edge_t new_edge = new_edge_opt.value(); 
      if(new_edge.p1.prev_point == i && new_edge.p1.ring_id == -1){
        new_points.push_back(new_edge.p1);
        distances.push_back(bg::comparable_distance(cur_edge[0], new_edge.p1.point));
        #ifdef DEBUG
        std::cout << "\t" << bg::wkt(new_edge.p1.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
        #endif // DEBUG
      }

      if(new_edge.p2.prev_point == i && new_edge.p2.ring_id == -1){
        new_points.push_back(new_edge.p2);
        distances.push_back(bg::comparable_distance(cur_edge[0], new_edge.p2.point));
        #ifdef DEBUG
        std::cout << "\t" << bg::wkt(new_edge.p2.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
        #endif // DEBUG
      }
    }

    for(size_t k : sort_indexes(distances)){
      new_border.push_back(new_points[k]);
    }
  }
  cur_border = new_border;
  #ifdef DEBUG
  std::cout << "points inserted into border\n";
  #endif // DEBUG

  // 4. Insert points of edges into the holes
  vector<vector<point_t>> new_holes(cur_holes.size());
  for(int i=0; i<cur_holes.size(); i++){
    vector<point_t> new_hole;
    for(int j=0; j<cur_holes[i].size(); j++){
      // Insert current
      new_hole.push_back(cur_holes[i][j]);

      // Insert any new points
      Line cur_edge;
      cur_edge.push_back(cur_holes[i][j].point);
      cur_edge.push_back(cur_holes[i][(j+1) % cur_holes.size()].point);

      std::vector<point_t> new_points;
      std::vector<float> distances;
      for(auto& new_edge_opt : edges){
        if(!new_edge_opt){
          continue;
        }
        edge_t new_edge = new_edge_opt.value(); 
        if(new_edge.p1.prev_point == j && new_edge.p1.ring_id == i){
          new_points.push_back(new_edge.p1);
          distances.push_back(bg::comparable_distance(cur_edge[0], new_edge.p1.point));
          #ifdef DEBUG
          std::cout << "\t" << bg::wkt(new_edge.p1.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
          #endif // DEBUG
        }

        if(new_edge.p2.prev_point == j && new_edge.p2.ring_id == i){
          new_points.push_back(new_edge.p2);
          distances.push_back(bg::comparable_distance(cur_edge[0], new_edge.p2.point));
          #ifdef DEBUG
          std::cout << "\t" << bg::wkt(new_edge.p2.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
          #endif // DEBUG
        }
      }

      for(size_t k : sort_indexes(distances)){
        new_hole.push_back(new_points[k]);
      }
    }
    new_holes[i] = new_hole;
  }
  cur_holes = new_holes;
  #ifdef DEBUG
  std::cout << "points inserted into holes\n";
  #endif // DEBUG

  // 5. Update data of points in cur_border and cur_holes
  #ifdef DEBUG
  std::cout << "border: \n";
  #endif // DEBUG
  for(int i=0; i<cur_border.size(); i++){
    cur_border[i].id = i;
    cur_border[i].ring_id = -1;
    #ifdef DEBUG
    std::cout << "\t" << bg::wkt(cur_border[i].point) << " " << cur_border[i].id << " " << (cur_border[i].is_new_edge ? "is new edge" : "")<< std::endl;
    #endif // DEBUG
  }
  #ifdef DEBUG
  std::cout << "holes:\n";
  #endif // DEBUG
  for(int i=0; i<cur_holes.size(); i++){
    for(int j=0; j<cur_holes[i].size(); j++){
      cur_holes[i][j].ring_id = i;
      cur_holes[i][j].id = j;
      #ifdef DEBUG
      std::cout << "\t" << bg::wkt(cur_holes[i][j].point) << " ring: " << i << " " << cur_holes[i][j].id << " " << (cur_holes[i][j].is_new_edge ? "is new edge" : "")<< std::endl;
      #endif // DEBUG
    }
    #ifdef DEBUG
    std::cout << std::endl;
    #endif // DEBUG
  }

  // 6. Update data of edge points
  #ifdef DEBUG
  std::cout << "Update data of edge points\n";
  #endif // DEBUG
  for(int i=0; i<edges.size(); i++){
    if(!edges[i]){
      continue;
    }
    edge_t& edge = edges[i].value();

    for(point_t& cur_p : cur_border){
      if(bg::equals(cur_p.point, edge.p1.point)){
        edge.p1 = cur_p;
      }
      if(bg::equals(cur_p.point, edge.p2.point)){
        edge.p2 = cur_p;
      }
      if(bg::equals(cur_p.point, edge.crit_p.point)){
        cur_p.is_crit_p = true;
        edge.crit_p = cur_p;
      }
    }

    for(auto& hole : cur_holes){
      for(point_t& cur_p : hole){
        if(bg::equals(cur_p.point, edge.p1.point)){
          edge.p1 = cur_p;
        }
        if(bg::equals(cur_p.point, edge.p2.point)){
          edge.p2 = cur_p;
        }
        if(bg::equals(cur_p.point, edge.crit_p.point)){
          cur_p.is_crit_p = true;
          edge.crit_p = cur_p;
        }
      }
    }
  }

  // 7. Update data of crit_points
  #ifdef DEBUG
  std::cout << "Update data of crit_points\n";
  #endif // DEBUG
  for(int i=0; i<crit_points.size(); i++){
    point_t& cp = crit_points[i];

    for(point_t& cur_p : cur_border){
      if(bg::equals(cur_p.point, cp.point)){
        cur_p.is_crit_p = true;
        cp = cur_p;
      }
    }

    for(auto& hole : cur_holes){
      for(point_t& cur_p : hole){
        if(bg::equals(cur_p.point, cp.point)){
          cur_p.is_crit_p = true;
          cp = cur_p;
        }
      }
    }
  }

  // 8. Divide the polygon into partitions
  #ifdef DEBUG
  std::cout << "Divide the polygon into partitions\n";
  #endif // DEBUG
  vector<cell_t> decomposition;
  vector<bool> big_used(crit_points.size(), false);
  vector<bool> first_used(crit_points.size(), false);
  vector<bool> second_used(crit_points.size(), false);
  // Here the hell begins
  for(int i=0; i<crit_points.size();){
    // if critical point generated all 3 partitions, move to the next one
    if(big_used[i] && first_used[i] && second_used[i]){
      #ifdef DEBUG
      std::cout << "\nskipping " << bg::wkt(crit_points[i].point) << std::endl;
      #endif // DEBUG
      i++;
      continue;
    }

    cell_t cur_cell;
    point_t next_point = crit_points[i];
    point_t start_point = crit_points[i];
    bool is_prev_edge = false;

    #ifdef DEBUG
    std::cout << "\nstarting with " << bg::wkt(crit_points[i].point) << std::endl;
    #endif // DEBUG
    // Briefly: find first unused end, insert edge and set next_point on p1 or p2
    if(!edges[i]){
      #ifdef DEBUG
      std::cout << "no edge\n";
      #endif // DEBUG
      big_used[i] = true;
      first_used[i] = true;
      second_used[i] = true;
    } else if(!big_used[i]){
      #ifdef DEBUG
      std::cout << "big edge\n";
      #endif // DEBUG
      big_used[i] = true;
      cur_cell.partition.push_back(crit_points[i].point);
      edge_t edge = edges[i].value();
      std::vector<point_t> insert;
      if(edge.follow_cp){
        insert = edge.part1;
        std::reverse(insert.begin(), insert.end());
        next_point = edge.p1;
      } else{
        insert = edge.part2;
        next_point = edge.p2;
      }
      is_prev_edge = true;
      pushPoints(cur_cell.partition, insert);
    } else if(!first_used[i]){
      #ifdef DEBUG
      std::cout << "first edge\n";
      #endif // DEBUG
      first_used[i] = true;
      is_prev_edge = true;
    } else if(!second_used[i]){
      #ifdef DEBUG
      std::cout << "second edge\n";
      #endif // DEBUG
      second_used[i] = true;
      is_prev_edge = false;
    } else{
      ROS_WARN("[MorseDecomposition]: Smth went wrong. Processing critical point that must be fully processed. Moving to next one");
      i++;
      continue;
    }
    cur_cell.crit_point1 = crit_points[i].point;

    int inserted = 0;
    while(!bg::equals(next_point.point, crit_points[i].point) || inserted == 0){
      inserted++;
      cur_cell.partition.push_back(next_point.point);
      #ifdef DEBUG
      std::cout << "inserted " << bg::wkt(next_point.point) << " " << next_point.ring_id << " " << next_point.id << std::endl;
      #endif // DEBUG

      if(next_point.is_crit_p){
        if(!bg::equals(cur_cell.crit_point1, next_point.point)){
          cur_cell.crit_point2 = next_point.point;
        }
        moveToNextAtCritPoint(next_point, is_prev_edge, cur_cell, edges, crit_points, cur_border, cur_holes, big_used, first_used, second_used);
      }
      else if(next_point.is_new_edge && !is_prev_edge){
        moveToNextAtNewEdge(next_point, start_point, is_prev_edge, cur_cell, edges, crit_points, cur_border, cur_holes, big_used, first_used, second_used);
      }
      else{
        next_point = getNext(cur_border, cur_holes, next_point);
        is_prev_edge = false;
      }
    }

    cur_cell.partition.push_back(cur_cell.partition.front());
    decomposition.push_back(cur_cell);
  } // end of for(int i=0; i<crit_points.size();)

  #ifdef DEBUG
  std::cout << "\nDECOMPOSED!\n";
  for(cell_t& cell : decomposition){
    std::cout << "partition:\n";
    for(Point2d& p : cell.partition){
      std::cout << "\t" << bg::wkt(p) << std::endl;
    }
    std::cout << std::endl;
  }
  #endif // DEBUG

  return decomposition;
} // end of getDecomposition()

void MorseDecomposition::moveToNextAtCritPoint(point_t& next_point,
                        bool& is_prev_edge,
                        cell_t& cur_cell,
                        std::vector<std::optional<edge_t>>& edges,
                        std::vector<point_t>&crit_points,
                        std::vector<point_t>& cur_border,
                        std::vector<std::vector<point_t>>& cur_holes, 
                        std::vector<bool>& big_used,
                        std::vector<bool>& first_used,
                        std::vector<bool>& second_used){
  #ifdef DEBUG
  std::cout << "\tbumped into a crit point\n";
  #endif
  edge_t found_edge;
  size_t found_edge_i = 0;
  bool found = false;
  // Find the edge we bumped into
  for(int i=0; i<edges.size(); i++){
    auto& edge_opt = edges[i];
    if(!edge_opt){
      continue;
    }
    if(bg::equals(next_point.point, edge_opt.value().crit_p.point)){
      found_edge = edge_opt.value();
      found = true;
      found_edge_i = i;
      break;
    }
  }

  if(!found){
    int cp_index = 0;
    for(int i=0; i<crit_points.size(); i++){
      if(bg::equals(crit_points[i].point, next_point.point)){
        cp_index = i;
        break;
      }
    }
    #ifdef DEBUG
    std::cout << "\t" << bg::wkt(crit_points[cp_index].point) << " has no edge\n";
    #endif // DEBUG

    big_used[cp_index] = true;
    first_used[cp_index] = true;
    second_used[cp_index] = true;
    next_point = getNext(cur_border, cur_holes, next_point);
    is_prev_edge = false;
    return;
  }

  #ifdef DEBUG
  std::cout << "\tmoving on after crit point\n";
  std::cout << "\tis_prev_edge: " << (is_prev_edge ? "true" : "false") << std::endl;
  #endif // DEBUG

  if(is_prev_edge){
    first_used[found_edge_i] = true;
    is_prev_edge = false;
    next_point = getNext(cur_border, cur_holes, next_point);
  }else{
    second_used[found_edge_i] = true;
    is_prev_edge = true;
    if(found_edge.follow_cp){
      next_point = found_edge.p2;
      pushPoints(cur_cell.partition, found_edge.part2);
    }else{
      next_point = found_edge.p1;
      auto to_insert = found_edge.part1;
      std::reverse(to_insert.begin(), to_insert.end());
      pushPoints(cur_cell.partition, to_insert);
    }
  }
}

void MorseDecomposition::moveToNextAtNewEdge(point_t& next_point, 
                        point_t& start_point,
                        bool& is_prev_edge,
                        cell_t& cur_cell,
                        std::vector<std::optional<edge_t>>& edges,
                        std::vector<point_t>& crit_points,
                        std::vector<point_t>& cur_border,
                        std::vector<std::vector<point_t>>& cur_holes, 
                        std::vector<bool>& big_used,
                        std::vector<bool>& first_used,
                        std::vector<bool>& second_used){
  #ifdef DEBUG
  std::cout << "\tbumped into new edge\n";
  #endif // DEBUG
  edge_t found_edge;
  size_t found_edge_i = 0;
  bool goto_cp = false;
  bool found = false;

  // Find the edge we bumped into
  for(int i=0; i<edges.size(); i++){
    auto& edge_opt = edges[i];
    if(!edge_opt){
      continue;
    }

    edge_t edge = edge_opt.value();
    bool equal_to_p1 = bg::equals(next_point.point, edge.p1.point);
    bool equal_to_p2 = bg::equals(next_point.point, edge.p2.point);
    if((equal_to_p1 && edge.follow_cp) || (equal_to_p2 && !edge.follow_cp)){
      found_edge = edge;
      found_edge_i = i;
      goto_cp = true;
      found = true;
      break;
    }
    if((equal_to_p1 && !edge.follow_cp) || (equal_to_p2 && edge.follow_cp)){
      found_edge = edge;
      found_edge_i = i;
      goto_cp = false;
      found = true;
      break;
    }
  }

  if(!found){
    next_point = getNext(cur_border, cur_holes, next_point);
    is_prev_edge = false;
    return;
  }

  #ifdef DEBUG
  std::cout << "\tgoto_cp is " << (goto_cp ? "true" : "false") << std::endl;
  std::cout << "\tfound_edge.follow_cp is " << (found_edge.follow_cp ? "true" : "false") << std::endl;
  #endif // DEBUG
  
  // Briefly: insert part of edge into current partition,
  //          set next_point to crit_point or p1 or p2
  is_prev_edge = true;
  if(found_edge.follow_cp && goto_cp){
    first_used[found_edge_i] = true;
    pushPoints(cur_cell.partition, found_edge.part1);
    next_point = found_edge.crit_p;
  }
  if(found_edge.follow_cp && !goto_cp){
    big_used[found_edge_i] = true;
    #ifdef DEBUG
    std::cout << "\tbig_used for " << bg::wkt(found_edge.crit_p.point) << std::endl;
    std::cout << "\tbig_used for " << bg::wkt(crit_points[found_edge_i].point) << std::endl;
    #endif // DEBUG
    vector<point_t> to_insert;
    to_insert.reserve(found_edge.part1.size() + found_edge.part2.size() + 1);
    for(point_t& p : found_edge.part2){
      to_insert.push_back(p);
    }
    std::reverse(to_insert.begin(), to_insert.end());

    // If crit point of the edge is the start point, partition is complete
    if(bg::equals(found_edge.crit_p.point, start_point.point)){
      pushPoints(cur_cell.partition, to_insert);
      next_point = found_edge.crit_p;
      return;
    }

    to_insert.push_back(found_edge.crit_p);
    cur_cell.crit_point2 = found_edge.crit_p.point;
    for(int i=found_edge.part1.size() - 1; i>=0; i--){
    // for(point_t& p : found_edge.part1){
      to_insert.push_back(found_edge.part1[i]);
    }
    pushPoints(cur_cell.partition, to_insert);
    next_point = found_edge.p1;
  }
  if(!found_edge.follow_cp && goto_cp){
    first_used[found_edge_i] = true;
    vector<point_t> to_insert = found_edge.part2;
    std::reverse(to_insert.begin(), to_insert.end());
    pushPoints(cur_cell.partition, to_insert);
    next_point = found_edge.crit_p;
  }
  if(!found_edge.follow_cp && !goto_cp){
    big_used[found_edge_i] = true;
    #ifdef DEBUG
    std::cout << "\tbig_used for " << bg::wkt(found_edge.crit_p.point) << std::endl;
    std::cout << "\tbig_used for " << bg::wkt(crit_points[found_edge_i].point) << std::endl;
    #endif // DEBUG
    pushPoints(cur_cell.partition, found_edge.part1);

    // If crit point of the edge is the start point, partition is complete
    if(bg::equals(found_edge.crit_p.point, start_point.point)){
      next_point = found_edge.crit_p;
      return;
    }

    cur_cell.partition.push_back(found_edge.crit_p.point);
    cur_cell.crit_point2 = found_edge.crit_p.point;
    pushPoints(cur_cell.partition, found_edge.part2);
    next_point = found_edge.p2;
  }
}

  //|------------------------------------------------------------------|
  //|------------------------- Generating path ------------------------|
  //|------------------------------------------------------------------|

vector<int> MorseDecomposition::findPath(vector<cell_t>& cells, int start_index, Point2d start_pos, float& total_len){
  // Results
  std::vector<float> total_lens;
  std::vector<vector<int>> total_paths;

  // Queues
  std::list<int> cells_to_visit{start_index};
  std::list<float> path_len{0};
  std::list<Point2d> last_point{start_pos};
  std::list<vector<int>> path_to_cell{{}}; // also serves as <visited>

  while(!cells_to_visit.empty()){
    // Take the info of current cell
    int cur_cell_i = cells_to_visit.front();
    float cur_path_len = path_len.front();
    Point2d cur_last_point = last_point.front();
    vector<int> cur_path = path_to_cell.front();

    // Pop the queue
    cells_to_visit.pop_front();
    path_len.pop_front();
    last_point.pop_front();
    path_to_cell.pop_front();

    // Find next cells
    std::list<int> next_cells;
    for(int neighbor : cells[cur_cell_i].adjacent_cells){
      if(std::find(cur_path.begin(), cur_path.end(), neighbor) == cur_path.end()){
        next_cells.emplace_back(neighbor);
      }
    }
    if(next_cells.empty()){
      for(int i=0; i<cells.size(); i++){
        if(std::find(cur_path.begin(), cur_path.end(), i) == cur_path.end() && i != cur_cell_i){
          next_cells.emplace_back(i);
        }
      }
    }

    // If no next cell was found, then all of them has been visited and we must record the path
    if(next_cells.empty()){
      vector<int> total_path = cur_path;
      total_path.push_back(cur_cell_i);

      total_paths.push_back(total_path);
      total_lens.push_back(cur_path_len);
      continue;
    }

    // How we go to next cells
    std::list<float> d_len;
    std::list<Point2d> finish_points;
    for(int next_cell_i : next_cells){
      int closest_i = findClosest(cells[next_cell_i], cur_last_point);
      if(closest_i < 0){
        d_len.emplace_back(0);
        finish_points.emplace_back(cur_last_point);
      }else{
        d_len.emplace_back(bg::distance(cur_last_point, cells[next_cell_i].paths[closest_i].front()));
        finish_points.emplace_back(cells[next_cell_i].paths[closest_i].back());
      }
    }

    // Add next cells to the queue
    for(int next_cell : next_cells){
      vector<int> new_path = cur_path;
      new_path.push_back(cur_cell_i);

      path_to_cell.emplace_front(new_path);
      cells_to_visit.emplace_front(next_cell);
    }
    for(float dl : d_len){
      path_len.emplace_front(cur_path_len + dl);
    }
    for(Point2d point : finish_points){
      last_point.emplace_front(point);
    }
  }

  // Find optimal path
  int argmin = -1;
  float min_len = std::numeric_limits<float>::max();
  for(int i=0; i<total_lens.size(); i++){
    if(min_len > total_lens[i]){
      min_len = total_lens[i];
      argmin = i;
    }
  }

  if(argmin == -1){
    total_len = -1;
    return {};
  }
  total_len = total_lens[argmin];
  return total_paths[argmin];
}

mrs_msgs::PathSrv MorseDecomposition::generatePath(vector<cell_t>& cells, vector<int>& path, Point2d start){
  mrs_msgs::PathSrv result;
  result.request.path.header.frame_id = polygon_frame_;
  result.request.path.stop_at_waypoints = false;
  result.request.path.use_heading = true;
  result.request.path.fly_now = true;
  result.request.path.loop = false;

  mrs_msgs::Reference start_r;
  start_r.position.x = bg::get<0>(start);
  start_r.position.y = bg::get<1>(start);
  start_r.position.z = height_;
  result.request.path.points.push_back(start_r);

  Point2d last_point = start;

  for(int cell_i : path){
    int closest_i = findClosest(cells[cell_i], last_point);
    for(Point2d& point : cells[cell_i].paths[closest_i]){
      mrs_msgs::Reference ref;
      ref.position.x = bg::get<0>(point);
      ref.position.y = bg::get<1>(point);
      ref.position.z = height_;
      result.request.path.points.push_back(ref);
    }

    if(cells[cell_i].paths[closest_i].size() != 0){
      last_point = cells[cell_i].paths[closest_i].back();
    }
  }

  result.request.path.points = fixPath(result.request.path.points);

  return result;
}

vector<int> MorseDecomposition::getAdjacentCells(vector<cell_t>& cells, int index){
  vector<int> res;
  for(int j=0;j<cells.size(); j++){
    if(j==index){
      continue;
    }

    bool found = false;
    for(Point2d& vertex1 : cells[j].partition){
      for(Point2d& vertex2 : cells[index].partition){
        if(bg::equals(vertex1, vertex2)){
          res.push_back(j);
          found = true;
          break;
        }
      }
      if(found){
        break;
      }
    }
  }
  return res;
}

int MorseDecomposition::findClosest(cell_t& cell, Point2d point){
  int res_i = -1;
  float min_dist = std::numeric_limits<float>::max();
  for(int i=0; i<cell.paths.size(); i++){
    if(cell.paths[i].size() == 0){
      ROS_WARN("[MorseDecomposition]: Cell #%d has empty path #%d", cell.id, i);
      continue;
    }

    float cur_dist = bg::comparable_distance(point, cell.paths[i].front());
    if(cur_dist < min_dist){
      min_dist = cur_dist;
      res_i = i;
    }
  }
  return res_i;
}

  //|------------------------------------------------------------------|
  //|-------------------------- General tools -------------------------|
  //|------------------------------------------------------------------|

MorseDecomposition::point_t MorseDecomposition::getNext(vector<point_t>& border, vector<vector<point_t>>& holes, point_t& cur){
  point_t res;
  if(cur.ring_id == -1){
    res = border[(cur.id + 1) % border.size()];
  } else{
    vector<point_t>& hole = holes[cur.ring_id];
    res = hole[(cur.id + 1) % hole.size()];
  }
  return res;
}

MorseDecomposition::point_t MorseDecomposition::getPrev(vector<point_t>& border, vector<vector<point_t>>& holes, point_t& cur){
  point_t res;
  int index = cur.id - 1;
  if(cur.ring_id == -1){
    if(index < 0){
      index = border.size() - 1;
    }
    res = border[index];
  } else{
    vector<point_t>& hole = holes[cur.ring_id];
    if(index < 0){
      index = hole.size() - 1;
    }
    res = hole[index];
  }
  return res;
}

bool MorseDecomposition::getWaypointPair(Ring& partition, Ogre::Vector3 sweep_dir, std::pair<Point2d, Point2d>& res){
  int found_num = 0;
  Point2d first;
  Point2d second;
  for(int j=0; j<partition.size() - 1; j++){
    Point2d e1 = partition[j];
    Point2d e2 = partition[j+1];
    
    auto point = getIntersection(sweep_dir, Line{e1, e2});
    if(!point){
      continue;
    }

    if(found_num == 0){
      first = point.value();
      found_num ++;
    }else{
      second = point.value();
      found_num ++;
      break;
    }
  }
  if(found_num != 2){
    return false;
  }
  res.first = first;
  res.second = second;
  return true;
}

void MorseDecomposition::pushPoints(Ring& ring, std::vector<MorseDecomposition::point_t>& points){
  for(point_t& p : points){
    ring.push_back(p.point);
  }
}

Ogre::Vector3 MorseDecomposition::toLine(Point2d start, float twist) {
  Ogre::Vector3 line;
  line.x = std::cos(twist);
  line.y = std::sin(twist);
  line.z = - (line.x * bg::get<0>(start) + line.y * bg::get<1>(start)); // ????
  return line;
}

std::optional<Point2d> MorseDecomposition::getIntersection(Ogre::Vector3 line, MorseDecomposition::Line edge){
  Ogre::Vector3 line2;
  line2.x =   (bg::get<1>(edge[1]) - bg::get<1>(edge[0]));
  line2.y =  -(bg::get<0>(edge[1]) - bg::get<0>(edge[0]));
  line2.z = bg::get<1>(edge[0]) * bg::get<0>(edge[1]) - bg::get<1>(edge[1]) * bg::get<0>(edge[0]);

  if(line.x == 0 && line2.x == 0){
    return std::nullopt;
  }
  
  if(line.x == 0){
    Ogre::Vector3 tmp = line;
    line = line2;
    line2 = tmp;
  }

  double a = line.x;
  double b = line.y;
  double c = line.z;
  double v = line2.x;
  double w = line2.y;
  double u = line2.z;

  double y0 = (c*v/a - u) / (w - (b*v/a));
  double x0 = -(b/a)*y0 - (c/a);

  double min_x = bg::get<0>(edge[0]) < bg::get<0>(edge[1]) ? bg::get<0>(edge[0]) : bg::get<0>(edge[1]);
  double min_y = bg::get<1>(edge[0]) < bg::get<1>(edge[1]) ? bg::get<1>(edge[0]) : bg::get<1>(edge[1]);

  double max_x = bg::get<0>(edge[0]) > bg::get<0>(edge[1]) ? bg::get<0>(edge[0]) : bg::get<0>(edge[1]);
  double max_y = bg::get<1>(edge[0]) > bg::get<1>(edge[1]) ? bg::get<1>(edge[0]) : bg::get<1>(edge[1]);

  // Computations may fail due to inaccuracy of float and 
  // double formats, so edge cases are processed separately
  if(bg::get<0>(edge[0]) == bg::get<0>(edge[1]) && y0 <= max_y && y0 >= min_y){
    return Point2d{x0, y0};
  }
  if(bg::get<1>(edge[0]) == bg::get<1>(edge[1]) && x0 <= max_x && x0 >= min_x){
    return Point2d{x0, y0};
  }

  if(y0 > max_y || y0 < min_y){
    return std::nullopt;
  }

  if(x0 <= max_x && x0 >= min_x){
    return Point2d{x0, y0};
  }

  return std::nullopt;
}

} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::MorseDecomposition, mrs_rviz_plugins::CoverageMethod)