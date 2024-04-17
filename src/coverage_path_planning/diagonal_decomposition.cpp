// Note for whoever decided to change this code (we feel sorry for you):
//
// Point2d, Line, Ring and Polygon are types registered in boost::geometry
//    (viz https://www.boost.org/doc/libs/1_84_0/libs/geometry/doc/html/index.html)
// point_t, line_t and cell_t are types defined in coverage_path_planning/diagonal_decomposition.h
//    for more convenient computations of the algorithm
// Ogre::Vector3 is only used for denoting infinite lines in 2d plane 
//    (vector.x * X + vector.y * Y + vector.z = 0)
//
// If points are written in std::vector<point_t>, the last and the first vertices
//     are not equal. They are equal if points are in Polygon or Ring
// 
// The following code is an implementation of "A practical algorithm for decomposing polygonal
//    domains into convex polygons by diagonals" by Jose Fernandez et al. with further
//    coverage path planning described in "Coverage path planning with unmanned aerial vehicles for 3D terrain
//    reconstruction" by Marina Torres et al.
// Polygon decomposition algorithm is MP3 and can be upgraded to MP4 or MP5 both with merging process.
// The algorithm does not need any start point as it finds the optimal one automatically.
//
// Known issues:
// generatePath() does not add heading to the references
// Final path may exceed safety area.
// getIntersection() may return invalid result due to inaccuracy of float type.

#include "coverage_path_planning/diagonal_decomposition.h"
#include "coverage_path_planning/planner_tool.h"

#include <boost/geometry.hpp>

#include <algorithm>
#include <optional>
#include <sstream>
#include <vector>
#include <limits>
#include <cmath>
#include <set>

// Uncomment this for useful outputs during computation
// #define DEBUG

namespace bg = boost::geometry;

using Polygon = mrs_lib::Polygon;
using Point2d = mrs_lib::Point2d;
using std::vector;

namespace mrs_rviz_plugins {

  //|------------------------------------------------------------------|
  //|-------------------- Public overriden methods --------------------|
  //|------------------------------------------------------------------|

void DiagonalDecomposition::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){
  ExactDecomposition::initialize(property_container, scene_manager, root_node);

  drone_name_property_ = new rviz::EditableEnumProperty("Uav", "", "Uav used to perform coverage mission", property_container);
  cell_num_property_ = new rviz::IntProperty("Cell number", 0, "Number of cells in current decomposition", property_container);
  turn_num_property_ = new rviz::IntProperty("Turn number", 0, "Number of turns in current path", property_container);

  cell_num_property_->setReadOnly(true);
  turn_num_property_->setReadOnly(true);

  std::vector<std::string> drone_names = PlannerTool::getUavNames();
  for(auto& name : drone_names){
    drone_name_property_->addOption(name.c_str());
  }

  if(drone_names.size() > 0){
    drone_name_property_->setString(drone_names[0].c_str());
  }else{
    ROS_WARN("[DiagonalDecomposition]: could not find any uav for coverage mission");
  }
}

DiagonalDecomposition::~DiagonalDecomposition(){
  delete drone_name_property_;
  delete cell_num_property_;
  delete turn_num_property_;
}

void DiagonalDecomposition::start() {
  if(!is_computed_){
    ROS_WARN("[DiagonalDecomposition]: Could not start the mission. The path has not been computed yet.");
    return;
  }
  client_ = nh_.serviceClient<mrs_msgs::PathSrv>("/" + drone_name_property_->getStdString() + "/trajectory_generation/path");

  // Make the call
  if(!client_.call(path_)){
    ROS_INFO("[DiagonalDecomposition]: Call failed. Service name: %s", client_.getService().c_str());
    return;
  }
  if (!path_.response.success) {
    ROS_INFO("[DiagonalDecomposition]: Call failed: %s", path_.response.message.c_str());
    return;
  } 
  ROS_INFO("[DiagonalDecomposition]: Call processed successfully");
}

void DiagonalDecomposition::compute() {
// {
//   std::cout << bg::wkt(current_polygon_) << std::endl;
//   Point2d start{0, 0};
//   Point2d finish{start_position_.x, start_position_.y};
//   std::cout << "start: " << bg::wkt(start) << std::endl;
//   std::cout << "finish: " << bg::wkt(finish) << std::endl;
//   std::cout << "starting bfs\n";
//   vector<Point2d> path = getPath(start, finish);
//   std::cout << "bfs finished\n";
//   mrs_msgs::PathSrv path_to_draw;
//   path_to_draw.request.path.header.frame_id = polygon_frame_;
//   std::cout << "path: \n";
//   for(Point2d& point : path){
//     mrs_msgs::Reference tmp;
//     tmp.position.x = bg::get<0>(point);
//     tmp.position.y = bg::get<1>(point);
//     tmp.position.z = height_;
//     path_to_draw.request.path.points.push_back(tmp);
//     std::cout << "\t" << bg::wkt(point) << std::endl;
//   }
//   std::cout << "drawing path..\n";
//   drawPath(path_to_draw);
//   return;
// }


  std::string msg;
  if(!bg::is_valid(current_polygon_, msg)){
    ROS_WARN("[DiagonalDecomposition]: Current polygon is invalid. Cannot perform the decomposition. Msg: %s", msg.c_str());
    return;
  }
  #ifdef DEBUG
  std::cout << "polygon is valid\n";
  #endif // DEBUG

  // Convert current_polygon_ border from Polygon to vector<point_t>
  vector<point_t> cur_p;
  for(int i=0; i<current_polygon_.outer().size() - 1; i++){
    point_t tmp;
    tmp.point = current_polygon_.outer()[i];
    tmp.ring_index = -1;
    tmp.id = i;
    cur_p.push_back(tmp);
  }

  // Convert holes of current_polygon_ border from Polygon to vector<point_t>
  vector<Ring> holes = bg::interior_rings(current_polygon_);
  vector<vector<point_t>> cur_holes;
  for(int i=0; i<holes.size(); i++){
    vector<point_t> tmp_hole;
    for(int j=0; j<holes[i].size() - 1; j++){
      point_t tmp;
      tmp.point = holes[i][j];
      tmp.ring_index = i;
      tmp.id = j;
      tmp_hole.push_back(tmp);
    }
    cur_holes.push_back(tmp_hole);
  }

  // Decompose
  vector<vector<point_t>> decomposition = decompose(cur_p, cur_holes);
  cell_num_property_->setInt(decomposition.size());

  #ifdef DEBUG
  std::cout << "\nDECOMPOSED!\n";
  int p_index = 1;
  for(auto& p : decomposition){
    std::cout << p_index << " " << printPolygon(p) << std::endl;
    p_index++;
  }
  #endif // DEBUG

  // Convert partitions from vector<point_t> to Ring
  vector<Ring> rings(decomposition.size());
  for(int i=0; i<decomposition.size(); i++){
    for(auto& p : decomposition[i]){
      rings[i].push_back(p.point);
    }
    rings[i].push_back(decomposition[i].front().point);
  }

  // Visualise decomposition
  drawDecomposition(rings);

  // Find the best cell permutation using DFS
  vector<cell_t> cells = fillCells(decomposition);
  fixCells(cells);
  vector<int> shortest_path;
  Point2d best_start;
  float min_total_len = std::numeric_limits<float>::max();
  for(int i=0; i<cells.size(); i++){
    if(cells[i].waypoints.size() == 0){
      continue;
    }

    Point2d drone_point;
    geometry_msgs::Point drone_point_gm;
    drone_point_gm.x = start_position_.x;
    drone_point_gm.y = start_position_.y;
    drone_point_gm.z = start_position_.z;
    const auto& drone_point_trasformed = transformer_.transformSingle(current_frame_, drone_point_gm, polygon_frame_);
    if(!drone_point_trasformed){
      ROS_WARN("[DiagonalDecomposition]: Could not transform start position from %s to %s", current_frame_.c_str(), polygon_frame_.c_str());
    } else{
      drone_point_gm = drone_point_trasformed.value();
    }
    bg::set<0>(drone_point, drone_point_gm.x);
    bg::set<1>(drone_point, drone_point_gm.y);

    vector<int> path(cells.size());
    std::set<int> visited;

    Point2d prev_point = cells[i].waypoints.front().first;
    Point2d start_point, finish_point;
    getStartAndFinish(prev_point, cells[i], start_point, finish_point);
    float total_path_len = 0;
    findPath(cells, drone_point, visited, i, 0, path, total_path_len);
    if(total_path_len < min_total_len){
      shortest_path = path;
      best_start = prev_point;
      min_total_len = total_path_len;
    }

    visited.clear();
    prev_point = cells[i].waypoints.front().second;
    getStartAndFinish(prev_point, cells[i], start_point, finish_point);
    total_path_len = 0;
    findPath(cells, drone_point, visited, i, 0, path, total_path_len);
    if(total_path_len < min_total_len){
      shortest_path = path;
      best_start = prev_point;
      min_total_len = total_path_len;
    }

    visited.clear();
    prev_point = cells[i].waypoints.back().first;
    getStartAndFinish(prev_point, cells[i], start_point, finish_point);
    total_path_len = 0;
    findPath(cells, drone_point, visited, i, 0, path, total_path_len);
    if(total_path_len < min_total_len){
      shortest_path = path;
      best_start = prev_point;
      min_total_len = total_path_len;
    }

    visited.clear();
    prev_point = cells[i].waypoints.back().second;
    getStartAndFinish(prev_point, cells[i], start_point, finish_point);
    total_path_len = 0;
    findPath(cells, drone_point, visited, i, 0, path, total_path_len);
    if(total_path_len < min_total_len){
      shortest_path = path;
      best_start = prev_point;
      min_total_len = total_path_len;
    }
  }

  #ifdef DEBUG
  std::cout << "Generating the path\n";
  #endif // DEBUG
  path_ = generatePath(cells, shortest_path, best_start);
  turn_num_property_->setInt(path_.request.path.points.size() - 1);
  is_computed_ = true;
  
  #ifdef DEBUG
  std::cout << "Drawing the path\n";
  #endif // DEBUG
  drawPath(path_);
}

  //|------------------------------------------------------------------|
  //|------------------- Procedures of MP3 algorithm-------------------|
  //|------------------------------------------------------------------|

vector<vector<DiagonalDecomposition::point_t>> DiagonalDecomposition::decompose(vector<point_t> border, vector<vector<point_t>> holes){
  vector<point_t>& cur_p = border;
  vector<vector<point_t>>& cur_holes = holes;

  int start = 0;
  vector<vector<point_t>> decomposition; // array of polygons
  vector<line_t> diagonals;
  bool terminated = false;
  while(!terminated){
    #ifdef DEBUG
    std::cout << std::endl;
    std::cout << "cur_p: " << printPolygon(cur_p) << std::endl;
    #endif // DEBUG
    
    vector<point_t> partition;
    line_t diagonal;
    terminated = getPartition(cur_p, start, partition, diagonal);

    #ifdef DEBUG
    std::cout << "\tpartition found: " << printPolygon(partition) << std::endl;
    std::cout << "\tdiagonal: " << printPoint(diagonal.first) << " " << printPoint(diagonal.second) << std::endl; 
    #endif // DEBUG

    // -----------------------------------------------------------------
    // |------------ Checking if diagonal connects a notch ------------|
    bool is_notch = false;
    
    // Notch can only be at diagonal
    // Going clockwise to find vertex of diagonal
    point_t prev_vertex;
    point_t cur_vertex = cur_p[start];
    point_t next_vertex;
    for(int i=start+1; i!=start; i = (i+1) % cur_p.size()){
      prev_vertex = cur_vertex;
      cur_vertex = cur_p[i];
      next_vertex = cur_p[(i+1) % cur_p.size()];

      if(!equals(cur_vertex, diagonal.second)){
        continue;
      }

      #ifdef DEBUG
      std::cout << "\tclockwise:\n";
      std::cout << "\t" << printPoint(prev_vertex) << printPoint(cur_vertex) << printPoint(next_vertex) << std::endl;
      std::cout <<  "\t" <<ang(prev_vertex, cur_vertex, next_vertex) << std::endl;
      #endif // DEBUG

      if(ang(prev_vertex, cur_vertex, next_vertex) <= M_PI){
        break;
      }
      is_notch = true;
    }

    // Going counter-clockwise to find vertex of diagonal
    cur_vertex = cur_p[start];
    for(int i=start-1; i!=start; --i){
      if(i<0){
        i = cur_p.size() + i;
      }
      prev_vertex = cur_vertex;
      cur_vertex = cur_p[i];
      int next_i = (i-1) < 0 ? (cur_p.size() + i - 1): (i-1);
      next_vertex = cur_p[next_i];

      if(!equals(cur_vertex, diagonal.first)){
        continue;
      }

      #ifdef DEBUG
      std::cout << "\tcounterclockwise:\n";
      std::cout << "\t" << printPoint(prev_vertex) << printPoint(cur_vertex) << printPoint(next_vertex) << " " << next_i << std::endl;
      std::cout << "\t" <<  ang(next_vertex, cur_vertex, prev_vertex) << std::endl;
      #endif // DEBUG

      if(ang(next_vertex, cur_vertex, prev_vertex) <= M_PI){
        break;
      }
      is_notch = true;
    }
    // |---------- Checking if diagonal connects a notch end ----------|
    // -----------------------------------------------------------------

    // If L has then more than two vertices, and at least one of the
    // vertices of the diagonal joining the last and first vertices in L is a notch, it generates
    // one of the polygons of the partition. Else go to next vertex
    #ifdef DEBUG
    std::cout << "\t" << (is_notch ? "Notch found" : "No notch") << std::endl;
    std::cout << "\t" << (terminated ? "terminated" : "Not terminated") << std::endl;
    #endif // DEBUG
    if((partition.size() <= 2 || !is_notch) && !terminated){
      start = (start + 1) % cur_p.size();
      continue;
    }

    // -----------------------------------------------------------------
    // |--------------------- AbsHol modification ---------------------|
    #ifdef DEBUG
    std::cout << "\tabsorption modification start\n";
    #endif // DEBUG
    bool is_cond_true = false;
    bool is_d_cut_by_hole = false;
    for(auto& hole : cur_holes){
      if(intersects(hole, partition)){
        is_cond_true = true;
      }
      if(intersects(hole, diagonal)){
        is_d_cut_by_hole = true;
      }
    }

    #ifdef DEBUG
    if(is_cond_true){
      std::cout << "\tcond_true\n";
    }
    if(is_d_cut_by_hole){
        std::cout << "\td_cut_by_hole\n";
    }
    #endif // DEBUG

    // if d is cut by a hole or there is a hole inside C
    if(is_cond_true){
      // if d is not cut by a hole
      if(!is_d_cut_by_hole){
        // d <- [v_i, v] hole where v hole is a vertex of one of the holes inside C.

        #ifdef DEBUG
        std::cout << "\td is not cut by hole\n";
        #endif // DEBUG
        diagonal.second = getClosestPoint(cur_holes, diagonal.first);
      }
      #ifdef DEBUG
      else{
        std::cout << "\td is cut by hole\n";
      }
      std::cout << "drawing true diag\n";
      #endif // DEBUG
      diagonal = drawTrueDiagonal(cur_holes, diagonal);

      #ifdef DEBUG
      std::cout << "\ttrue diagonal: " << printPoint(diagonal.first) << " " << printPoint(diagonal.second) << std::endl; 
      std::cout << "\tcurrent holes:\n";
      for(auto& hole : cur_holes){
        std::cout << "\t" << printPolygon(hole) << std::endl;
      }
      #endif // DEBUG

      // Absorption of H
      vector<point_t> new_border;
      int inserted = 0;
      for(int i=0; i<cur_p.size(); i++){
        point_t& point = cur_p[i];
        point.id = inserted;
        new_border.push_back(point);
        inserted++;

        // Check if it's time to insert hole's vertices
        if(!equals(point, diagonal.first)){
          continue;
        }
        
        // absorb the hole
        int hole_index = diagonal.second.ring_index;
        for(int j=0; j<cur_holes[hole_index].size(); j++){
          int cur_index = (j + diagonal.second.id) % cur_holes[hole_index].size();
          point_t cur_point = cur_holes[hole_index][cur_index];
          cur_point.ring_index = -1;
          cur_point.id = inserted;
          new_border.push_back(cur_point);
          inserted++;
        }
        // add first vertex of the hole again
        point_t cur_point = cur_holes[hole_index][diagonal.second.id];
        cur_point.ring_index = -1;
        cur_point.id = inserted;
        new_border.push_back(cur_point);
        inserted++;
        // add diagonal.first again
        cur_point = diagonal.first;
        cur_point.ring_index = -1;
        cur_point.id = inserted;
        new_border.push_back(cur_point);
        inserted++;
      }
      cur_p = new_border;

      // Delete the absorbed hole
      for(int i=0; i<cur_holes.size();){
        if(cur_holes[i].front().ring_index == diagonal.second.ring_index){
          cur_holes.erase(cur_holes.begin() + i);
        }else{
          i++;
        }
      }
      for(int i=0; i<cur_holes.size(); i++){
        for(int j=0; j<cur_holes[i].size(); j++){
          cur_holes[i][j].ring_index = i;
        }
      }

      #ifdef DEBUG
      std::cout << "\tcurrent holes after absorption:\n";
      for(auto& hole : cur_holes){
        std::cout << "\t" << printPolygon(hole) << std::endl;
      }
      #endif // DEBUG
      terminated = false;
      start = 0;
      continue;
    }
    // |------------------- AbsHol modification end -------------------|
    // -----------------------------------------------------------------

    decomposition.push_back(partition);
    diagonals.push_back(diagonal);

    // Delete the vertices from cur_p
    for(int i=1; i<partition.size()-1; i++){
      for(int j=0; j<cur_p.size(); j++){
        if(equals(partition[i], cur_p[j])){
          cur_p.erase(cur_p.begin() + j);
          continue;
        }
      }
    }
    for(int i=0; i<cur_p.size(); i++){
      cur_p[i].id = i;
    }
    start = 0;
  }

  return decomposition;
}

bool DiagonalDecomposition::getPartition(vector<point_t>& border, int index_start, vector<point_t>& res_poly, std::pair<point_t, point_t>& res_line){
  // Initially, the list consists of one vertex.
  res_poly.push_back(border[index_start]);

  // Try to find as big convex partition as possible hoing clockwise
  bool terminated = getPartitionClockwise(border, index_start, res_poly);

  // Algorithm terminates iff. the hole polygon has been generated
  if(!terminated){
    getPartitionCounterClockwise(border, index_start, res_poly);
  }

  res_line = line_t(res_poly.front(), res_poly.back());

  return terminated;
}

bool DiagonalDecomposition::getPartitionClockwise(const vector<point_t>& border, int index_start, vector<point_t>& res) {
  #ifdef DEBUG
  std::cout<<"\tstarted clockwise"  << std::endl;
  #endif // DEBUG

  // Note: the points in Polygon are stored in clockwise order
  std::vector<size_t> indices(border.size());
  
  for(size_t i=0; i<indices.size(); ++i){
    int cur = index_start + i;
    indices[i] = cur % indices.size();
  }

  // We add the next consecutive vertex (in clockwise order only) of P
  res.push_back(border[indices[1]]);
  #ifdef DEBUG
  std::cout << "\t\tadded " << bg::wkt(border[indices[1]].point) << std::endl;
  #endif // DEBUG

  bool is_outer_convex = true;

  // We go on adding new vertices to L until all the vertices of P are in L...
  int last_i = 1;
  for(int i=1; i<indices.size()-1; i++, last_i++){
    // ... or until we first find a vertex failing one of the three conditions.
    // Note: indices[i] is the index of last vertex in res and we consider adding indices[i+1]
    
    // Checking the angles in clockwise order
    if(ang(border[indices[i-1]], border[indices[i]], border[indices[i+1]]) > M_PI ||
      ang(border[indices[i]], border[indices[i+1]], res[0]) > M_PI    ||
      ang(border[indices[i+1]], res[0], res[1]) > M_PI)
    {
      is_outer_convex = false;
      break;
    }
    res.push_back(border[indices[i+1]]);
    #ifdef DEBUG
    std::cout << "\t\tadded " << bg::wkt(border[indices[i+1]].point) << std::endl;
    #endif // DEBUG
  }

  // If the convex polygon generated is the whole polygon P, the algorithm stops (terminates)
  if(is_outer_convex){
    return true;
  }

  if(res.size() <= 2){
    return false;
  }

  // If k > 2, then we have to check whether the convex polygon
  // generated by the diagonal v_k v_1 contains vertices of P \ L.
  for(int i=last_i; i<indices.size(); i++){ // iterating over vertices of P \ L.
    Polygon tmp_complete;
    for(point_t& p : res){
      bg::append(tmp_complete, p.point);
    }
    bg::append(tmp_complete, res[0].point);
    bg::correct(tmp_complete);
    // If a vertex v is found to be in the polygon generated by L, then 
    // we remove from L its last vertex v_k and all the vertices of L in 
    // the half-plane generated by [v_1, v] containing v_k .
    if(bg::within(border[indices[i]].point, tmp_complete)){
      #ifdef DEBUG
      std::cout << "\t\tpoint within: " << bg::wkt(border[indices[i]].point) << std::endl;
      #endif // DEBUG
      Point2d v_1 = border[index_start].point;
      Point2d v_k = res.back().point;
      Point2d v = border[indices[i]].point;

      float dist_v_k = signedDistComparable(Line{v_1, v}, v_k); 

      // Starting from index 1 in order not to delete the first vertex because of inaccuracy
      for(int j=1; j<res.size();){
        float cur_dist =  signedDistComparable(Line{v_1, v}, res[j].point);
        if((cur_dist > 0 && dist_v_k > 0) || (cur_dist < 0 && dist_v_k < 0)){
          #ifdef DEBUG
          std::cout << "\t\tremoving " << bg::wkt(res[j].point) << std::endl;
          #endif // DEBUG
          res.erase(res.begin() + j);
        }else{
          ++j;
        }
      }
    }
  }

  return false;
}

bool DiagonalDecomposition::getPartitionCounterClockwise(const vector<point_t>& border, int index_start, vector<point_t>& res){
  #ifdef DEBUG
  std::cout<<"\tstarted counter clockwise"  << std::endl;
  #endif // DEBUG

  // Note: the points in Polygon are stored in clockwise order
  std::vector<size_t> indices(border.size());
  for(size_t i=0; i<indices.size(); ++i){
    int cur = index_start - i;
    indices[i] = cur < 0 ? indices.size() + cur : cur;
  }

  point_t A = res[0];
  point_t B = res[1];
  point_t C = res[res.size() - 2];
  point_t D = res.back();
  point_t G = border[indices[1]];

  // We add the next consecutive vertex if it is suitable
  if(ang(G, A, B) < M_PI &&
    ang(D, G, A) < M_PI &&
    ang(C, D, G) < M_PI)
  {
    #ifdef DEBUG
    std::cout << "\t\tadded " << bg::wkt(border[indices[1]].point) << std::endl;
    #endif // DEBUG
    res.insert(res.begin(), border[indices[1]]);
  }else{
    // If it does not fit, there is no reason to continue
    return false;
  }

  // We go on adding new vertices to L until all the vertices of P are in L...
  int last_i = 1;
  for(int i=1; i<indices.size()-1; i++, last_i++){
    // ... or until we first find a vertex failing one of the three conditions.
    // Note: indices[i] is the index of last vertex in res and we consider adding indices[i+1]

    // Checking the angles in counter-clockwise order
    if(ang(border[indices[i+1]], border[indices[i]], border[indices[i-1]]) > M_PI ||
      ang(res.back(), border[indices[i+1]], border[indices[i]]) > M_PI    ||
      ang(res[res.size() - 2], res.back(), border[indices[i+1]]) > M_PI)
    {
      // is_outer_convex = false;
      break;
    }
    #ifdef DEBUG
    std::cout << "\t\tadded " << bg::wkt(border[indices[i+1]].point) << std::endl;
    #endif // DEBUG
    res.insert(res.begin(), border[indices[i+1]]);
  }


  // The counter-clockwise version is always called after
  // the clockwise one, so full polygon can not be generated


  // If k > 2, then we have to check whether the convex polygon
  // generated by the diagonal v_k v_1 contains vertices of P \ L.
  if(res.size() <= 2){
    return false;
  }

  for(int i=last_i; i<indices.size(); i++){ // iterating over vertices of P \ L.
    Polygon tmp_complete;
    for(point_t& p : res){
      bg::append(tmp_complete, p.point);
    }
    bg::append(tmp_complete, res[0].point);
    bg::correct(tmp_complete);

    // If a vertex v is found to be in the polygon generated by L, then 
    // we remove from L its last vertex v_k and all the vertices of L in 
    // the half-plane generated by [v_1, v] containing v_k .
    if(bg::within(border[indices[i]].point, tmp_complete)){
      #ifdef DEBUG
      std::cout << "\t\tpoint within: " << bg::wkt(border[indices[i]].point) << std::endl;
      #endif // DEBUG
      Point2d v_1 = res.back().point;
      Point2d v_k = res.front().point;
      Point2d v = border[indices[i]].point;

      float dist_v_k = signedDistComparable(Line{v_1, v}, v_k);

      for(int j=0; j<res.size();){
        float cur_dist =  signedDistComparable(Line{v_1, v}, res[j].point);
        if((cur_dist > 0 && dist_v_k > 0) || (cur_dist < 0 && dist_v_k < 0)){
          #ifdef DEBUG
          std::cout << "\t\tremoving " << bg::wkt(res[j].point) << std::endl;
          #endif // DEBUG
          res.erase(res.begin() + j);
        }else{
          ++j;
        }
      }
    }
  }
  
  return false;
}

// Algorithm 2: Procedure DrawTrueDiagonal
DiagonalDecomposition::line_t DiagonalDecomposition::drawTrueDiagonal(vector<vector<point_t>>& _holes, line_t diagonal){
  #ifdef DEBUG
  std::cout << "\tstarted drawing true diagonal\n";
  #endif // DEBUG
  vector<Ring> holes;
  for(int i=0; i<_holes.size(); i++){
    Ring tmp;
    for(int j=0; j<_holes[i].size(); j++){
      tmp.push_back(_holes[i][j].point);
    }
    tmp.push_back(_holes[i].front().point);
    holes.push_back(tmp);
  }

  #ifdef DEBUG
  std::cout << "\t\tholes.size(): " << holes.size() << std::endl;
  #endif // DEBUG

  // 1. Read the diagonal and the vertices of partition
  line_t res_line = diagonal;

  while(true){
    // 2. While the diagonal is intersected by the holes, do
    #ifdef DEBUG
    std::cout << "\t\tprocessing 2\n";
    #endif // DEBUG
    bool intersects = false;
    int intersected_hole_i = -1;
    for(int i=0; i<holes.size(); i++){
      Line tmp {res_line.first.point, res_line.second.point};
      if(bg::crosses(holes[i], tmp)){
        // std::cout << "crosses: \n";
        // std::cout << bg::wkt(hole) << std::endl;
        // std::cout << bg::wkt(res_line) << std::endl;
        intersects = true;
        intersected_hole_i = i;
        break;
      }
    }
    if(!intersects){
      break;
    }

    // 3. Find all the edges of holes which intersect d, and calculate the
    // corresponding intersection points.
    #ifdef DEBUG
    std::cout << "\t\tprocessing 3\n";
    #endif // DEBUG
    vector<Point2d> intersections;
    vector<line_t> intersected_lines;
    for(int j=0; j<holes.size(); j++){
      auto& hole = holes[j];
      for(int i=0; i<hole.size()-1; ++i){
        Line cur_edge{hole[i], hole[i+1]};
        Line cur_diagonal {res_line.first.point, res_line.second.point};
        vector<Point2d> cur_intersections;  // container for the output
        bg::intersection(cur_edge, cur_diagonal, cur_intersections);
        for(Point2d& intersection : cur_intersections){
          line_t tmp;
          tmp.first.point = cur_edge[0];
          tmp.first.id = i;
          tmp.first.ring_index = j;
          tmp.second = tmp.first;
          tmp.second.point = cur_edge[1];
          intersected_lines.push_back(tmp);
          intersections.push_back(intersection);
        }
      }
    }

    // 4. Find the intersection point closest to diagonal[0], and endpoint
    // of intersected edge closest to diagonal[0]
    #ifdef DEBUG
    std::cout << "\t\tprocessing 4\n";
    #endif // DEBUG
    point_t endpoint;
    size_t hole_index = 0;
    float min = std::numeric_limits<float>::max();
    for(size_t i=0; i<intersections.size(); ++i){
      float tmp = bg::distance(res_line.first.point, intersections[i]);
      if(tmp > min){
        continue;
      }
      min = tmp;
      hole_index = i;
      float d1 = bg::distance(res_line.first.point, intersected_lines[i].first.point);
      float d2 = bg::distance(res_line.first.point, intersected_lines[i].second.point);
      if(d1 < d2){
        endpoint = intersected_lines[i].first;
      }else{
        endpoint = intersected_lines[i].second;
      }
    }

    // 5. Update diagonal
    #ifdef DEBUG
    std::cout << "\t\tprocessing 5\n";
    #endif // DEBUG
    res_line.second = endpoint;
  }

  return res_line;
}

// |---------------------------------------------------------------|
// |---------------- Searching for exhaustive path ----------------|
// |---------------------------------------------------------------|

vector<DiagonalDecomposition::cell_t> DiagonalDecomposition::fillCells(vector<vector<point_t>>& polygons){
  float angle_rad = (((float)angle_) / 180) * M_PI;
  float camera_width = (std::tan(angle_rad / 2) * height_);
  float distance = camera_width * (1 - overlap_);

  vector<cell_t> result;

  for(int i=0; i<polygons.size(); i++){
    cell_t new_cell;
    new_cell.polygon_id = i;
    new_cell.adjacent_polygons = getAdjacentPolygons(polygons, i);

    // Filling waypoints
    Ogre::Vector3 sweep_dir = getSweepDirection(polygons[i]);
    int num = 0;
    for(float c=sweep_dir.z-distance; ; c-=distance){
      sweep_dir.z = c;
      std::pair<Point2d, Point2d> waypoint_pair;
      if(getWaypointPair(polygons[i], sweep_dir, distance, waypoint_pair)){
        new_cell.waypoints.push_back(waypoint_pair);
        num++;
        continue;
      }
      break;
    }

    #ifdef DEBUG
    if(num == 0){
      std::cout << "polygon without any waypoint: " << printPolygon(polygons[i]) << std::endl;
    }
    #endif // DEBUG
    result.push_back(new_cell);
  }

  return result;
}

vector<int> DiagonalDecomposition::getAdjacentPolygons(vector<vector<point_t>>& polygons, int index){
  vector<int> res;
  for(int j=0;j<polygons.size(); j++){
    if(j==index){
      continue;
    }

    bool found = false;
    for(point_t& vertex1 : polygons[j]){
      for(point_t& vertex2 : polygons[index]){
        if(bg::equals(vertex1.point, vertex2.point)){
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

bool DiagonalDecomposition::getWaypointPair(vector<point_t>& polygon, Ogre::Vector3 sweep_dir, float distance, std::pair<Point2d, Point2d>& res){
  int found_num = 0;
  Point2d first;
  Point2d second;
  for(int j=0; j<polygon.size(); j++){
    Point2d e1 = polygon[j].point;
    Point2d e2 = polygon[(j+1)%polygon.size()].point;
    
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

std::optional<Point2d> DiagonalDecomposition::getIntersection(Ogre::Vector3 line, DiagonalDecomposition::Line edge){
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

bool DiagonalDecomposition::findPath(
      vector<DiagonalDecomposition::cell_t>& cells, 
      Point2d prev_point,
      std::set<int> visited, 
      int cur_cell_index, 
      int path_len, 
      vector<int>& path,
      float& total_path_len){
  visited.insert(cur_cell_index);
  path[path_len] = cur_cell_index;

  // Finish condition
  if(visited.size() == cells.size()){
    return true;
  }

  // All the found paths
  vector<vector<int>> paths;
  vector<float> path_total_lens;

  bool found_adjacent = false;

  // Continue search with adjacent cells
  for(int next : cells[cur_cell_index].adjacent_polygons){
    if(visited.find(next) != visited.end()){
      continue;
    }
    cell_t cur_cell = cells[next];

    // Get finish point of cur polygon
    Point2d start_point;
    Point2d finish_point = prev_point;
    float new_total_path_len = total_path_len;
    if(cur_cell.waypoints.size() != 0){
      getStartAndFinish(prev_point, cur_cell, start_point, finish_point);
      new_total_path_len = total_path_len + bg::distance(prev_point, start_point); // getPath() takes too much time
    }

    if(findPath(cells, finish_point, visited, next, path_len+1, path, new_total_path_len)){
      paths.push_back(path);
      path_total_lens.push_back(new_total_path_len);
      found_adjacent = true;
    }
  }

  if(!found_adjacent){
    // Continue search with every unvisited cell
    for(int next=0; next<cells.size(); next++){
      if(visited.find(next) != visited.end()){
        continue;
      }
      cell_t cur_cell = cells[next];

      // Get finish point of cur polygon
      Point2d start_point;
      Point2d finish_point = prev_point;
      float new_total_path_len = total_path_len;
      if(cur_cell.waypoints.size() != 0){
        getStartAndFinish(prev_point, cur_cell, start_point, finish_point);
        new_total_path_len = total_path_len + bg::distance(prev_point, start_point); // getPath() takes too much time
      }

      if(findPath(cells, finish_point, visited, next, path_len+1, path, new_total_path_len)){
        paths.push_back(path);
        path_total_lens.push_back(new_total_path_len);
      }
    }
  }

  int index=0;
  float min_total_path_len = std::numeric_limits<float>::max();
  for(int i=0; i<paths.size(); i++){
    if(path_total_lens[i] < min_total_path_len){
      min_total_path_len = path_total_lens[i];
      index = i;
    }
  }
  path = paths[index];
  total_path_len = min_total_path_len;
  return true;
}

void DiagonalDecomposition::getStartAndFinish(Point2d prev_point, DiagonalDecomposition::cell_t& cur_cell, Point2d& start_point, Point2d& finish_point){
  start_point = cur_cell.waypoints.front().first;
  finish_point = cur_cell.waypoints.size() % 2 == 0 ? cur_cell.waypoints.back().first : cur_cell.waypoints.back().second;
  if(bg::comparable_distance(prev_point, cur_cell.waypoints.front().second) < bg::comparable_distance(prev_point, start_point)){
    start_point = cur_cell.waypoints.front().second;
    finish_point = cur_cell.waypoints.size() % 2 == 0 ? cur_cell.waypoints.back().second : cur_cell.waypoints.back().first;
  }
  if(bg::comparable_distance(prev_point, cur_cell.waypoints.back().first) < bg::comparable_distance(prev_point, start_point)){
    start_point = cur_cell.waypoints.back().first;
    finish_point = cur_cell.waypoints.size() % 2 == 0 ? cur_cell.waypoints.front().first : cur_cell.waypoints.front().second;
  }
  if(bg::comparable_distance(prev_point, cur_cell.waypoints.back().second) < bg::comparable_distance(prev_point, start_point)){
    start_point = cur_cell.waypoints.back().second;
    finish_point = cur_cell.waypoints.size() % 2 == 0 ? cur_cell.waypoints.front().second : cur_cell.waypoints.front().first;
  }
}

void DiagonalDecomposition::fixCells(vector<DiagonalDecomposition::cell_t>& cells){
  for(cell_t& cell : cells){
    Point2d cur_point = cells[0].waypoints[0].first;
    for(std::pair<Point2d, Point2d>& pair : cell.waypoints){
      if(bg::comparable_distance(cur_point, pair.first) > bg::comparable_distance(cur_point, pair.second)){
        Point2d tmp = pair.first;
        pair.first = pair.second;
        pair.second = tmp;
      }
      cur_point = pair.first;
    }
  }
}

// |---------------------------------------------------------------|
// |-------------------- Tools for convenience --------------------|
// |---------------------------------------------------------------|

mrs_msgs::PathSrv DiagonalDecomposition::generatePath(std::vector<cell_t>& cells, std::vector<int> path, Point2d start){
  mrs_msgs::PathSrv result;
  result.request.path.header.frame_id = polygon_frame_;
  result.request.path.stop_at_waypoints = false;
  result.request.path.use_heading = true;
  result.request.path.loop = false;
  mrs_msgs::Reference start_r;
  start_r.position.x = start_position_.x;
  start_r.position.y = start_position_.y;
  start_r.position.z = height_;
  const auto& tr = transformer_.transformSingle(current_frame_, start_r, polygon_frame_);
  if(!tr){
    ROS_WARN("[DiagonalDecomposition]: could not transform start reference from %s to %s", current_frame_.c_str(), polygon_frame_.c_str());
  }else{
    result.request.path.points.push_back(tr.value());
  }

  float angle_rad = (((float)angle_) / 180) * M_PI;
  float camera_width = (std::tan(angle_rad / 2) * height_);
  float distance = camera_width * (1 - overlap_);
  
  Point2d cur_point = start;
  bool is_front;
  bool is_first;
  #ifdef DEBUG
  std::cout << "\nGENERATING PATH\n";
  #endif // DEBUG
  for(int cell_index : path){
    cell_t& cur_cell = cells[cell_index];
    if(cur_cell.waypoints.size() == 0){
      #ifdef DEBUG
      std::cout << "\tfound a cell without waypoints. cell id: " << cur_cell.polygon_id << std::endl;
      #endif // DEBUG
      continue;
    }
    // Find the first waypoint of the cell (getPathLen() takes too much time for)
    Point2d start_point = cur_cell.waypoints.front().first;
    is_front = true;
    is_first = true;
    if(bg::comparable_distance(cur_point, cur_cell.waypoints.front().second) < bg::comparable_distance(cur_point, start_point)){
      start_point = cur_cell.waypoints.front().second;
      is_front = true;
      is_first = false;
    }
    if(bg::comparable_distance(cur_point, cur_cell.waypoints.back().first) < bg::comparable_distance(cur_point, start_point)){
      start_point = cur_cell.waypoints.back().first;
      is_front = false;
      is_first = true;
    }
    if(bg::comparable_distance(cur_point, cur_cell.waypoints.back().second) < bg::comparable_distance(cur_point, start_point)){
      start_point = cur_cell.waypoints.back().second;
      is_front = false;
      is_first = false;
    }

    auto waypoints = cells[cell_index].waypoints;
    if(!is_front){
      std::reverse(waypoints.begin(), waypoints.end());
    }
    for(auto& pair : waypoints){

      Point2d cur_waypoint1;
      Point2d cur_waypoint2;
      if(is_first){
        cur_waypoint1 = pair.first;
        cur_waypoint2 = pair.second;
      }else{
        cur_waypoint1 = pair.second;
        cur_waypoint2 = pair.first;
      }
      if(bg::distance(cur_waypoint1, cur_waypoint2) > 2 * distance){
        mrs_msgs::Reference r1;
        mrs_msgs::Reference r2;

        Line shrinked = shrink(cur_waypoint1, cur_waypoint2, distance);

        // TODO: add heading
        r1.position.x = bg::get<0>(shrinked[0]);
        r1.position.y = bg::get<1>(shrinked[0]);
        r2.position.x = bg::get<0>(shrinked[1]);
        r2.position.y = bg::get<1>(shrinked[1]);
        r1.position.z = height_;
        r2.position.z = height_;

        result.request.path.points.push_back(r1);
        result.request.path.points.push_back(r2);
      }else{
        // TODO: add heading
        mrs_msgs::Reference r;
        r.position.x = (bg::get<0>(cur_waypoint1) + bg::get<0>(cur_waypoint2)) / 2;
        r.position.y = (bg::get<1>(cur_waypoint1) + bg::get<1>(cur_waypoint2)) / 2;
        r.position.z = height_;

        result.request.path.points.push_back(r);
      }
      is_first = !is_first;
    }
    if(result.request.path.points.size() != 0){
      cur_point = Point2d(result.request.path.points.back().position.x, result.request.path.points.back().position.y);
    }
  }

  // todo: if path between 2 points in path does not lie within the polygon, 
  // add points using getPath()

  // todo: erase points that are equal to the next one

  // todo: erase points that lie on the same [infinite] line

  #ifdef DEBUG
  std::cout <<"PATH GENERATED\n";
  #endif // DEBUG
  return result;
}

DiagonalDecomposition::Line DiagonalDecomposition::shrink(Point2d p1, Point2d p2, float dist) {
  float as_vector_x = bg::get<0>(p2) - bg::get<0>(p1);
  float as_vector_y = bg::get<1>(p2) - bg::get<1>(p1);

  float normalizer = std::pow(as_vector_x * as_vector_x + as_vector_y * as_vector_y, 0.5);
  float subtrahend_vector_x = (as_vector_x / normalizer) * dist;
  float subtrahend_vector_y = (as_vector_y / normalizer) * dist;

  float res_p1_x = bg::get<0>(p1) + subtrahend_vector_x;
  float res_p1_y = bg::get<1>(p1) + subtrahend_vector_y;

  float res_p2_x = bg::get<0>(p2) - subtrahend_vector_x;
  float res_p2_y = bg::get<1>(p2) - subtrahend_vector_y;

  Line res;
  res.push_back(Point2d{res_p1_x, res_p1_y});
  res.push_back(Point2d{res_p2_x, res_p2_y});
  return res;
}

double DiagonalDecomposition::ang(Point2d a, Point2d b, Point2d c) {
  bg::subtract_point(a, b);
  bg::subtract_point(c, b);
  Point2d zero{0, 0};

  double cos_a = bg::get<0>(a) / bg::distance(a, zero);
  double cos_c = bg::get<0>(c) / bg::distance(c, zero);

  double a1 = std::acos(cos_a);
  if(bg::get<1>(a) < 0){
    a1 = 2 * M_PI - a1;
  }

  double c1 = std::acos(cos_c);
  if(bg::get<1>(c) < 0){
    c1 = 2 * M_PI - c1;
  }

  return fmod(2 * M_PI - a1 + c1, 2*M_PI);
}

double DiagonalDecomposition::ang(point_t a, point_t b, point_t c){
  return ang(a.point, b.point, c.point);
}

double DiagonalDecomposition::signedDistComparable(Line line, Point2d point) {
  double A =   (bg::get<1>(line[1]) - bg::get<1>(line[0]));
  double B =  -(bg::get<0>(line[1]) - bg::get<0>(line[0]));
  double C = bg::get<1>(line[0]) * bg::get<0>(line[1]) - bg::get<1>(line[1]) * bg::get<0>(line[0]);

  return (A * bg::get<0>(point)) + (B * bg::get<1>(point)) + C;
}

DiagonalDecomposition::point_t DiagonalDecomposition::getClosestPoint(vector<vector<point_t>>& holes, point_t point){
  float min_dist = std::numeric_limits<float>::max();
  point_t res;

  for(auto& hole : holes){
    for(point_t& cur : hole){
      float cur_dist = bg::distance(cur.point, point.point);
      if(cur_dist < min_dist){
        min_dist = cur_dist;
        res = cur;
      }
    }
  }
  return res;
}

bool DiagonalDecomposition::equals(point_t a, point_t b){
  return bg::equals(a.point, b.point) && a.id == b.id && a.ring_index == b.ring_index;
}

bool DiagonalDecomposition::intersects(std::vector<point_t>& ring1, std::vector<point_t>& ring2){
  Ring bg_ring1;
  for(point_t& p : ring1){
    bg_ring1.push_back(p.point);
  }
  bg_ring1.push_back(ring1.front().point);

  Ring bg_ring2;
  for(point_t& p : ring2){
    bg_ring2.push_back(p.point);
  }
  bg_ring2.push_back(ring2.front().point);

  return bg::intersects(bg_ring1, bg_ring2);
}

bool DiagonalDecomposition::intersects(std::vector<point_t>& ring, line_t& line){
  Ring bg_ring1;
  for(point_t& p : ring){
    bg_ring1.push_back(p.point);
  }

  Line bg_line{line.first.point, line.second.point};

  return bg::intersects(bg_ring1, bg_line);
}

std::string DiagonalDecomposition::printPolygon(std::vector<point_t>& poly) {
  if(poly.size() == 0){
    return "the polygon is empty";
  }

  std::stringstream ss;
  int ring_id = poly.front().ring_index;
  std::string error = "";
  ss << "(";
  for(point_t& p : poly){
    ss << bg::get<0>(p.point) << " " << bg::get<1>(p.point) << ", ";
    if(p.ring_index != ring_id){
      error = "ring id is not consistent!";
    }
  }
  ss << ")";
  ss << " ring: " << ring_id << " " << error;
  return ss.str();
}

std::string DiagonalDecomposition::printPoint(point_t& point){
  std::stringstream ss;
  ss << bg::wkt(point.point) << " r:" << point.ring_index << " i:" << point.id << " ";
  return ss.str();
}

Ogre::Vector3 DiagonalDecomposition::getSweepDirection(vector<point_t>& polygon) {
  float optimal_dist = std::numeric_limits<float>::max();
  Ogre::Vector3 line_sweep;
  for(int i=0; i<polygon.size(); i++){
    int next_i = (i+1) % polygon.size();
    Line edge{polygon[i].point, polygon[next_i].point};
    float max_dist_edge = 0;
    Point2d opposed_vertex;
    for(int j=0; j<polygon.size(); j++){
      float distance = std::abs(signedDistComparable(edge, polygon[j].point));
      if(distance > max_dist_edge){
        max_dist_edge = distance;
        opposed_vertex = polygon[j].point;
      }
    }

    if(max_dist_edge < optimal_dist || i==0){
      optimal_dist = max_dist_edge;
      line_sweep.x =   (bg::get<1>(edge[1]) - bg::get<1>(edge[0]));
      line_sweep.y =  -(bg::get<0>(edge[1]) - bg::get<0>(edge[0]));

      line_sweep.z = bg::get<1>(edge[0]) * bg::get<0>(edge[1]) - bg::get<1>(edge[1]) * bg::get<0>(edge[0]);

      float normalizer = std::pow((line_sweep.x * line_sweep.x) + (line_sweep.y * line_sweep.y), 0.5);
      line_sweep = line_sweep / normalizer;
    }
  }
  return line_sweep;
}

} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::DiagonalDecomposition, mrs_rviz_plugins::CoverageMethod)