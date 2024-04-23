#include "coverage_path_planning/morse_decomposition.h"

#include <algorithm>
#include <optional>
#include <numeric>
#include <limits>
#include <cmath>

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
}

void MorseDecomposition::compute() {
  std::cout << "compute\n";

  Polygon poly;

  {mrs_lib::Point2d p{0, 2}; bg::append(poly, p);}
  {mrs_lib::Point2d p{4, 3}; bg::append(poly, p);}
  {mrs_lib::Point2d p{2, 4}; bg::append(poly, p);}
  {mrs_lib::Point2d p{5, 5}; bg::append(poly, p);}
  {mrs_lib::Point2d p{7, 5}; bg::append(poly, p);}
  {mrs_lib::Point2d p{8, 2}; bg::append(poly, p);}
  {mrs_lib::Point2d p{7, 0}; bg::append(poly, p);}
  {mrs_lib::Point2d p{0, 2}; bg::append(poly, p);}

  bg::interior_rings(poly).resize(2);

  {mrs_lib::Point2d p{2, 2}; bg::append(poly, p, 0);}
  {mrs_lib::Point2d p{3, 2.38}; bg::append(poly, p, 0);}
  {mrs_lib::Point2d p{3.68, 2}; bg::append(poly, p, 0);}
  {mrs_lib::Point2d p{3, 1.62}; bg::append(poly, p, 0);}
  {mrs_lib::Point2d p{2, 2}; bg::append(poly, p, 0);}


  {mrs_lib::Point2d p{5.46326, 3.99254}; bg::append(poly, p, 1);}
  {mrs_lib::Point2d p{6.64386, 2.79007}; bg::append(poly, p, 1);}
  {mrs_lib::Point2d p{5.57257, 1.47828}; bg::append(poly, p, 1);}
  {mrs_lib::Point2d p{6.00983, 2.72448}; bg::append(poly, p, 1);}
  {mrs_lib::Point2d p{5.46326, 3.99254}; bg::append(poly, p, 1);}

  std::string msg;
  if (!bg::is_valid(poly, msg)){
    std::cout << "poly is not valid: " << msg << std::endl;
  }
  bg::correct(poly);
  bg::is_valid(poly, msg);
  std::cout << "after correcting: " << msg << std::endl;

  bg::is_valid(poly, msg);
  std::cout << "after reversing holes" << msg << std::endl;

  Point2d start{6, 2};

  vector<point_t> cps = getCriticalPoints(start, poly.outer(), 0, -1);
  vector<Ring> holes = bg::interior_rings(poly); 
  for(int i=0; i<holes.size(); i++){
    vector<point_t> tmp = getCriticalPoints(start, holes[i], 0, i);
    cps.insert(cps.end(), tmp.begin(), tmp.end());
  }

  auto asldkfj = getDecomposition(poly, cps, start, 0);
  return;
}

void MorseDecomposition::start() {
  std::cout << "start\n";
}

  //|------------------------------------------------------------------|
  //|------------------------ Virtual methods -------------------------|
  //|------------------------------------------------------------------|

vector<MorseDecomposition::point_t> MorseDecomposition::getCriticalPoints(Point2d start, Ring obstacle, float twist, int ring_id) {
  // 1. Compute parameters of infinite line (the slice)
  Ogre::Vector3 line = toLine(start, twist);
  std::cout << "line.x:" << line.x << " y:" << line.y << " z:" << line.z << std::endl;

  // 2. Compute values of the (linear) function at each vertex
  vector<float> values(obstacle.size() - 1);
  for(int i=0; i<obstacle.size() - 1; i++){
    values[i] = line.x * bg::get<0>(obstacle[i]) + line.y * bg::get<1>(obstacle[i]) + line.z; // Note: line.z seems to be redundant since we compare all the values afterwards
    std::cout << values[i] << " ";
  }
  std::cout << std::endl;

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
      result.push_back(crit_point);
      std::cout << "value: " << cur << " point: " << bg::wkt(crit_point.point) << std::endl;
    }
    prev = cur;
    cur = next;
    next = values[(i+2) % values.size()];
  }

  return result;
}

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

vector<MorseDecomposition::cell_t> MorseDecomposition::getDecomposition(Polygon& polygon, vector<MorseDecomposition::point_t>& crit_points, mrs_lib::Point2d start, float twist) {
  // TODO: implement me!
  
  // 1. Find all the edges generated by critical points
  vector<std::optional<edge_t>> edges;
  for(point_t& cp : crit_points){
    Ogre::Vector3 line = toLine(cp.point, 0);
    edges.push_back(getEdge(polygon, cp, line));
  }
  std::cout << "edges computed\n";

  // 2. Transform polygon into more convenient format
  vector<point_t> cur_border(polygon.outer().size()-1);
  for(int i=0; i<polygon.outer().size(); i++){
    cur_border[i].id = i;
    cur_border[i].ring_id = -1;
    cur_border[i].point = polygon.outer()[i];
  }
  std::cout << "border transformed\n";
  vector<Ring> holes = bg::interior_rings(polygon);
  vector<vector<point_t>> cur_holes(holes.size());
  for(int i=0; i<holes.size(); i++){
    vector<point_t> new_hole(holes[i].size() - 1);
    for(int j=0; j<new_hole.size(); j++){
      // filling from the end to define the hole in clockwise order
      int cur_index = new_hole.size() - j;
      new_hole[cur_index].id = cur_index;
      new_hole[cur_index].ring_id = i;
      new_hole[cur_index].point = holes[i][cur_index];
    }
    cur_holes[i] = new_hole;
  }
  std::cout << "holes transformed\n";

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
        std::cout << "\t" << bg::wkt(new_edge.p1.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
      }

      if(new_edge.p2.prev_point == i && new_edge.p2.ring_id == -1){
        new_points.push_back(new_edge.p2);
        distances.push_back(bg::comparable_distance(cur_edge[0], new_edge.p2.point));
        std::cout << "\t" << bg::wkt(new_edge.p2.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
      }
    }

    for(size_t k : sort_indexes(distances)){
      new_border.push_back(new_points[k]);
    }
  }
  cur_border = new_border;
  std::cout << "points inserted into border\n";

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
          std::cout << "\t" << bg::wkt(new_edge.p1.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
        }

        if(new_edge.p2.prev_point == j && new_edge.p2.ring_id == i){
          new_points.push_back(new_edge.p2);
          distances.push_back(bg::comparable_distance(cur_edge[0], new_edge.p2.point));
          std::cout << "\t" << bg::wkt(new_edge.p2.point) << " is covered by " << bg::wkt(cur_edge) << std::endl;
        }
      }

      for(size_t k : sort_indexes(distances)){
        new_hole.push_back(new_points[k]);
      }
    }
    new_holes[i] = new_hole;
  }
  cur_holes = new_holes;

  std::cout << "points inserted into holes\n";

  // 5. Update data of points in cur_border and cur_holes
  std::cout << "border: \n";
  for(int i=0; i<cur_border.size(); i++){
    cur_border[i].id = i;
    cur_border[i].ring_id = -1;
    std::cout << "\t" << bg::wkt(cur_border[i].point) << " " << cur_border[i].id << " " << (cur_border[i].is_new_edge ? "is new edge" : "")<< std::endl;
  }
  std::cout << "holes:\n";
  for(int i=0; i<cur_holes.size(); i++){
    for(int j=0; j<cur_holes[i].size(); j++){
      cur_holes[i][j].ring_id = i;
      cur_holes[i][j].id = j;
      std::cout << "\t" << bg::wkt(cur_holes[i][j].point) << " ring: " << i << " " << cur_holes[i][j].id << " " << (cur_holes[i][j].is_new_edge ? "is new edge" : "")<< std::endl;
    }
    std::cout << std::endl;
  }

  // // 6. Devide the polygon into partitions
  // vector<cell_t> decomposition;
  // vector<bool> big_used(crit_points.size(), false);
  // vector<bool> first_used(crit_points.size(), false);
  // vector<bool> second_used(crit_points.size(), false);
  // for(int i=0; i<crit_points.size(); i++){
  //   if(big_used[i] && first_used[i] && second_used[i]){
  //     continue;
  //   }

  //   if(!edges[i]){
  //     big_used[i] = true;
  //     first_used[i] = true;
  //     second_used[i] = true;
  //   }


  // }


  std::cout << "exiting the function\n";
  return {};
}

  //|------------------------------------------------------------------|
  //|----------------------------- Tools ------------------------------|
  //|------------------------------------------------------------------|

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


double MorseDecomposition::signedDistComparable(Line line, Point2d point) {
  double A =   (bg::get<1>(line[1]) - bg::get<1>(line[0]));
  double B =  -(bg::get<0>(line[1]) - bg::get<0>(line[0]));
  double C = bg::get<1>(line[0]) * bg::get<0>(line[1]) - bg::get<1>(line[1]) * bg::get<0>(line[0]);

  return (A * bg::get<0>(point)) + (B * bg::get<1>(point)) + C;
}

double MorseDecomposition::signedDistComparable(Ogre::Vector3 line, Point2d point) {
  return (line.x * bg::get<0>(point)) + (line.y * bg::get<1>(point)) + line.z;
}


  //|------------------------------------------------------------------|
  //|---------------------------- Private -----------------------------|
  //|------------------------------------------------------------------|

std::optional<MorseDecomposition::edge_t> MorseDecomposition::getEdge(Polygon& polygon, point_t crit_point, Ogre::Vector3 line) {
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
      // std::cout << "intersection found: " << bg::wkt(tmp.point) << " after point #" << tmp.prev_point << std::endl;
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
        // std::cout << "intersection found: " << bg::wkt(tmp.point) << " after point #" << tmp.prev_point << std::endl;
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
    printf("edge (%.7f %.7f)  (%.7f %.7f) is not covered by the polygon\n", bg::get<0>(result.p1.point), bg::get<1>(result.p1.point), bg::get<0>(result.p2.point), bg::get<1>(result.p2.point));
    return std::nullopt;
  }
  printf("edge (%.7f %.7f)  (%.7f %.7f) is covered by the polygon\n", bg::get<0>(result.p1.point), bg::get<1>(result.p1.point), bg::get<0>(result.p2.point), bg::get<1>(result.p2.point));

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
  std::cout << "\t" << (result.follow_cp ? "follows cp" : "does not follow cp") << std::endl;

  return result;
}
} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::MorseDecomposition, mrs_rviz_plugins::CoverageMethod)