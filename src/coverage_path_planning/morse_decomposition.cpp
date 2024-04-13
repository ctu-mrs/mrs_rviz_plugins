#include "coverage_path_planning/morse_decomposition.h"

#include <optional>
#include <cmath>
#include <limits>

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

  Point2d start{6, 2};

  vector<Point2d> cps = getCriticalPoints(start, poly.outer(), 0);
  std::cout << "critical points: \n";
  for(Point2d& cp : cps){
    std::cout << "\t" << bg::wkt(cp) << std::endl; 
  }

  for(Point2d& cp : cps){
    Ogre::Vector3 line = toLine(cp, 0);
    auto res = getEdge(poly, cp, line);
    if(res){
      std::cout << "edge of cp " << bg::wkt(cp) << " is " << bg::wkt(res.value()  ) << std::endl;
    }else{
      std::cout << "edge of cp " << bg::wkt(cp) << " is " << "not found\n";
    }
  }
  return;
}

void MorseDecomposition::start() {
  std::cout << "start\n";
}

  //|------------------------------------------------------------------|
  //|------------------------ Virtual methods -------------------------|
  //|------------------------------------------------------------------|

vector<Point2d> MorseDecomposition::getCriticalPoints(Point2d start, Ring obstacle, float twist) {
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
  vector<Point2d> result;
  float prev = values.back();
  float cur  = values[0];
  float next = values[1];
  for(int i=0; i<values.size(); i++){
    if((cur > prev && cur > next) || (cur < prev && cur < next)){
      result.push_back(obstacle[i]);
      std::cout << "value: " << cur << " point: " << bg::wkt(obstacle[i]) << std::endl;
    }
    prev = cur;
    cur = next;
    next = values[(i+2) % values.size()];
  }

  return result;
}

vector<MorseDecomposition::cell_t> MorseDecomposition::getDecomposition(Polygon polygon, vector<Point2d> crit_points, mrs_lib::Point2d start, float twist) {
  // TODO: implement me!
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


std::optional<MorseDecomposition::Line> MorseDecomposition::getEdge(Polygon& polygon, Point2d crit_point, Ogre::Vector3 line) {
  // 1. Find all the intersections of line with edges of polygon
  vector<Point2d> intersections;
  for(int i=0; i<polygon.outer().size() - 1; i++){
    Line edge;
    edge.push_back(polygon.outer()[i]);
    edge.push_back(polygon.outer()[i+1]);

    // Skip intersections with critical point itself
    if(bg::equals(edge[0], crit_point) || bg::equals(edge[1], crit_point)){
      continue;
    }

    auto intersection = getIntersection(line, edge);
    if(intersection){
      intersections.push_back(intersection.value());
    }
  }

  vector<Ring>& obstacles = bg::interior_rings(polygon);
  for(Ring& obstacle : obstacles){
    for(int i=0; i<obstacle.size() - 1; i++){
      Line edge;
      edge.push_back(obstacle[i]);
      edge.push_back(obstacle[i+1]);

      // Skip intersections with critical point itself
      if(bg::equals(edge[0], crit_point) || bg::equals(edge[1], crit_point)){
        continue;
      }

      auto intersection = getIntersection(line, edge);
      if(intersection){
        intersections.push_back(intersection.value());
      }
    }
  }

  // 2. Compute signed distances from critical point to the intersections
  vector<float> distances(intersections.size());
  Ogre::Vector3 norm_line;
  norm_line.x = line.y;
  norm_line.y = -line.x;
  norm_line.z = - ( norm_line.x * bg::get<0>(crit_point)  + norm_line.y * bg::get<1>(crit_point) );
  for(int i=0; i<intersections.size(); i++){
    distances[i] = signedDistComparable(norm_line, intersections[i]);
  }

  // 3. Find closest intersections on both sides of line
  bool found_negative = false;
  bool found_positive = false;
  float closest_negative_dist = - std::numeric_limits<float>::max();
  float closest_positive_dist = std::numeric_limits<float>::max();
  Point2d closest_negative_point;
  Point2d closest_positive_point;
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

  Line result;
  result.push_back(closest_negative_point);
  result.push_back(closest_positive_point);
  return result;
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
} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::MorseDecomposition, mrs_rviz_plugins::CoverageMethod)