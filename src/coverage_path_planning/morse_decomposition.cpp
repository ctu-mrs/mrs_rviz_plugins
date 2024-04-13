#include "coverage_path_planning/morse_decomposition.h"

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
}

void MorseDecomposition::start() {
  std::cout << "start\n";
}

  //|------------------------------------------------------------------|
  //|------------------------ Virtual methods -------------------------|
  //|------------------------------------------------------------------|

vector<Point2d> MorseDecomposition::getCriticalPoints(Point2d start, Ring obstacle, float twist) {
  // 1. Compute parameters of infinite line (the slice)
  Ogre::Vector3 line;
  line.x = std::cos(twist);
  line.y = std::sin(twist);
  line.z = - (line.x * bg::get<0>(start) + line.y * bg::get<1>(start)); // ????

  // 2. Compute values of the (linear) function at each vertex
  vector<float> values(obstacle.size() - 1);
  for(int i=0; i<obstacle.size() - 1; i++){
    values[i] = line.x * bg::get<0>(obstacle[i]) + line.y * bg::get<1>(obstacle[i]) + line.z; // Note: line.z seems to be redundant since we compare all the values afterwards
  }

  // 3. Find values that are greater than previous and next ones.
  //    Points of these values are critical ones.
  vector<Point2d> result;
  float prev = values.back();
  float cur  = values[0];
  float next = values[1];
  for(int i=0; i<values.size(); i++){
    if((cur > prev && cur > next) || (cur < prev && cur < next)){
      result.push_back(obstacle[i]);
    }
    prev = cur;
    cur = next;
    next = values[(i+1) % values.size()];
  }

  return result;
}


} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::MorseDecomposition, mrs_rviz_plugins::CoverageMethod)