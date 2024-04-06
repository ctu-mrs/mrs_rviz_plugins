#include <coverage_path_planning/diagonal_decomposition.h>

#include <boost/geometry.hpp>

#include <vector>
#include <limits>

namespace bg = boost::geometry;

using Polygon = mrs_lib::Polygon;
using Point2d = mrs_lib::Point2d;

namespace mrs_rviz_plugins {

void DiagonalDecomposition::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){

}

void DiagonalDecomposition::compute() {
  std::cout << "compute\n";
}

std::pair<mrs_lib::Polygon, DiagonalDecomposition::Line> DiagonalDecomposition::getPartition(mrs_lib::Polygon& border, int index_start) {
  // Todo: implement me
}

// Algorithm 2: Procedure DrawTrueDiagonal
std::pair<mrs_lib::Polygon::ring_type, DiagonalDecomposition::Line> DiagonalDecomposition::drawTrueDiagonal(mrs_lib::Polygon& polygon, Line diagonal){
  auto holes = bg::interior_rings(polygon);

  // 1. Read the diagonal and the vertices of partition
  Line res_line = diagonal;
  Polygon::ring_type res_hole = polygon.outer();

  while(true){
    // 2. While the diagonal is intersected by the holes, do
    bool intersects = false;
    for(auto& hole : holes){
      if(bg::crosses(hole, res_line)){
        // std::cout << "crosses: \n";
        // std::cout << bg::wkt(hole) << std::endl;
        // std::cout << bg::wkt(res_line) << std::endl;
        intersects = true;
        break;
      }
    }
    if(!intersects){
      break;
    }
    
    // 3. Find all the edges of holes which intersect d, and calculate the
    // corresponding intersection points.
    std::vector<Point2d> intersections;
    std::vector<Line> intersected_lines;
    std::vector<Polygon::ring_type> intersected_holes;
    for(auto& hole : holes){
      // Iterate over edges and find intersections with the diagonal
      for(size_t i=0; i<hole.size()-1; ++i){
        Line cur_edge{hole[i], hole[i+1]};
        std::vector<Point2d> cur_intersections;  // container for the output
        bg::intersection(cur_edge, res_line, cur_intersections);
        if(cur_intersections.size()){
          intersections.insert(intersections.end(), cur_intersections.begin(), cur_intersections.end());
          intersected_lines.push_back(cur_edge);
          intersected_holes.push_back(hole);
        }
      }
    }

    // 4. Find the intersection point closest to diagonal[0], and endpoint
    // of intersected edge closest to diagonal[0]
    Point2d endpoint;
    size_t hole_index = 0;
    float min = std::numeric_limits<float>::max();
    for(size_t i=0; i<intersections.size(); ++i){
      float tmp = bg::distance(res_line[0], intersections[i]);
      if(tmp > min){
        continue;
      }
      min = tmp;
      hole_index = i;
      float d1 = bg::distance(res_line[0], intersected_lines[i][0]);
      float d2 = bg::distance(res_line[0], intersected_lines[i][1]);
      if(d1 < d2){
        endpoint = intersected_lines[i][0];
      }else{
        endpoint = intersected_lines[i][1];
      }
    }

    // 5. Update diagonal and corresponding hole
    res_line[1] = endpoint;
    res_hole = intersected_holes[hole_index];
  }

  return std::pair<Polygon::ring_type, Line>(res_hole, res_line);
}

void DiagonalDecomposition::start() {
  std::cout << "start\n";
}

void DiagonalDecomposition::setStart(Ogre::Vector3 position){
  std::cout << "setStart\n";
}

void DiagonalDecomposition::setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update){
  std::cout << "setPolygon\n";
}

void DiagonalDecomposition::setAngle(int angle, bool update){
  std::cout << "setAngle\n";
}

void DiagonalDecomposition::setOverlap(float percentage, bool update){
  std::cout << "setOverlap\n";
}

void DiagonalDecomposition::setHeight(float height, bool update){
  std::cout << "setHeight\n";
}

void DiagonalDecomposition::setFrame(std::string new_frame, bool update){
  std::cout << "setFrame\n";
}

} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::DiagonalDecomposition, mrs_rviz_plugins::CoverageMethod)