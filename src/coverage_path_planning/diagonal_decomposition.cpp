#include <coverage_path_planning/diagonal_decomposition.h>

#include <boost/geometry.hpp>

#include <vector>
#include <limits>
#include <cmath>

namespace bg = boost::geometry;

using Polygon = mrs_lib::Polygon;
using Point2d = mrs_lib::Point2d;

namespace mrs_rviz_plugins {

void DiagonalDecomposition::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){

}

void DiagonalDecomposition::compute() {
  // outer
  Polygon poly = mrs_lib::Polygon();
  {mrs_lib::Point2d p{15, 10}; bg::append(poly, p);}
  {mrs_lib::Point2d p{15, 0}; bg::append(poly, p);}
  {mrs_lib::Point2d p{5, 0}; bg::append(poly, p);}
  {mrs_lib::Point2d p{5, 20}; bg::append(poly, p);}
  {mrs_lib::Point2d p{30, 20}; bg::append(poly, p);}
  {mrs_lib::Point2d p{30, 0}; bg::append(poly, p);}
  {mrs_lib::Point2d p{15, 0}; bg::append(poly, p);}
  {mrs_lib::Point2d p{15, 10}; bg::append(poly, p);}
  {mrs_lib::Point2d p{25, 10}; bg::append(poly, p);}
  {mrs_lib::Point2d p{25, 15}; bg::append(poly, p);}
  {mrs_lib::Point2d p{15, 15}; bg::append(poly, p);}
  {mrs_lib::Point2d p{15, 10}; bg::append(poly, p);}

  // bg::correct(poly);
  std::string msg;
  bg::is_valid(poly, msg);
  std::cout << msg << std::endl;

  current_polygon_ = poly;

  int start = 0;
  Polygon cur_p = current_polygon_;
  std::vector<Polygon> decomposition;
  std::vector<Line> diagonals;
  bool terminated = false;
  while(!terminated){
    std::cout << "\ncur_polygon: " << bg::wkt(cur_p) << std::endl;
    std::cout << "\tstart point: " << bg::wkt(cur_p.outer()[start]) << std::endl;
    Polygon partition;
    Line diagonal;
    terminated = getPartition(cur_p, start, partition, diagonal);

    if(terminated){
      decomposition.push_back(partition);
      diagonals.push_back(diagonal);
      break;
    }

    std::cout << "found partition polygon: " << bg::wkt(partition) << std::endl;

    // If L has then more than two vertices, and at least one of the
    // vertices of the diagonal joining the last and first vertices in L is a notch, it generates
    // one of the polygons of the partition.
    bool is_notch = false;
    for(int i=1; i<cur_p.outer().size(); i++){
      Point2d cur_vertex = cur_p.outer()[i];
      Point2d prev_vertex = cur_p.outer()[i-1];
      Point2d next_vertex = cur_p.outer()[(i + 1)  % (cur_p.outer().size() - 1)];

      if(ang(prev_vertex, cur_vertex, next_vertex) <= M_PI){
        continue;
      }

      if(!bg::equals(cur_vertex, diagonal[0]) && !bg::equals(cur_vertex, diagonal[1])){
        continue;
      }

      if(!fits(cur_p, i, partition)){
        continue;
      }

      is_notch = true;
      break;
    }

    std::cout << "\t" << (is_notch ? "Notch found" : "No notch") << std::endl;
    if(partition.outer().size() <= 3 || !is_notch){
      start = (start + 1) % cur_p.outer().size();
      std::cout << "\tdiagonal:" <<bg::wkt(diagonal) << std::endl;
      std::cout << "\tnext start: " << start << std::endl;
      continue;
    }

    decomposition.push_back(partition);
    diagonals.push_back(diagonal);

    // Delete the vertices from cur_p
    bool cleaned = false;
    for(int i=0; i<cur_p.outer().size() && !cleaned; i++){
      bool start_cleaning = fits(cur_p, i, partition);

      if(!start_cleaning){
        continue;
      }

      cur_p.outer().erase(cur_p.outer().begin() + cur_p.outer().size() - 1);
      int index = (i+1) % cur_p.outer().size();
      for(int j=1; j<partition.outer().size() - 2; j++){
        std::cout << "\ti: " << i << " index: " << index << std::endl;
        std::cout << "\tdeleting vertices " << bg::wkt(cur_p.outer()[index]) << " " << bg::wkt(partition.outer()[j]) << std::endl;
        cur_p.outer().erase(cur_p.outer().begin() + index);
        index = index % cur_p.outer().size();
      }
      cleaned = true;
    } 
    start = 0;

    bg::append(cur_p, cur_p.outer().front());
    
    std::cout << "after cleanup " << bg::wkt(cur_p)  << cur_p.outer().size() << std::endl;
  }

  std::cout << "\nDECOMPOSED!\n";
  for(auto& p : decomposition){
    std::cout << bg::wkt(p) << std::endl;
  }
}

bool DiagonalDecomposition::getPartition(Polygon& border, int index_start, Polygon& res_poly, Line& res_line) {
  Polygon::ring_type outer = border.outer();

  // the first and last points of ring must be equal, 
  // the following is done for prettier computations
  if(index_start == outer.size()-1){
    index_start = 0; 
  }

  // |---------------- MP3 algorithm ----------------|

  // Initially, the list consists of one vertex.
  Polygon cur_part;
  bg::append(cur_part, outer[index_start]);

  bool terminated = getPartitionClockwise(border, index_start, cur_part);
  if(!terminated){
    getPartitionCounterClockwise(border, index_start, cur_part);
  }

  res_line = Line{cur_part.outer().front(), cur_part.outer().back()};
  bg::append(cur_part, cur_part.outer().front());

  res_poly = cur_part;

  return terminated;
}

bool DiagonalDecomposition::getPartitionClockwise(const Polygon& border, int index_start, Polygon& res){
  std::cout<<"\tstarted " << "clockwise"  << std::endl;
  // Todo: implement me
  Polygon::ring_type outer = border.outer();

  // Note: the points in Polygon are stored in clockwise order
  std::vector<size_t> indices(outer.size()-1);
  
  for(size_t i=0; i<indices.size(); ++i){
    int cur = index_start + i;
    indices[i] = cur % indices.size();
  }

  // We add the next consecutive vertex (in clockwise order only) of P
  bg::append(res, outer[indices[1]]);
  std::cout << "\t\tadded " << bg::wkt(outer[indices[1]]) << std::endl;
  
  bool is_outer_convex = true;

  // We go on adding new vertices to L until all the vertices of P are in L...
  int last_i = 1;
  for(int i=1; i<indices.size()-1; i++, last_i++){
    // ... or until we first find a vertex failing one of the three conditions.
    // Note: indices[i] is the index of last vertex in res and we consider adding indices[i+1]
    
    // Checking the angles in clockwise order
    if(ang(outer[indices[i-1]], outer[indices[i]], outer[indices[i+1]]) > M_PI ||
      ang(outer[indices[i]], outer[indices[i+1]], res.outer()[0]) > M_PI    ||
      ang(outer[indices[i+1]], res.outer()[0], res.outer()[1]) > M_PI)
    {
      is_outer_convex = false;
      break;
    }

    bg::append(res, outer[indices[i+1]]);
    std::cout << "\t\tadded " << bg::wkt(outer[indices[i+1]]) << std::endl;
  }

  // If the convex polygon generated is the whole polygon P, the algorithm stops
  if(is_outer_convex){
    Line res_line = {outer[index_start], outer[index_start]};
    std::cout << "whole convex found: " << bg::wkt(res) << std::endl;
    std::cout << indices.size() << std::endl;
    for(int i=0; i<indices.size()-1; i++){
      std::cout << " " << indices[i];
    }
    std::cout << std::endl;
    for(int i=0; i<indices.size()-1; i++){
      std::cout << " " << bg::wkt(outer[indices[i+1]]);
    }
    return true;
  }

  // If k > 2, then we have to check whether the convex polygon
  // generated by the diagonal v_k v_1 contains vertices of P \ L.
  if(res.outer().size() > 2){

    for(int i=last_i; i<indices.size(); i++){ // iterating over vertices of P \ L.
      Polygon tmp_complete = res;
      bg::append(tmp_complete, res.outer()[0]);
      bg::correct(tmp_complete);
      // If a vertex v is found to be in the polygon generated by L, then 
      // we remove from L its last vertex v_k and all the vertices of L in 
      // the half-plane generated by [v_1, v] containing v_k .
      if(bg::within(outer[indices[i]], tmp_complete)){
        std::cout << "\t\tpoint within: " << bg::wkt(outer[indices[i]]) << std::endl;
        Point2d v_1 = border.outer()[index_start];
        Point2d v_k = res.outer().back();
        Point2d v = outer[indices[i]];

        float dist_v_k = signedDistComparable(Line{v_1, v}, v_k); 

        // Starting from 1 in order not to delete the first vertex because of inaccuracy
        for(int j=1; j<res.outer().size();){
          float cur_dist =  signedDistComparable(Line{v_1, v}, res.outer()[j]);
          if((cur_dist > 0 && dist_v_k > 0) || (cur_dist < 0 && dist_v_k < 0)){
            std::cout << "\t\tremoving " << bg::wkt(res.outer()[j]) << std::endl;
            res.outer().erase(res.outer().begin() + j);
          }else{
            ++j;
          }
        }
      }
    }
  }

  // res = cur_part;
  return false;
}

bool DiagonalDecomposition::getPartitionCounterClockwise(const Polygon& border, int index_start, Polygon& res){
  std::cout<<"\tstarted " << "counter clockwise"  << std::endl;
  Polygon::ring_type outer = border.outer();

  // Note: the points in Polygon are stored in clockwise order
  std::vector<size_t> indices(outer.size()-1);
  for(size_t i=0; i<indices.size(); ++i){
    int cur = index_start - i;
    indices[i] = cur < 0 ? indices.size() + cur : cur;
  }

  Point2d A = res.outer()[0];
  Point2d B = res.outer()[1];
  Point2d C = res.outer()[res.outer().size() - 2];
  Point2d D = res.outer().back();
  Point2d G = outer[indices[1]];

  // We add the next consecutive vertex if it is suitable
  if(ang(G, A, B) < M_PI &&
    ang(D, G, A) < M_PI &&
    ang(C, D, G) < M_PI)
  {
    res.outer().insert(res.outer().begin(), outer[indices[1]]);
  }else{
    // If it does not fit, no reason to continue
    return false;
  }


  // We go on adding new vertices to L until all the vertices of P are in L...
  int last_i = 1;
  for(int i=1; i<indices.size()-1; i++, last_i++){
    // ... or until we first find a vertex failing one of the three conditions.
    // Note: indices[i] is the index of last vertex in res and we consider adding indices[i+1]

    // Checking the angles in counter-clockwise order
    if(ang(outer[indices[i+1]], outer[indices[i]], outer[indices[i-1]]) > M_PI ||
      ang(res.outer().back(), outer[indices[i+1]], outer[indices[i]]) > M_PI    ||
      ang(res.outer()[res.outer().size() - 2], res.outer().back(), outer[indices[i+1]]) > M_PI)
    {
      // is_outer_convex = false;
      break;
    }

    res.outer().insert(res.outer().begin(), outer[indices[i+1]]);
  }

  // If the convex polygon generated is the whole polygon P, the algorithm stops
  // if(is_outer_convex){
  //   Line res_line = {outer[index_start], outer[index_start]};
  //   // res = res;
  //   return true;
  // }

  // If k > 2, then we have to check whether the convex polygon
  // generated by the diagonal v_k v_1 contains vertices of P \ L.
  if(res.outer().size() > 2){

    for(int i=last_i; i<indices.size(); i++){ // iterating over vertices of P \ L.
      Polygon tmp_complete = res;
      bg::append(tmp_complete, res.outer()[0]);
      bg::correct(tmp_complete);
      // If a vertex v is found to be in the polygon generated by L, then 
      // we remove from L its last vertex v_k and all the vertices of L in 
      // the half-plane generated by [v_1, v] containing v_k .
      if(bg::within(outer[indices[i]], tmp_complete)){
        Point2d v_1 = res.outer().back();
        Point2d v_k = res.outer().front();
        Point2d v = outer[indices[i]];

        float dist_v_k = signedDistComparable(Line{v_1, v}, v_k);

        for(int j=0; j<res.outer().size();){
          float cur_dist =  signedDistComparable(Line{v_1, v}, res.outer()[j]);
          if((cur_dist > 0 && dist_v_k > 0) || (cur_dist < 0 && dist_v_k < 0)){
            res.outer().erase(res.outer().begin() + j);
          }else{
            ++j;
          }
        }
      }
    }
  }

  // res = cur_part;
  return false;
}


// Algorithm 2: Procedure DrawTrueDiagonal
std::pair<mrs_lib::Polygon::ring_type, DiagonalDecomposition::Line> DiagonalDecomposition::drawTrueDiagonal(Polygon& polygon, Line diagonal){
  // auto holes = bg::interior_rings(polygon);

  // // 1. Read the diagonal and the vertices of partition
  // Line res_line = diagonal;
  // Polygon::ring_type res_hole = polygon.outer();

  // while(true){
  //   // 2. While the diagonal is intersected by the holes, do
  //   bool intersects = false;
  //   for(auto& hole : holes){
  //     if(bg::crosses(hole, res_line)){
  //       // std::cout << "crosses: \n";
  //       // std::cout << bg::wkt(hole) << std::endl;
  //       // std::cout << bg::wkt(res_line) << std::endl;
  //       intersects = true;
  //       break;
  //     }
  //   }
  //   if(!intersects){
  //     break;
  //   }
    
  //   // 3. Find all the edges of holes which intersect d, and calculate the
  //   // corresponding intersection points.
  //   std::vector<Point2d> intersections;
  //   std::vector<Line> intersected_lines;
  //   std::vector<Polygon::ring_type> intersected_holes;
  //   for(auto& hole : holes){
  //     // Iterate over edges and find intersections with the diagonal
  //     for(size_t i=0; i<hole.size()-1; ++i){
  //       Line cur_edge{hole[i], hole[i+1]};
  //       std::vector<Point2d> cur_intersections;  // container for the output
  //       bg::intersection(cur_edge, res_line, cur_intersections);
  //       if(cur_intersections.size()){
  //         intersections.insert(intersections.end(), cur_intersections.begin(), cur_intersections.end());
  //         intersected_lines.push_back(cur_edge);
  //         intersected_holes.push_back(hole);
  //       }
  //     }
  //   }

  //   // 4. Find the intersection point closest to diagonal[0], and endpoint
  //   // of intersected edge closest to diagonal[0]
  //   Point2d endpoint;
  //   size_t hole_index = 0;
  //   float min = std::numeric_limits<float>::max();
  //   for(size_t i=0; i<intersections.size(); ++i){
  //     float tmp = bg::distance(res_line[0], intersections[i]);
  //     if(tmp > min){
  //       continue;
  //     }
  //     min = tmp;
  //     hole_index = i;
  //     float d1 = bg::distance(res_line[0], intersected_lines[i][0]);
  //     float d2 = bg::distance(res_line[0], intersected_lines[i][1]);
  //     if(d1 < d2){
  //       endpoint = intersected_lines[i][0];
  //     }else{
  //       endpoint = intersected_lines[i][1];
  //     }
  //   }

  //   // 5. Update diagonal and corresponding hole
  //   res_line[1] = endpoint;
  //   res_hole = intersected_holes[hole_index];
  // }

  // return std::pair<Polygon::ring_type, Line>(res_hole, res_line);

  Polygon::ring_type res_hole;
  Line res_line;
  return std::pair<Polygon::ring_type, Line>(res_hole, res_line);
}

float DiagonalDecomposition::ang(Point2d a, Point2d b, Point2d c) {
  bg::subtract_point(a, b);
  bg::subtract_point(c, b);
  Point2d zero{0, 0};

  float cos_a = bg::get<0>(a) / bg::distance(a, zero);
  float cos_c = bg::get<0>(c) / bg::distance(c, zero);

  float a1 = std::acos(cos_a);
  if(bg::get<1>(a) < 0){
    a1 = 2 * M_PI - a1;
  }

  float c1 = std::acos(cos_c);
  if(bg::get<1>(c) < 0){
    c1 = 2 * M_PI - c1;
  }

  return fmod(2 * M_PI - a1 + c1, 2*M_PI);
}

float DiagonalDecomposition::signedDistComparable(Line line, Point2d point) {
  float A =   (bg::get<1>(line[1]) - bg::get<1>(line[0]));
  float B =  -(bg::get<0>(line[1]) - bg::get<0>(line[0]));
  float C = bg::get<1>(line[0]) * bg::get<0>(line[1]) - bg::get<1>(line[1]) * bg::get<0>(line[0]);

  return (A * bg::get<0>(point)) + (B * bg::get<1>(point)) + C;
}

bool DiagonalDecomposition::fits(Polygon& main, int start, Polygon& part){
  for(int j=0; j<part.outer().size()-1; j++){
    int index = (start + j) % (main.outer().size()-1);
    if(!bg::equals(main.outer()[index], part.outer()[j])){
      return false;
    }
  }
  return true;
}


void DiagonalDecomposition::start() {
  std::cout << "start\n";
}

void DiagonalDecomposition::setStart(Ogre::Vector3 position){
  std::cout << "setStart\n";
}

void DiagonalDecomposition::setPolygon(std::string frame_id, Polygon &new_polygon, bool update){
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