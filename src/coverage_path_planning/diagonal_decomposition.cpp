#include <coverage_path_planning/diagonal_decomposition.h>

#include <boost/geometry.hpp>

#include <sstream>
#include <vector>
#include <limits>
#include <cmath>

namespace bg = boost::geometry;

using Polygon = mrs_lib::Polygon;
using Point2d = mrs_lib::Point2d;
using std::vector;

// Note: if points are written in std::vector<point_t>, the last and the first vertices are not equal
// They are equal if points are in Polygon or Ring

namespace mrs_rviz_plugins {

void DiagonalDecomposition::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){

}

void DiagonalDecomposition::compute() {
  Polygon poly = mrs_lib::Polygon();


  // // Polygon with a hole inside (wirks fine)
  // {mrs_lib::Point2d p{0, 0};    bg::append(poly, p);}
  // {mrs_lib::Point2d p{0, 10};   bg::append(poly, p);}
  // {mrs_lib::Point2d p{10, 10};  bg::append(poly, p);}
  // {mrs_lib::Point2d p{10, 0};   bg::append(poly, p);}
  // {mrs_lib::Point2d p{0, 0};    bg::append(poly, p);}

  // bg::interior_rings(poly).resize(1);
  // {mrs_lib::Point2d p{5,   7.5};  bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{7.5, 5};    bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{5,   2.5};  bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{2.5, 5};    bg::append(poly, p, 0);}  
  // {mrs_lib::Point2d p{5,   7.5};  bg::append(poly, p, 0);}

  // // Polygon with crossing diagonal (works fine)
  // {mrs_lib::Point2d p{0, 5};    bg::append(poly, p);}
  // {mrs_lib::Point2d p{3, 3};   bg::append(poly, p);}
  // {mrs_lib::Point2d p{6, 5};  bg::append(poly, p);}
  // {mrs_lib::Point2d p{6, 0};   bg::append(poly, p);}
  // {mrs_lib::Point2d p{0, 0};    bg::append(poly, p);}
  // {mrs_lib::Point2d p{0, 5};    bg::append(poly, p);}

  // bg::interior_rings(poly).resize(1);
  // {mrs_lib::Point2d p{4, 2};    bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{5, 2};    bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{5, 1};    bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{4, 1};    bg::append(poly, p, 0);}
  // {mrs_lib::Point2d p{4, 2};    bg::append(poly, p, 0);}

  // Complex polygon with 2 holes
  {mrs_lib::Point2d p{4.14361, 8.02656};  bg::append(poly, p);}//A
  {mrs_lib::Point2d p{9.27532, 6.44607};  bg::append(poly, p);}//B
  {mrs_lib::Point2d p{9.60703, 7.42168};  bg::append(poly, p);}//C
  {mrs_lib::Point2d p{11.168, 5.56802};   bg::append(poly, p);}//D
  {mrs_lib::Point2d p{11, 3};             bg::append(poly, p);}//E
  {mrs_lib::Point2d p{9.29483, 0.98266};  bg::append(poly, p);}//F
  {mrs_lib::Point2d p{1.29483, 0.63144};  bg::append(poly, p);}//G
  {mrs_lib::Point2d p{0.768, 5.64607};    bg::append(poly, p);}//H
  {mrs_lib::Point2d p{4.14361, 8.02656};  bg::append(poly, p);}//A

  bg::interior_rings(poly).resize(2);

  {mrs_lib::Point2d p{2.62166, 6.58266};  bg::append(poly, p, 0);}//I
  {mrs_lib::Point2d p{3.53873, 6.44607};  bg::append(poly, p, 0);}//J
  {mrs_lib::Point2d p{4.00703, 4.35827};  bg::append(poly, p, 0);}//K
  {mrs_lib::Point2d p{4.33873, 2.40705};  bg::append(poly, p, 0);}//L
  {mrs_lib::Point2d p{5.74361, 3.59729};  bg::append(poly, p, 0);}//M
  {mrs_lib::Point2d p{5.78264, 1.62656};  bg::append(poly, p, 0);}//N
  {mrs_lib::Point2d p{2.60215, 1.19729};  bg::append(poly, p, 0);}//O
  {mrs_lib::Point2d p{1.4119, 3.67534};  bg::append(poly, p, 0);}//P
  {mrs_lib::Point2d p{2.62166, 6.58266};  bg::append(poly, p, 0);}//I

  {mrs_lib::Point2d p{7.38264, 5.19729};  bg::append(poly, p, 1);}//Q
  {mrs_lib::Point2d p{8.88508, 5.19729};  bg::append(poly, p, 1);}//R
  {mrs_lib::Point2d p{9.568, 3.49973};  bg::append(poly, p, 1);}//S
  {mrs_lib::Point2d p{7.07044, 1.86071};  bg::append(poly, p, 1);}//T
  {mrs_lib::Point2d p{7.55825, 3.6168};  bg::append(poly, p, 1);}//U
  {mrs_lib::Point2d p{6.30947, 4.26071};  bg::append(poly, p, 1);}//V
  {mrs_lib::Point2d p{7.38264, 5.19729};  bg::append(poly, p, 1);}//Q



  bg::correct(poly);

  std::string msg;
  std::cout << bg::wkt(poly) << std::endl;
  bg::is_valid(poly, msg);
  std::cout << msg << std::endl;

  current_polygon_ = poly;

  // std::cout << "testing getPartition()\n";
  // vector<point_t> outer_ring;
  // for(int i=0; i<poly.outer().size() - 1; i++){
  //   point_t tmp;
  //   tmp.point = poly.outer()[i];
  //   tmp.ring_index = -1;
  //   tmp.id = i;
  //   outer_ring.push_back(tmp);
  // }

  // vector<point_t> res_poly;
  // line_t res_line;

  // bool res_bool = getPartition(outer_ring, 0, res_poly, res_line);

  // std::cout << (res_bool ? "terminated" : "not terminated") << std::endl;
  // for(point_t& p : res_poly){
  //   std::cout << p.ring_index << " " << p.id << " " << bg::wkt(p.point) << std::endl;
  // }

  // {

  // std::cout << "\ntesting drawTrueDiagonal() crossing\n";
  // line_t diagonal;
  // diagonal.first = outer_ring[3];
  // diagonal.second = outer_ring[7];

  // std::pair<Ring, line_t> res = drawTrueDiagonal(poly, diagonal);
  // std::cout << "hole: " << bg::wkt(res.first) << std::endl;
  // std::cout << res.second.first.ring_index << " " << res.second.first.id << " " << bg::wkt(res.second.first.point) << std::endl;
  // std::cout << res.second.second.ring_index << " " << res.second.second.id << " " << bg::wkt(res.second.second.point) << std::endl;
  // }

  // {
  // std::cout << "\ntesting drawTrueDiagonal() not crossing\n";
  // line_t diagonal;
  // diagonal.first = outer_ring[3];
  // diagonal.second.ring_index = 1;
  // diagonal.second.id = 1;
  // diagonal.second.point = Point2d {8.88508, 5.19729};

  // std::pair<Ring, line_t> res = drawTrueDiagonal(poly, diagonal);
  // std::cout << "hole: " << bg::wkt(res.first) << std::endl;
  // std::cout << res.second.first.ring_index << " " << res.second.first.id << " " << bg::wkt(res.second.first.point) << std::endl;
  // std::cout << res.second.second.ring_index << " " << res.second.second.id << " " << bg::wkt(res.second.second.point) << std::endl;
  // }

  vector<point_t> cur_p;
  for(int i=0; i<current_polygon_.outer().size() - 1; i++){
    point_t tmp;
    tmp.point = current_polygon_.outer()[i];
    tmp.ring_index = -1;
    tmp.id = i;
    cur_p.push_back(tmp);
  }

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

  std::cout << printPolygon(cur_holes[0]) << std::endl;

  int start = 0;
  vector<vector<point_t>> decomposition; // array of polygons
  vector<line_t> diagonals;
  bool terminated = false;
  int it_num =0;
  while(!terminated && it_num<500){
    it_num ++;
    std::cout << std::endl;
    std::cout << "cur_p: " << printPolygon(cur_p) << std::endl;
    
    vector<point_t> partition;
    line_t diagonal;
    terminated = getPartition(cur_p, start, partition, diagonal);
    std::cout << "\tpartition found: " << printPolygon(partition) << std::endl;
    std::cout << "\tdiagonal: " << printPoint(diagonal.first) << " " << printPoint(diagonal.second) << std::endl; 

    // If L has then more than two vertices, and at least one of the
    // vertices of the diagonal joining the last and first vertices in L is a notch, it generates
    // one of the polygons of the partition.
    bool is_notch = false;
    
    // Notch can only be at diagonal
    // Going clockwise to find vertex of diagonal
    point_t prev_vertex;
    point_t cur_vertex = cur_p[start];
    point_t next_vertex;
    for(int i=start+1; i!=start; i = (i+1) % cur_p.size()){
      prev_vertex = cur_vertex;
      cur_vertex = cur_p[i];
      next_vertex = cur_p[i+1];


      if(!equals(cur_vertex, diagonal.second)){
        continue;
      }
      std::cout << "\tclockwise:\n";
      std::cout << "\t" << printPoint(prev_vertex) << printPoint(cur_vertex) << printPoint(next_vertex) << std::endl;
      std::cout <<  "\t" <<ang(prev_vertex, cur_vertex, next_vertex) << std::endl;
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
      int next_i = (i-1) < 0 ? (i-1) : cur_p.size() + i - 1;
      next_vertex = cur_p[next_i];

      if(!equals(cur_vertex, diagonal.first)){
        continue;
      }

      std::cout << "\tcounterclockwise:\n";
      std::cout << "\t" << printPoint(prev_vertex) << printPoint(cur_vertex) << printPoint(next_vertex) << std::endl;
      std::cout <<"\t" <<  ang(next_vertex, cur_vertex, prev_vertex) << std::endl;
      if(ang(next_vertex, cur_vertex, prev_vertex) <= M_PI){
        break;
      }
      is_notch = true;
    }

    std::cout << "\t" << (is_notch ? "Notch found" : "No notch") << std::endl;
    std::cout << "\t" << (terminated ? "terminated" : "Not terminated") << std::endl;
    if((partition.size() <= 2 || !is_notch) && !terminated){
      start = (start + 1) % cur_p.size();
      // std::cout << "\tdiagonal:" <<bg::wkt(diagonal.first) << " " << bg::wkt(diagonal.second) << std::endl;
      // std::cout << "\tnext start: " << start << std::endl;
      continue;
    }


    // -----------------------------------------------------------------
    // |--------------------- AbsHol modification ---------------------|
    std::cout << "\tabsorption modification\n";
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

    if(is_cond_true){
      std::cout << "\tcond_true\n";
    }
    if(is_d_cut_by_hole){
        std::cout << "\td_cut_by_hole\n";
    }

    // if d is cut by a hole or there is a hole inside C
    if(is_cond_true){
      // if d is not cut by a hole
      if(!is_d_cut_by_hole){
        // d <- [v_i, v] hole where v hole is a vertex of one of the holes inside C.
        diagonal.second = getClosestPoint(cur_holes, diagonal.first);
      }

      auto tmp = drawTrueDiagonal(cur_holes, diagonal);
      diagonal = tmp.second;
      // diagonals.push_back(diagonal);

      std::cout << "\ttrue diagonal: " << printPoint(diagonal.first) << " " << printPoint(diagonal.second) << std::endl; 

      // Absorption of H
      vector<point_t> new_border;
      int inserted = 0;
      for(int i=0; i<cur_p.size(); i++){
        point_t& point = cur_p[i];
        point.id = inserted;
        new_border.push_back(point);
        inserted++;

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
      for(int i=0; i<cur_holes.size(); i++){
        if(cur_holes[i].front().ring_index == diagonal.second.ring_index){
          cur_holes.erase(cur_holes.begin() + i);
          continue;
        }
        for(int j=0; j<cur_holes[i].size(); j++){
          cur_holes[i][j].ring_index = i;
        }
      }
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


  std::cout << "\nDECOMPOSED!\n";
  int alsdkfj = 1;
  for(auto& p : decomposition){
    std::cout << alsdkfj << " " << printPolygon(p) << std::endl;
    alsdkfj++;
  }

}

bool DiagonalDecomposition::getPartition(vector<point_t>& border, int index_start, vector<point_t>& res_poly, std::pair<point_t, point_t>& res_line){

  // |---------------- MP3 algorithm ----------------|

  // Initially, the list consists of one vertex.
  res_poly.push_back(border[index_start]);

  bool terminated = getPartitionClockwise(border, index_start, res_poly);
  if(!terminated){
    getPartitionCounterClockwise(border, index_start, res_poly);
  }

  res_line = line_t(res_poly.front(), res_poly.back());

  return terminated;
}

bool DiagonalDecomposition::getPartitionClockwise(const vector<point_t>& border, int index_start, vector<point_t>& res) {
  std::cout<<"\tstarted clockwise"  << std::endl;

  // Note: the points in Polygon are stored in clockwise order
  std::vector<size_t> indices(border.size());
  
  for(size_t i=0; i<indices.size(); ++i){
    int cur = index_start + i;
    indices[i] = cur % indices.size();
  }

  // We add the next consecutive vertex (in clockwise order only) of P
  res.push_back(border[indices[1]]);
  std::cout << "\t\tadded " << bg::wkt(border[indices[1]].point) << std::endl;

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
    std::cout << "\t\tadded " << bg::wkt(border[indices[i+1]].point) << std::endl;
  }

  // If the convex polygon generated is the whole polygon P, the algorithm stops
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
      std::cout << "\t\tpoint within: " << bg::wkt(border[indices[i]].point) << std::endl;
      Point2d v_1 = border[index_start].point;
      Point2d v_k = res.back().point;
      Point2d v = border[indices[i]].point;

      float dist_v_k = signedDistComparable(Line{v_1, v}, v_k); 

      // Starting from 1 in order not to delete the first vertex because of inaccuracy
      for(int j=1; j<res.size();){
        float cur_dist =  signedDistComparable(Line{v_1, v}, res[j].point);
        if((cur_dist > 0 && dist_v_k > 0) || (cur_dist < 0 && dist_v_k < 0)){
          std::cout << "\t\tremoving " << bg::wkt(res[j].point) << std::endl;
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
  std::cout<<"\tstarted counter clockwise"  << std::endl;

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

  // std::cout << "A "<< A.ring_index << " " << A.id << " " << bg::wkt(A.point) << std::endl;
  // std::cout << "B "<< B.ring_index << " " << B.id << " " << bg::wkt(B.point) << std::endl;
  // std::cout << "C "<< C.ring_index << " " << C.id << " " << bg::wkt(C.point) << std::endl;
  // std::cout << "D "<< D.ring_index << " " << D.id << " " << bg::wkt(D.point) << std::endl;
  // std::cout << "G "<< G.ring_index << " " << G.id << " " << bg::wkt(G.point) << std::endl;
  // std::cout << "ang(GAB) " << ang(G, A, B) << std::endl;
  // std::cout << "ang(DGA) " << ang(D, G, A) << std::endl;
  // std::cout << "ang(CDG) " << ang(C, D, G) << std::endl;

  // We add the next consecutive vertex if it is suitable
  if(ang(G, A, B) < M_PI &&
    ang(D, G, A) < M_PI &&
    ang(C, D, G) < M_PI)
  {
    std::cout << "\t\tadded " << bg::wkt(border[indices[1]].point) << std::endl;
    res.insert(res.begin(), border[indices[1]]);
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
    if(ang(border[indices[i+1]], border[indices[i]], border[indices[i-1]]) > M_PI ||
      ang(res.back(), border[indices[i+1]], border[indices[i]]) > M_PI    ||
      ang(res[res.size() - 2], res.back(), border[indices[i+1]]) > M_PI)
    {
      // is_outer_convex = false;
      break;
    }
    std::cout << "\t\tadded " << bg::wkt(border[indices[i+1]].point) << std::endl;
    res.insert(res.begin(), border[indices[i+1]]);
  }


  // The counter-clockwise version is always called after 
  // clockwise, so full polygon can not be generated


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
      std::cout << "\t\tpoint within: " << bg::wkt(border[indices[i]].point) << std::endl;
      Point2d v_1 = res.back().point;
      Point2d v_k = res.front().point;
      Point2d v = border[indices[i]].point;

      float dist_v_k = signedDistComparable(Line{v_1, v}, v_k);

      for(int j=0; j<res.size();){
        float cur_dist =  signedDistComparable(Line{v_1, v}, res[j].point);
        if((cur_dist > 0 && dist_v_k > 0) || (cur_dist < 0 && dist_v_k < 0)){
          std::cout << "\t\tremoving " << bg::wkt(res[j].point) << std::endl;
          res.erase(res.begin() + j);
        }else{
          ++j;
        }
      }
    }
  }
  
  return false;
}

// bool DiagonalDecomposition::getPartitionCounterClockwise(const Polygon& border, int index_start, Polygon& res){
  
// }

// Algorithm 2: Procedure DrawTrueDiagonal
std::pair<DiagonalDecomposition::Ring, DiagonalDecomposition::line_t> DiagonalDecomposition::drawTrueDiagonal(vector<vector<point_t>>& _holes, line_t diagonal){
  vector<Ring> holes;
  for(int i=0; i<_holes.size(); i++){
    Ring tmp;
    for(int j=0; j<_holes[i].size(); j++){
      tmp.push_back(_holes[i][j].point);
    }
    tmp.push_back(_holes[i].front().point);
    holes.push_back(tmp);
  }

  // 1. Read the diagonal and the vertices of partition
  line_t res_line = diagonal;

  while(true){
    // 2. While the diagonal is intersected by the holes, do
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
    res_line.second = endpoint;
  }

  return std::pair<Ring, line_t>(holes[res_line.second.ring_index], res_line);
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

float DiagonalDecomposition::ang(point_t a, point_t b, point_t c){
  return ang(a.point, b.point, c.point);
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