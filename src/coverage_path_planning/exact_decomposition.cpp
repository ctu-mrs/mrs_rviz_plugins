#include <coverage_path_planning/exact_decomposition.h>

// TODO: include polygon only
// #include <mrs_lib/safety_zone/polygon.h>
#include <mrs_lib/safety_zone/prism.h>

#include <rviz/ogre_helpers/line.h>

#include <limits>
#include <cmath>

namespace bg = boost::geometry;

namespace mrs_rviz_plugins{

void ExactDecomposition::drawCurrentPolygon(){
  if(boundaries_node_){
    scene_manager_->destroySceneNode(boundaries_node_);
  }
  boundaries_node_ = root_node_->createChildSceneNode();

  const auto& tf = transformer_.getTransform(polygon_frame_, current_frame_);
  if (!tf) {
    ROS_INFO("[ExactDecomposition]: Transformation is not found. boundaries will not be displayed");
    return;
  }
  drawRing(current_polygon_.outer(), tf.value(), boundaries_node_);
  auto& obstacles = bg::interior_rings(current_polygon_);
  for(auto& obstacle : obstacles){
    drawRing(obstacle, tf.value(), boundaries_node_);
  }

  boundaries_node_->setVisible(boundaries_property_->getBool());
}

void ExactDecomposition::drawRing(mrs_lib::Polygon::ring_type& ring, geometry_msgs::TransformStamped tf, Ogre::SceneNode* node){
  for(int i=0; i<ring.size()-1; i++){
    mrs_lib::Point2d tmp_start = ring[i];
    mrs_lib::Point2d tmp_end = ring[i+1];

    geometry_msgs::Point start;
    start.x = bg::get<0>(tmp_start);
    start.y = bg::get<1>(tmp_start);
    start.z = height_;

    geometry_msgs::Point end;
    end.x = bg::get<0>(tmp_end);
    end.y = bg::get<1>(tmp_end);
    end.z = height_;
    // Transform points to current frame
    const auto& start_transformed = transformer_.transform(start, tf);
    const auto& end_transformed = transformer_.transform(end, tf);
    if (!start_transformed || !end_transformed) {
      ROS_INFO("[ExactDecomposition]: Unable to transform cmd reference from %s to %s at time %.6f.", current_frame_.c_str(), polygon_frame_.c_str(),
              ros::Time::now().toSec());
      continue;
    }
    start = start_transformed.value();
    end = end_transformed.value();

    rviz::Line* line = new rviz::Line(scene_manager_, node);
    line->setColor(1, 0, 0, 1);
    line->setPoints(Ogre::Vector3(start.x, start.y, start.z), Ogre::Vector3(end.x, end.y, end.z));
    line->setPosition(Ogre::Vector3(0, 0, 0));
    line->setScale(Ogre::Vector3(1, 1, 1));
    line->setVisible(true);
  }
}

void ExactDecomposition::drawDecomposition(std::vector<mrs_lib::Polygon::ring_type>& polygons){
  if(decomposition_node_){
    scene_manager_->destroySceneNode(decomposition_node_);
  }
  decomposition_node_ = root_node_->createChildSceneNode();

  const auto& tf = transformer_.getTransform(polygon_frame_, current_frame_);
  if (!tf) {
    ROS_INFO("[ExactDecomposition]: Transformation is not found. boundaries will not be displayed");
    return;
  }
  for(auto& polygon : polygons){
    drawRing(polygon, tf.value(), decomposition_node_);
  }
  decomposition_node_->setVisible(decomposition_property_->getBool());
}

void ExactDecomposition::drawPath(mrs_msgs::PathSrv& path){
  // Delete previous path
  if(path_node_){
    scene_manager_->destroySceneNode(path_node_);
  }
  path_node_ = root_node_->createChildSceneNode();

  // Prepare transformation
  const auto& tf = transformer_.getTransform(path.request.path.header.frame_id, current_frame_);
  if (!tf) {
    ROS_INFO("[ExactDecomposition]: Transformation is not found. Path will not be displayed");
    return;
  }

  for(size_t i=0; i<path.request.path.points.size()-1; ++i){
    geometry_msgs::Point start = path.request.path.points[i].position;
    geometry_msgs::Point end = path.request.path.points[i+1].position;

    // Transform points to current frame
    const auto& start_transformed = transformer_.transform(start, tf.value());
    const auto& end_transformed = transformer_.transform(end, tf.value());
    if (!start_transformed || !end_transformed) {
      ROS_INFO("[ExactDecomposition]: Unable to transform cmd reference from %s to %s at time %.6f.", path.request.path.header.frame_id.c_str(), current_frame_.c_str(),
              ros::Time::now().toSec());
      continue;
    }
    start = start_transformed.value();
    end = end_transformed.value();

    rviz::Line* line = new rviz::Line(scene_manager_, path_node_);
    line->setColor(1, 0, 0, 1);
    line->setPoints(Ogre::Vector3(start.x, start.y, start.z), Ogre::Vector3(end.x, end.y, end.z));
    line->setPosition(Ogre::Vector3(0, 0, 0));
    line->setScale(Ogre::Vector3(1, 1, 1));
    line->setVisible(true);
  }
  path_node_->setVisible(path_property_->getBool());
}

std::vector<mrs_lib::Point2d> ExactDecomposition::getPath(mrs_lib::Point2d p1, mrs_lib::Point2d p2){
  using Point2d = mrs_lib::Point2d;
  // If the path is straight line
  bg::model::linestring<Point2d> line;
  line.push_back(p1);
  line.push_back(p2);
  if(bg::within(line, current_polygon_)){
    return {p1, p2};
  }
  
  float max_x = std::numeric_limits<float>::lowest();
  float min_x = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  for(Point2d& point : current_polygon_.outer()){
    float point_x = bg::get<0>(point);
    float point_y = bg::get<1>(point);
    max_x = max_x < point_x ? point_x : max_x;
    min_x = min_x > point_x ? point_x : min_x;
    max_y = max_y < point_y ? point_y : max_y;
    min_y = min_y > point_y ? point_y : min_y;
  }

  // Define possible movements: up, down, left, right, diagonally
  int dx[] = {-1, 1, 0, 0, 1, 1, -1, -1};
  int dy[] = {0, 0, -1, 1, 1, -1, -1, 1};


  int numRows = std::ceil(max_x - min_x);
  int numCols = std::ceil(max_y - min_y);
  Ogre::Vector2 start_pos;
  start_pos.x = std::ceil(bg::get<0>(p1) - min_x) - 1;
  start_pos.y = std::ceil(bg::get<1>(p1) - min_y) - 1;

  if(start_pos.x < 0 || start_pos.x >= numRows || start_pos.y < 0 || start_pos.y >= numCols){
    return {};
  }

  // Create a queue for BFS
  std::queue<std::pair<Ogre::Vector2, std::vector<Point2d>>> q;

  // Mark the start cell as visited and enqueue it with an empty path
  std::vector<std::vector<bool>> visited(numRows, std::vector<bool>(numCols, false));
  visited[start_pos.x][start_pos.y] = true;
  q.push({start_pos, {p1}});

  // Perform BFS
  while(!q.empty()) {
    Ogre::Vector2 currCell = q.front().first;
    std::vector<Point2d> path = q.front().second;
    q.pop();
    Point2d cur_point = path.back();

    // Check if p2 point can be reached from the cur_point
    line[0] = cur_point;
    if(bg::within(line, current_polygon_)){
      path.push_back(cur_point);
      path.push_back(p2);
      return path;
    }

    // Explore adjacent cells
    for (int i = 0; i < 8; i++) {
      int newRow = currCell.x + dx[i];
      int newCol = currCell.y + dy[i];

      Point2d new_point{bg::get<0>(cur_point) + dx[i], bg::get<1>(cur_point) + dy[i]};

      bg::model::linestring<Point2d> step;
      step.push_back(cur_point);
      step.push_back(new_point);

      bool is_valid = true;
      is_valid = is_valid && (newRow > 0) && (newRow < numRows);
      is_valid = is_valid && (newCol > 0) && (newCol < numCols);
      is_valid = is_valid && bg::within(step, current_polygon_);
      if(is_valid){
        is_valid = is_valid && !visited[newRow][newCol];
      }


      // Check if the new cell is valid and not visited
      if (is_valid) {
        // std::cout << "step " << bg::wkt(step) << " is " << (bg::within(step, current_polygon_) ? "" : "not ") << "within the polygon\n";
        // Mark the new cell as visited
        visited[newRow][newCol] = true;
        // Enqueue the new cell with the current path plus the new cell
        std::vector<Point2d> newPath = path;
        newPath.push_back(new_point);
        q.push({{newRow, newCol}, newPath});
      }
    }
  }

  return {};
}

std::vector<mrs_msgs::Reference> ExactDecomposition::fixPath(std::vector<mrs_msgs::Reference>& path){
  std::vector<mrs_msgs::Reference> updated_points;

  // Make points lie within the current_polygon_
  for(int i=0; i<path.size()-1; i++){
    updated_points.push_back(path[i]);
    mrs_lib::Point2d p1 {path[i].position.x, path[i].position.y};
    mrs_lib::Point2d p2 {path[i+1].position.x, path[i+1].position.y};
    
    boost::geometry::model::linestring<mrs_lib::Point2d> line;
    line.push_back(p1);
    line.push_back(p2);
    if(bg::within(line, current_polygon_)){
      continue;
    }

    auto new_partial_path = getPath(p1, p2);
    std::cout << "partial path len = " << new_partial_path.size() << std::endl;
    for(mrs_lib::Point2d& new_p : new_partial_path){
      mrs_msgs::Reference new_r;
      new_r.position.x = bg::get<0>(new_p);
      new_r.position.y = bg::get<1>(new_p);
      new_r.position.z = height_;
      updated_points.push_back(new_r);
    }
  }
  updated_points.push_back(path.back());

  // Erase same references
  for(int i=0; i<updated_points.size()-1;){
    if(updated_points[i] == updated_points[i+1]){
      updated_points.erase(updated_points.begin() + i);
    } else{
      i++;
    }
  }

  // todo: erase points that lie on the same [infinite] line

  return updated_points;
}

float ExactDecomposition::getPathLen(mrs_lib::Point2d p1, mrs_lib::Point2d p2) {
  float res = 0;
  std::vector<mrs_lib::Point2d> path = getPath(p1, p2);
  if(path.size() == 0){
    return -1;
  }
  for(int i=0; i<path.size() - 1; i++){
    res += bg::distance(path[i], path[i+1]);
  }
  return res;
}

double ExactDecomposition::signedDistComparable(Line line, mrs_lib::Point2d point) {
  double A =   (bg::get<1>(line[1]) - bg::get<1>(line[0]));
  double B =  -(bg::get<0>(line[1]) - bg::get<0>(line[0]));
  double C = bg::get<1>(line[0]) * bg::get<0>(line[1]) - bg::get<1>(line[1]) * bg::get<0>(line[0]);

  return (A * bg::get<0>(point)) + (B * bg::get<1>(point)) + C;
}

double ExactDecomposition::signedDistComparable(Ogre::Vector3 line, mrs_lib::Point2d point) {
  return (line.x * bg::get<0>(point)) + (line.y * bg::get<1>(point)) + line.z;
}

ExactDecomposition::Line ExactDecomposition::shrink(mrs_lib::Point2d p1, mrs_lib::Point2d p2, float dist) {
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
  res.push_back(mrs_lib::Point2d{res_p1_x, res_p1_y});
  res.push_back(mrs_lib::Point2d{res_p2_x, res_p2_y});
  return res;
}

// |----------------------- Public methods -----------------------|

void ExactDecomposition::initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node) {
  CoverageMethod::initialize(property_container, scene_manager, root_node);

  boundaries_property_    = new rviz::BoolProperty("Show boundaries", true, "Enable to show", property_container, SLOT(boundariesChanged()), this);
  decomposition_property_ = new rviz::BoolProperty("Show decomposition", true, "Enable to show", property_container, SLOT(decompositionChanged()), this);
  path_property_          = new rviz::BoolProperty("Show path", true, "Enable to show", property_container, SLOT(pathChanged()), this);
}

void ExactDecomposition::setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update){
  current_polygon_ = new_polygon;
  polygon_frame_ = frame_id;
  if(update){
    drawCurrentPolygon();
  }
}

void ExactDecomposition::setStart(Ogre::Vector3 position){
  start_position_ = position;
}

void ExactDecomposition::setAngle(int angle, bool update) {
  angle_ = angle;
  if(update){
    drawCurrentPolygon();
  }
}

void ExactDecomposition::setOverlap(float percentage, bool update) {
  overlap_ = percentage;
  if(update){
    drawCurrentPolygon();
  }
}

void ExactDecomposition::setHeight(float height, bool update) {
  height_ = height;
  if(update){
    drawCurrentPolygon();
  }
}

void ExactDecomposition::setFrame(std::string new_frame, bool update){
  current_frame_ = new_frame;
  if(update){
    drawCurrentPolygon();
  }
}

// |--------------------- Slots ---------------------|

void ExactDecomposition::decompositionChanged() {
  if(decomposition_node_){
    decomposition_node_->setVisible(decomposition_property_->getBool());
  }
}

void ExactDecomposition::boundariesChanged() {
  if(boundaries_node_){
    boundaries_node_->setVisible(boundaries_property_->getBool());
  }
}

void ExactDecomposition::pathChanged() {
  if(path_node_){
    path_node_->setVisible(path_property_->getBool());
  }
}

ExactDecomposition::~ExactDecomposition(){
  if(boundaries_node_){
    scene_manager_->destroySceneNode(boundaries_node_);
  }
  if(decomposition_node_){
    scene_manager_->destroySceneNode(decomposition_node_);
  }
  if(path_node_){
    scene_manager_->destroySceneNode(path_node_);
  }

  delete decomposition_property_;
  delete boundaries_property_;
  delete path_property_;
}

} // namespace mrs_rviz_plugins