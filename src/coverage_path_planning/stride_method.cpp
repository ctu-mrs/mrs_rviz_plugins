#include "coverage_path_planning/stride_method.h"
#include "coverage_path_planning/planner_tool.h"

#include <ros/ros.h>

#include <rviz/ogre_helpers/line.h>

#include <mrs_lib/safety_zone/prism.h>

#include <boost/geometry.hpp>

#include <vector>
#include <cmath>
#include <limits>
#include <queue>

namespace bg = boost::geometry;
using Line = bg::model::linestring<mrs_lib::Point2d>;

namespace mrs_rviz_plugins{

void StrideMethod::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){
  ApproximateDecomposition::initialize(property_container, scene_manager, root_node);

  turn_num_property_ = new rviz::IntProperty("Turns", 0, "Number of turns in current path", property_container);
  drone_name_property_ = new rviz::EditableEnumProperty("Uav", "", "Uav used to perform coverage mission", property_container);

  turn_num_property_->setReadOnly(true);

  std::vector<std::string> drone_names = PlannerTool::getUavNames();
  for(auto& name : drone_names){
    drone_name_property_->addOption(name.c_str());
  }

  if(drone_names.size() > 0){
    drone_name_property_->setString(drone_names[0].c_str());
  }else{
    ROS_WARN("[StrideMethod]: could not find any uav for coverage mission");
  }
}

void StrideMethod::compute(){
  Ogre::Vector2 index_min;

  geometry_msgs::Point start;
  start.x = start_position_.x;
  start.y = start_position_.y;
  start.z = start_position_.z;
  const auto& start_transformed = transformer_.transformSingle(current_frame_, start, polygon_frame_);
  if (!start_transformed) {
    ROS_INFO("[StrideMethod]: Unable to transform cmd reference from %s to %s at time %.6f.", current_frame_.c_str(), polygon_frame_.c_str(),
            ros::Time::now().toSec());
    return;
  }
  start = start_transformed.value();

  // Searching for the closest cell to the start position.
  float min_dist = std::numeric_limits<float>::max();
  for(size_t i=0; i<grid_.size(); ++i){
    for(size_t j=0; j<grid_[i].size(); ++j){
      float cur_dist;
      if(!grid_[i][j].valid){
        continue;
      }

      cur_dist = std::pow(grid_[i][j].x - start.x, 2) + std::pow(grid_[i][j].y - start.y, 2);
      if(cur_dist < min_dist){
        index_min.x = i;
        index_min.y = j;
        min_dist = cur_dist;
      }
    }
  }

  // |------------------- Algorithm -------------------|
  
  // 1. Set the current cell to the initial cell.
  Ogre::Vector2 cur_cell;
  cur_cell = index_min;
  path_.request.path.points.clear();
  path_.request.path.header.frame_id   = polygon_frame_;
  path_.request.path.fly_now           = true;
  path_.request.path.stop_at_waypoints = false;
  path_.request.path.loop              = false;
  addCellToPath(cur_cell);

  while(true){
    // 2. Find all unvisited neighbor cells of the current cell
    std::vector<Ogre::Vector2> directions {
      Ogre::Vector2(0, 1),
      Ogre::Vector2(0, -1),
      Ogre::Vector2(1, 0),
      Ogre::Vector2(-1, 0)
    };
    std::vector<size_t> valid_directions;
    std::cout << "computing valid directions\n";
    for(size_t i=0; i<directions.size(); ++i){
      Ogre::Vector2 neighbor_i = cur_cell + directions[i]; 
      bool is_valid = true;
      is_valid = is_valid && !isLimit(neighbor_i);

      if(is_valid){
        mrs_lib::Point2d p1{grid_[cur_cell.x][cur_cell.y].x, grid_[cur_cell.x][cur_cell.y].y};
        mrs_lib::Point2d p2{grid_[neighbor_i.x][neighbor_i.y].x, grid_[neighbor_i.x][neighbor_i.y].y};
        Line step;
        step.push_back(p1);
        step.push_back(p2);
        is_valid = is_valid && bg::within(step, current_polygon_);
      }

      if(is_valid){
        valid_directions.push_back(i);
      }
    }

    // 2.1 If no neighbor has been found, find 
    // the nearest unvisited cell located next to cells already visited.
    if(valid_directions.size() == 0){
      std::cout << "Neighbor cell was not found, starting bfs..\n";
      std::vector<Ogre::Vector2> path_to_next = getPathToNextCell(cur_cell);
      
      if(path_to_next.size() == 0){
        ROS_WARN("[StrideMethod]: Could not find the path to unvisited cell. Terminating algorithm");
        break;
      }

      // Delete redundant waypoints
      for(size_t i=1; i<path_to_next.size() - 1; ++i){
        if(path_to_next[i] - path_to_next[i-1] == path_to_next[i+1] - path_to_next[i]){
          continue;
        }
        addCellToPath(path_to_next[i]);
      }
      addCellToPath(path_to_next.back());

      cur_cell = path_to_next.back();
      continue;
    }

    std::cout << "computing strides\n";
    // 3. Generate the longest possible stride in the direction
    // of each unvisited neighbor cell.
    std::vector<stride_t> strides;
    for(size_t& index : valid_directions){
      strides.push_back(computeStride(cur_cell, directions[index]));
    }

    std::cout << "selecting the longes stride\n";
    // 4. Select the longest stride.
    stride_t longest_stride;
    longest_stride.len = 0;
    for(stride_t& stride : strides){
      if(stride.len > longest_stride.len){
        longest_stride = stride;
      }
    }

    std::cout << "adding stride to the path\n";
    // 5. Add all cells of the stride to the path and mark them
    // as visited.
    Ogre::Vector2 last_cell = longest_stride.start;
    for(size_t i=0; i<longest_stride.len; i++){
      grid_[last_cell.x][last_cell.y].visited = true;
      last_cell = last_cell + longest_stride.direction;
    }
    addCellToPath(last_cell);

    // 6. Set the current cell to the last cell of the stride.
    cur_cell = last_cell;

    // 7. Repeat starting at point 2 until all cells have been
    // visited.
    bool finished = true;
    for(auto& vector : grid_){
      for(auto& cell : vector){
        if(!cell.visited && cell.valid){
          finished = false;
        }
      }
    }
    if(finished){
      break;
    }
  }
  is_computed_ = true;
  ROS_INFO("[StrideMethod]: The path has been computed");
  
  // Cleaning the grid_ for future computations.
  for(auto& vector : grid_)
    for(auto& cell : vector)
      cell.visited = false;

  drawPath();
}

std::vector<mrs_msgs::Path> StrideMethod::getPath() {
  std::vector<mrs_msgs::Path> result;
  if(is_computed_){
    result.push_back(path_.request.path);
  }
  return result;
}

void StrideMethod::setPath(std::vector<mrs_msgs::Path> paths) {
  if(paths.size() != 0){
    is_computed_ = true;
    path_.request.path = paths.front();
    drawPath();
  }
}

void StrideMethod::drawPath(){
  // Delete previous path
  if(path_node_){
    scene_manager_->destroySceneNode(path_node_);
  }
  path_node_ = root_node_->createChildSceneNode();

  // Prepare transformation
  const auto& tf = transformer_.getTransform(path_.request.path.header.frame_id, current_frame_);
  if (!tf) {
    ROS_INFO("[StrideMethod]: Transformation is not found. Path will not be displayed");
    return;
  }

  for(size_t i=0; i<path_.request.path.points.size()-1; ++i){
    geometry_msgs::Point start = path_.request.path.points[i].position;
    geometry_msgs::Point end = path_.request.path.points[i+1].position;

    // Transform points to current frame
    const auto& start_transformed = transformer_.transform(start, tf.value());
    const auto& end_transformed = transformer_.transform(end, tf.value());
    if (!start_transformed || !end_transformed) {
      ROS_INFO("[StrideMethod]: Unable to transform cmd reference from %s to %s at time %.6f.", path_.request.path.header.frame_id.c_str(), current_frame_.c_str(),
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

  turn_num_property_->setInt(path_.request.path.points.size() - 1);
}

void StrideMethod::addCellToPath(Ogre::Vector2 cell){
  grid_[cell.x][cell.y].visited = true;
  mrs_msgs::Reference ref;
  ref.position.x = grid_[cell.x][cell.y].x;
  ref.position.y = grid_[cell.x][cell.y].y;
  ref.position.z = height_;
  path_.request.path.points.push_back(ref);
}

// The function has been generated by ChatGPT
bool isValid(int row, int col, int numRows, int numCols) {
    return (row >= 0) && (row < numRows) && (col >= 0) && (col < numCols);
}

// The function has been generated by ChatGPT
std::vector<Ogre::Vector2> StrideMethod::getPathToNextCell(Ogre::Vector2 start){
  // Define possible movements: up, down, left, right, diagonally
  int dx[] = {-1, 1, 0, 0, 1, 1, -1, -1};
  int dy[] = {0, 0, -1, 1, 1, -1, -1, 1};

  int numRows = grid_.size();
  int numCols = grid_[0].size();

  // Create a queue for BFS
  std::queue<std::pair<Ogre::Vector2, std::vector<Ogre::Vector2>>> q;

  // Mark the start cell as visited and enqueue it with an empty path
  std::vector<std::vector<bool>> visited(numRows, std::vector<bool>(numCols, false));
  visited[start.x][start.y] = true;
  q.push({start, {}});

  // Perform BFS
  while (!q.empty()) {
    Ogre::Vector2 currCell = q.front().first;
    std::vector<Ogre::Vector2> path = q.front().second;
    q.pop();

    // Check if the current cell is the goal cell
    if (isNextToVisited(currCell)) {
      // Include the goal cell in the path and return it
      path.push_back(currCell);
      return path;
    }

    // Explore adjacent cells
    for (int i = 0; i < 8; i++) {
      int newRow = currCell.x + dx[i];
      int newCol = currCell.y + dy[i];

      bool is_valid = true;

      // Check if the new cell is valid and not visited
      is_valid = is_valid && isValid(newRow, newCol, numRows, numCols);

      if(is_valid){
        // Check if the step lies within the polygon
        mrs_lib::Point2d p1 {grid_[currCell.x][currCell.y].x, grid_[currCell.x][currCell.y].y};
        mrs_lib::Point2d p2 {grid_[newRow][newCol].x, grid_[newRow][newCol].y};
        bg::model::linestring<mrs_lib::Point2d> line;
        line.push_back(p1);
        line.push_back(p2);
        is_valid = is_valid && bg::within(line, current_polygon_);
        
        is_valid = is_valid && !visited[newRow][newCol];
        is_valid = is_valid &&  grid_[newRow][newCol].valid;
      }

      if (is_valid) {
        // Mark the new cell as visited
        visited[newRow][newCol] = true;
        // Enqueue the new cell with the current path plus the new cell
        std::vector<Ogre::Vector2> newPath = path;
        newPath.push_back(currCell);
        q.push({{newRow, newCol}, newPath}); 
      }
    }
  }

  // If goal cell is not reachable, return an empty path
  return {};
}

bool StrideMethod::isNextToVisited(Ogre::Vector2 cell){
  if(grid_[cell.x][cell.y].visited){
    return false;
  }

  bool result = false;
  std::vector<Ogre::Vector2> directions {
    Ogre::Vector2(0, 1),
    Ogre::Vector2(0, -1),
    Ogre::Vector2(1, 0),
    Ogre::Vector2(-1, 0)
  };

  for(size_t i=0; i<directions.size(); ++i){
    Ogre::Vector2 cur_cell = cell + directions[i];
    if(cur_cell.x < 0 || cur_cell.x >= grid_.size()) {
      continue;
    }
    if(cur_cell.y < 0 || cur_cell.y >= grid_[cell.x].size()){
      continue;
    }
    if(grid_[cur_cell.x][cur_cell.y].visited && grid_[cur_cell.x][cur_cell.y].valid){
      result = true;
    }
  }
  return result;
}

void StrideMethod::start(){
  if(!is_computed_){
    ROS_WARN("[StrideMethod]: Could not start the mission. The path has not been computed yet.");
    return;
  }
  client_ = nh_.serviceClient<mrs_msgs::PathSrv>("/" + drone_name_property_->getStdString() + "/trajectory_generation/path");

  // Make the call
  if(!client_.call(path_)){
    ROS_INFO("[StrideMethod]: Call failed. Service name: %s", client_.getService().c_str());
    return;
  }
  if (!path_.response.success) {
    ROS_INFO("[StrideMethod]: Call failed: %s", path_.response.message.c_str());
    return;
  } 
  ROS_INFO("[StrideMethod]: Call processed successfully");
}

StrideMethod::stride_t StrideMethod::computeStride(Ogre::Vector2 start, Ogre::Vector2 direction){
  StrideMethod::stride_t result;
  result.start = start;
  result.direction = direction;
  result.len = 1;

  Ogre::Vector2 last_cell = start;
  Ogre::Vector2 next_cell;
  next_cell.x = start.x + direction.x;
  next_cell.y = start.y + direction.y;

  limit_t limits_start = getLimits(start, direction);
  std::cout << "direction: " << direction.x << " " << direction.y << std::endl;
  std::cout << "limits_start: " << (limits_start.first ? "true" : "false") << " " << (limits_start.first ? "true" : "false") << std::endl;

  while(true){
    last_cell = next_cell;
    next_cell.x = next_cell.x + direction.x;
    next_cell.y = next_cell.y + direction.y;

    // We do not add next_cell if it is already in the generated path or 
    // if it falls outside the boundaries of the area
    if(next_cell.x < 0 || next_cell.x >= grid_.size()) {
      std::cout << "break x size\n";
      break;
    }
    if(next_cell.y < 0 || next_cell.y >= grid_[next_cell.x].size()){
      std::cout << "break y size\n";
      break;
    }
    if(grid_[next_cell.x][next_cell.y].visited || !grid_[next_cell.x][next_cell.y].valid){
      std::cout << "break grid visited or valid \n";
      break;
    }
    mrs_lib::Point2d p1{grid_[start.x][start.y].x, grid_[start.x][start.y].x};
    mrs_lib::Point2d p2{grid_[next_cell.x][next_cell.y].x, grid_[next_cell.x][next_cell.y].x};
    Line step;
    step.push_back(p1);
    step.push_back(p2);
    if(bg::within(step, current_polygon_)){
      break;
    }

    limit_t limits_last = getLimits(last_cell, direction);
    limit_t limits_next = getLimits(next_cell, direction);
    std::cout << "limits_last: " << (limits_last.first ? "true" : "false") << " " << (limits_last.first ? "true" : "false") << std::endl;
    std::cout << "limits_next: " << (limits_next.first ? "true" : "false") << " " << (limits_next.first ? "true" : "false") << std::endl;

    // If getLimitNum(last_cell) = 2 we always add next_cell, because next_cell
    // is the only possible cell where we can go to from last_cell
    if(limits_last.num == 2){
      result.len++;
      continue;
    }

    // If getLimitNum(last_cell) != 2, any of the following conditions will 
    // prevent addition of next_cell to the stride:

    if(limits_next.num == 0){
      break;
    }

    if(limits_next.num != limits_start.num){
      break;
    }

    if(limits_next.num == 1 &&
      limits_next.first == limits_start.second && 
      limits_next.second == limits_start.first){
      break;
    }

    result.len++;
  }

  return result;
}

StrideMethod::limit_t StrideMethod::getLimits(Ogre::Vector2 cell, Ogre::Vector2 direction){
  limit_t res;

  Ogre::Vector2 sample;
  sample.x = cell.x + direction.y;
  sample.y = cell.y + direction.x;
  if(isLimit(sample)){
    res.first = true;
    res.num++;
  }

  sample.x = cell.x - direction.y;
  sample.y = cell.y - direction.x;
  if(isLimit(sample)){
    res.second = true;
    res.num++;
  }

  return res;
}

bool StrideMethod::isLimit(Ogre::Vector2 cell){
  if(cell.x < 0 || cell.x >= grid_.size()) {
    return true;
  }
  if(cell.y < 0 || cell.y >= grid_[cell.x].size()){
    return true;
  }
  if(grid_[cell.x][cell.y].visited || !grid_[cell.x][cell.y].valid){
    return true;
  }
  return false;
}


StrideMethod::~StrideMethod(){
  delete turn_num_property_;
  delete drone_name_property_;
  if(path_node_){
    scene_manager_->destroySceneNode(path_node_);
  }
}

} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StrideMethod, mrs_rviz_plugins::CoverageMethod)