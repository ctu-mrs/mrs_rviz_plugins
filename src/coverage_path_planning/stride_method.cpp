#include "coverage_path_planning/stride_method.h"

#include <ros/ros.h>

#include <vector>
#include <cmath>
#include <limits>

namespace mrs_rviz_plugins{

void StrideMethod::compute(){
  float min_dist = std::numeric_limits<float>::max();
  Ogre::Vector2 index_min;
  
  std::cout << "compute entered\n";
  std::cout << "start: " << start_position_.x << " " << start_position_.y << std::endl;
  // Searching for the closest cell to the start position.
  for(size_t i=0; i<grid.size(); ++i){
    for(size_t j=0; j<grid[i].size(); ++j){
      float cur_dist;

      // std::cout << "grid: " << grid[i][j].x << " " << grid[i][j].y << std::endl;
      cur_dist = std::pow(grid[i][j].x - start_position_.x, 2) + std::pow(grid[i][j].y - start_position_.y, 2);
      if(cur_dist < min_dist){
        index_min.x = i;
        index_min.y = j;
        min_dist = cur_dist;
      }
    }
  }

  std::cout << "closest cell found\n";
  start_position_.x = grid[index_min.x][index_min.y].x;
  start_position_.y = grid[index_min.x][index_min.y].y;

  // |------------------- Algorithm -------------------|
  
  // 1. Set the current cell to the initial cell.
  Ogre::Vector2 cur_cell;
  cur_cell = index_min;
  path_.request.path.points.clear();
  std::cout << "1. processed\n";

  while(true){
    // 2. Find all unvisited neighbor cells of the current cell
    std::vector<Ogre::Vector2> directions {
      Ogre::Vector2(0, 1),
      Ogre::Vector2(0, -1),
      Ogre::Vector2(1, 0),
      Ogre::Vector2(-1, 0)
    };
    std::vector<size_t> valid_directions;
    for(size_t i=0; i<directions.size(); ++i){
      if(!isLimit(index_min + directions[i])){
        valid_directions.push_back(i);
      }
    }

    std::cout << "2. processed\n";

    // 2.1 If no neighbor has been found, find 
    // the nearest unvisited cell located next to cells already visited.
    if(valid_directions.size() == 0){
      // TODO: implement me!
      break;
    }

    // 3. Generate the longest possible stride in the direction
    // of each unvisited neighbor cell.
    std::vector<stride_t> strides;
    for(size_t& index : valid_directions){
      strides.push_back(computeStride(cur_cell, directions[index]));
    }

    std::cout << "3. processed\n";
    // 4. Select the longest stride.
    stride_t longest_stride;
    longest_stride.len = 0;
    for(stride_t& stride : strides){
      if(stride.len > longest_stride.len){
        longest_stride = stride;
      }
    }

    std::cout << "4. processed\n";

    // 5. Add all cells of the stride to the path and mark them
    // as visited.
    path_.request.path.header.frame_id   = polygon_frame_;
    path_.request.path.fly_now           = true;
    path_.request.path.stop_at_waypoints = true;
    path_.request.path.loop              = false;
    Ogre::Vector2 last_cell = longest_stride.start;
    for(size_t i=0; i<longest_stride.len; i++){
      grid[last_cell.x][last_cell.y].visited = true;
      last_cell = last_cell + longest_stride.direction;
    }
    mrs_msgs::Reference ref;
    ref.position.x = grid[last_cell.x][last_cell.y].x;
    ref.position.y = grid[last_cell.x][last_cell.y].y;
    ref.position.z = height_;
    path_.request.path.points.push_back(ref);

    std::cout << "5. processed\n";

    // 6. Set the current cell to the last cell of the stride.
    cur_cell = last_cell;

    std::cout << "6. processed\n";

    // 7. Repeat starting at point 2 until all cells have been
    // visited.
    for(auto& vector : grid){
      for(auto& cell : vector){
        if(!cell.visited && cell.valid){
          continue;
        }
      }
    }
    
    std::cout << "7. processed\n";
    break;

  }
  is_computed_ = true;
  
  // Cleaning the grid for future computations.
  for(auto& vector : grid)
    for(auto& cell : vector)
      cell.visited = false;
}

void StrideMethod::start(){
  if(!is_computed_){
    ROS_WARN("[StrideMethod]: Could not start the mission. The path has not been computed yet.");
    return;
  }
  ROS_INFO("[StrideMethod]: start() is called");
}

StrideMethod::stride_t StrideMethod::computeStride(Ogre::Vector2 start, Ogre::Vector2 direction){
  StrideMethod::stride_t result;
  result.start = start;
  result.direction = direction;
  result.len = 1;

  Ogre::Vector2 last_cell = start;
  Ogre::Vector2 next_cell;
  next_cell.x = start.x;
  next_cell.y = start.y;

  limit_t limits_start = getLimits(start, direction);

  while(true){
    next_cell.x = next_cell.x + direction.x;
    next_cell.y = next_cell.y + direction.y;

    // We do not add next_cell if it is already in the generated path or 
    // if it falls outside the boundaries of the area
    if(next_cell.x < 0 || next_cell.x >= grid.size()) {
      break;
    }
    if(next_cell.y < 0 || next_cell.y >= grid[next_cell.x].size()){
      break;
    }
    if(grid[next_cell.x][next_cell.y].visited || !grid[next_cell.x][next_cell.y].valid){
      break;
    }

    limit_t limits_last = getLimits(last_cell, direction);
    limit_t limits_next = getLimits(next_cell, direction);
    
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
      limits_next.first == limits_start.second && limits_next.second == limits_start.first){
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
    res.second = false;
    res.num++;
  }

  return res;
}

bool StrideMethod::isLimit(Ogre::Vector2 cell){
  if(cell.x < 0 || cell.x >= grid.size()) {
    return true;
  }
  if(cell.y < 0 || cell.y >= grid[cell.x].size()){
    return true;
  }
  if(grid[cell.x][cell.y].visited || !grid[cell.x][cell.y].valid){
    return true;
  }
  return false;
}

} // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StrideMethod, mrs_rviz_plugins::CoverageMethod)