#include <coverage_path_planning/approximate_decomposition.h>

#include <rviz/ogre_helpers/shape.h>

// TODO: include polygon only
// #include <mrs_lib/safety_zone/polygon.h>
#include <mrs_lib/safety_zone/prism.h>

#include <ros/ros.h>

#include <cmath>
#include <limits>

namespace bg = boost::geometry; 

namespace mrs_rviz_plugins{

void ApproximateDecomposition::drawGrid(){
  scene_manager_->destroySceneNode(grid_node_);
  grid_node_ = root_node_->createChildSceneNode();
  float twist_rad = (((float)twist_property_->getInt()) / 180) * M_PI;

  // 1: We find rectagle boundaries of the polygon in rotated axes

  // 1.1: Initialize values (max and min are in rotated axes)
  auto outer_ring = current_polygon_.outer();
  float max_x = std::numeric_limits<float>::lowest();
  float min_x = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();

  // 1.2: Iterate over vertices and find max and min 
  for(size_t i=0; i<outer_ring.size(); ++i){
    // Get position in original axes
    float cur_x = bg::get<0>(outer_ring[i]);
    float cur_y = bg::get<1>(outer_ring[i]);

    // Find position in rotated axes
    float tmp_x = std::cos(twist_rad) * cur_x + std::sin(twist_rad) * cur_y;
    float tmp_y = -std::sin(twist_rad) * cur_x + std::cos(twist_rad) * cur_y;

    // Update extrems
    max_x = max_x < tmp_x ? tmp_x : max_x;
    min_x = min_x > tmp_x ? tmp_x : min_x;

    max_y = max_y < tmp_y ? tmp_y : max_y;
    min_y = min_y > tmp_y ? tmp_y : min_y;
  }

  // 2: We fill the rectangle with waypoints

  // 2.1: Compute parameters
  float angle_rad = (((float)angle_) / 180) * M_PI;
  float camera_width = (std::tan(angle_rad / 2) * height_);
  float distance = camera_width * (1 - overlap_);
  Ogre::Quaternion cube_rotation(Ogre::Radian(twist_rad), Ogre::Vector3(0, 0, 1));
  bool show_grid = true;
  const auto& tf = transformer_.getTransform(current_frame_, polygon_frame_);
  if (!tf) {
    ROS_INFO("[ApproximateDecomposition]: Transformation is not found. Grid will not be displayed");
    show_grid = false;
  }

  // 2.2: Iterate over matrix *grid, assign positions (in original axes) and add waypoints to the visualisation
  grid.resize(std::ceil((max_y - min_y) / distance));
  for(size_t i=0; i<grid.size(); ++i){
    grid[i].resize(std::ceil((max_x - min_x) / distance));
    for(size_t j=0; j<grid[i].size(); ++j){
      // Find position in rotated axes
      float tmp_x = min_x + (camera_width / 2) + distance * j;
      float tmp_y = min_y + (camera_width / 2) + distance * i;

      // Assigning position in original axes
      grid[i][j].x = std::cos(-twist_rad) * tmp_x + std::sin(-twist_rad) * tmp_y;
      grid[i][j].y = -std::sin(-twist_rad) * tmp_x + std::cos(-twist_rad) * tmp_y;

      // Add waypoint to the visualisation
      if(!show_grid){
        if(bg::within(mrs_lib::Point2d{grid[i][j].x, grid[i][j].y}, current_polygon_)){
          grid[i][j].valid = true;
        }else{
          grid[i][j].valid = false;
        }
        continue;
      }
      rviz::Shape* cube = new rviz::Shape(rviz::Shape::Type::Cube, scene_manager_, grid_node_);
      cube->setScale(Ogre::Vector3(1, 1, 1));

      // Setting waypoint position
      geometry_msgs::Pose pose;
      pose.position.x = grid[i][j].x;
      pose.position.y = grid[i][j].y;
      pose.position.z = height_;
      pose.orientation.w = cube_rotation.w;
      pose.orientation.x = cube_rotation.x;
      pose.orientation.y = cube_rotation.y;
      pose.orientation.z = cube_rotation.z;
      const auto& point_transformed = transformer_.transform(pose, tf.value());
      if (!point_transformed) {
        ROS_INFO("[ApproximateDecomposition]: Unable to transform cmd reference from %s to %s at time %.6f.", current_frame_.c_str(), polygon_frame_.c_str(),
                ros::Time::now().toSec());
        continue;
      }
      pose = point_transformed.value();
      cube->setPosition(Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z));
      cube->setOrientation(Ogre::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));

      // Verify if the waipoint lies within the polygon
      if(bg::within(mrs_lib::Point2d{grid[i][j].x, grid[i][j].y}, current_polygon_)){
        cube->setColor(0.0F, 1.0F, 0.0F, 1.0F);
        grid[i][j].valid = true;
      }else{
        cube->setColor(1.0F, 0.0F, 0.0F, 1.0F);
        grid[i][j].valid = false;
      }

    }
  }

}

void ApproximateDecomposition::initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){
  CoverageMethod::initialize(property_container, scene_manager, root_node);
  grid_node_ = root_node_->createChildSceneNode();
  twist_property_ = new rviz::IntProperty("Twist", 0, "TODO: add description", property_container_, SLOT(twistChanged()), this);
  twist_property_->setMax(180);
  twist_property_->setMin(0);
  transformer_ = mrs_lib::Transformer(nh_);
}

void ApproximateDecomposition::setPolygon(std::string frame_id, mrs_lib::Polygon &new_polygon, bool update){
  current_polygon_ = new_polygon;
  polygon_frame_ = frame_id;
  if(update){
    drawGrid();
  }
}

void ApproximateDecomposition::setStart(Ogre::Vector3 position){
  start_position_ = position;
}

void ApproximateDecomposition::setAngle(int angle, bool update) {
  angle_ = angle;
  if(update){
    drawGrid();
  }
}

void ApproximateDecomposition::setOverlap(float percentage, bool update) {
  overlap_ = percentage;
  if(update){
    drawGrid();
  }
}

void ApproximateDecomposition::setHeight(float height, bool update) {
  height_ = height;
  if(update){
    drawGrid();
  }
}

void ApproximateDecomposition::setFrame(std::string new_frame, bool update){
  current_frame_ = new_frame;
  if(update){
    drawGrid();
  }
}

void ApproximateDecomposition::twistChanged(){
  drawGrid();
}

} // namespace mrs_rviz_plugins