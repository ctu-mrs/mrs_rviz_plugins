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

void ApproximateDecomposition::initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){
  CoverageMethod::initialize(property_container, scene_manager, root_node);
  grid_node_ = root_node_->createChildSceneNode();

  twist_property_ = new rviz::IntProperty("Twist", 0, "Rotate the grid", property_container_, SLOT(twistChanged()), this);
  cell_num_property_ = new rviz::IntProperty("Cells", 0, "Number of cells that are to cover", property_container);
  
  twist_property_->setMax(180);
  twist_property_->setMin(0);
  cell_num_property_->setReadOnly(true);

}

void ApproximateDecomposition::drawGrid(){
  scene_manager_->destroySceneNode(grid_node_);
  grid_node_ = root_node_->createChildSceneNode();
  float twist_rad = (((float)twist_property_->getInt()) / 180) * M_PI;

  // 1: We find rectagle boundaries of the polygon in rotated axes
  float max_x, min_x, max_y, min_y;
  getPolygonBoundaries(max_x, min_x, max_y, min_y);

  // 2: We fill the rectangle with waypoints

  // 2.1: Compute parameters
  int cell_num = 0;
  float angle_rad = (((float)angle_) / 180) * M_PI;
  float camera_width = (std::tan(angle_rad / 2) * height_);
  float distance = camera_width * (1 - overlap_);
  Ogre::Quaternion cube_rotation(Ogre::Radian(twist_rad), Ogre::Vector3(0, 0, 1));
  bool show_grid = true;
  const auto& tf = transformer_.getTransform(polygon_frame_, current_frame_);
  if (!tf) {
    ROS_INFO("[ApproximateDecomposition]: Transformation is not found. Grid will not be displayed");
    show_grid = false;
  }

  // 2.2: Iterate over matrix *grid_, assign positions (in original axes) and add waypoints to the visualisation
  grid_.resize(std::ceil((max_y - min_y) / distance));
  for(size_t i=0; i<grid_.size(); ++i){
    grid_[i].resize(std::ceil((max_x - min_x) / distance));
    for(size_t j=0; j<grid_[i].size(); ++j){
      
      grid_[i][j] = getCell(min_x, min_y, camera_width, distance, twist_rad, i, j);
      cell_num += grid_[i][j].valid ? 1 : 0;

      if(!show_grid){
        continue;
      }

      // Transform point from polygon_frame_ to current_frame_ for the visualisation
      const auto& point_transformed = transform(grid_[i][j], height_, cube_rotation, tf.value());
      if (!point_transformed) {
        ROS_INFO("[ApproximateDecomposition]: Unable to transform cmd reference from %s to %s at time %.6f.", 
                current_frame_.c_str(), 
                polygon_frame_.c_str(),
                ros::Time::now().toSec());
        continue;
      }
      geometry_msgs::Pose pose = point_transformed.value();

      // Add waypoint to the visualisation
      addWaypoint(pose, grid_[i][j].valid);
    }
  }

  cell_num_property_->setInt(cell_num);
}

void ApproximateDecomposition::addWaypoint(geometry_msgs::Pose pose, bool valid){
  rviz::Shape* cube = new rviz::Shape(rviz::Shape::Type::Cube, scene_manager_, grid_node_);
  cube->setScale(Ogre::Vector3(1, 1, 1));
  cube->setPosition(Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z));
  cube->setOrientation(Ogre::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));

  if(valid){
    cube->setColor(0.0F, 1.0F, 0.0F, 1.0F);
  }else{
    cube->setColor(1.0F, 0.0F, 0.0F, 1.0F);
  }
}

ApproximateDecomposition::cell_t ApproximateDecomposition::getCell(float min_x, float min_y, float camera_width, float dist, float twist_rad, size_t x, size_t y){
  cell_t result;
  
  // Find position in rotated axes
  float tmp_x = min_x + (camera_width / 2) + dist * y;
  float tmp_y = min_y + (camera_width / 2) + dist * x;

  // Assigning position in original axes
  result.x = std::cos(-twist_rad) * tmp_x + std::sin(-twist_rad) * tmp_y;
  result.y = -std::sin(-twist_rad) * tmp_x + std::cos(-twist_rad) * tmp_y;

  result.valid = bg::within(mrs_lib::Point2d{result.x, result.y}, current_polygon_);

  return result;
}

void ApproximateDecomposition::getPolygonBoundaries(float& max_x, float& min_x,
                                                    float& max_y, float& min_y){
  // Initialize values
  max_x = std::numeric_limits<float>::lowest();
  min_x = std::numeric_limits<float>::max();
  max_y = std::numeric_limits<float>::lowest();
  min_y = std::numeric_limits<float>::max();
  float twist_rad = (((float)twist_property_->getInt()) / 180) * M_PI;
  auto outer_ring = current_polygon_.outer();

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
}

std::optional<geometry_msgs::Pose> ApproximateDecomposition::transform(cell_t cell, float height, Ogre::Quaternion quat, geometry_msgs::TransformStamped tf){
  geometry_msgs::Pose pose;
  pose.position.x = cell.x;
  pose.position.y = cell.y;
  pose.position.z = height;
  pose.orientation.w = quat.w;
  pose.orientation.x = quat.x;
  pose.orientation.y = quat.y;
  pose.orientation.z = quat.z;
  return transformer_.transform(pose, tf);
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

ApproximateDecomposition::~ApproximateDecomposition(){
  delete twist_property_;
  delete cell_num_property_;
  if(grid_node_){
    scene_manager_->destroySceneNode(grid_node_);
  }
}

} // namespace mrs_rviz_plugins