#include <coverage_path_planning/stride_method.h>

#include <rviz/ogre_helpers/shape.h>

// TODO: include polygon only
// #include <mrs_lib/safety_zone/polygon.h>
#include <mrs_lib/safety_zone/prism.h>

#include <ros/ros.h>

#include <cmath>

namespace bg = boost::geometry; 

namespace mrs_rviz_plugins{

void StrideMethod::drawGrid(){
  auto outer_ring = current_polygon_.outer();
  float max_x = bg::get<0>(outer_ring[0]);
  float min_x = max_x;
  float max_y = bg::get<1>(outer_ring[0]);
  float min_y = max_y;
  for(size_t i=0; i<outer_ring.size(); ++i){
    float cur_x = bg::get<0>(outer_ring[i]);
    float cur_y = bg::get<1>(outer_ring[i]);

    max_x = max_x < cur_x ? cur_x : max_x;
    min_x = min_x > cur_x ? cur_x : min_x;

    max_y = max_x < cur_y ? cur_y : max_y;
    min_y = min_x > cur_y ? cur_y : min_y;
  }

  scene_manager_->destroySceneNode(grid_node_);
  grid_node_ = root_node_->createChildSceneNode();

  float rad = (((float)angle_) / 180) * M_PI;
  float camera_width = (std::tan(rad / 2) * height_);
  float distance = camera_width * (1 - overlap_);

  grid.resize(std::ceil((max_y - min_y) / distance));

  for(size_t i=0; i<grid.size(); ++i){
    grid[i].resize(std::ceil((max_x - min_x) / distance));
    for(size_t j=0; j<grid[i].size(); ++j){
      grid[i][j].x = min_x + (camera_width / 2) + distance * j;
      grid[i][j].y = min_y + (camera_width / 2) + distance * i;

      rviz::Shape* cube = new rviz::Shape(rviz::Shape::Type::Cube, scene_manager_, grid_node_);

      cube->setPosition(Ogre::Vector3(grid[i][j].x, grid[i][j].y, height_));

      if(bg::within(mrs_lib::Point2d{grid[i][j].x, grid[i][j].y}, current_polygon_)){
        cube->setColor(0.0F, 1.0F, 0.0F, 1.0F);
        grid[i][j].valid = true;
      }else{
        cube->setColor(1.0F, 0.0F, 0.0F, 1.0F);
        grid[i][j].valid = false;
      }

      cube->setScale(Ogre::Vector3(1, 1, 1));
    }
  }

}

void StrideMethod::compute(){

}

void StrideMethod::initialize(rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){
  CoverageMethod::initialize(property_container, scene_manager, root_node);
  grid_node_ = root_node_->createChildSceneNode();
}

void StrideMethod::start(){
  ROS_INFO("[StrideMethod]: start called");
}

void StrideMethod::setPolygon(mrs_lib::Polygon &new_polygon){
  current_polygon_ = new_polygon;
  drawGrid();
}

void StrideMethod::setStart(Ogre::Vector3 position){
  start_position_ = position;
}

void StrideMethod::setAngle(int angle) {
  angle_ = angle;
  drawGrid();
}

void StrideMethod::setOverlap(float percentage) {
  overlap_ = percentage;
  drawGrid();
}

void StrideMethod::setHeight(float height) {
  height_ = height;
  drawGrid();
}

} // namespace mrs_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::StrideMethod, mrs_rviz_plugins::CoverageMethod)