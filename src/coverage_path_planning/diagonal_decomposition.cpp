#include <coverage_path_planning/diagonal_decomposition.h>

namespace mrs_rviz_plugins {

void DiagonalDecomposition::initialize (rviz::Property* property_container, Ogre::SceneManager* scene_manager, Ogre::SceneNode* root_node){

}


void DiagonalDecomposition::compute() {
  std::cout << "compute\n";
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