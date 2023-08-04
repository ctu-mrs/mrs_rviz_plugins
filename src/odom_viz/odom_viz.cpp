#include "odom_viz/odom_viz.h"

#include <Eigen/Dense>

namespace mrs_rviz_plugins
{

OdomViz::OdomViz() {
  // General
  pose_tolerance_property =
      new rviz::FloatProperty("Position tolerance", 0.1, "Distance, in meters from the last arrow dropped, that will cause a new arrow to drop.", this);
  angle_tolerance_property =
      new rviz::FloatProperty("Angle tolerance", 0.1, "Angular distance from the last arrow dropped, that will cause a new arrow to drop.", this);
  keep_property = new rviz::IntProperty("Keep", 100, "Number of arrows to keep before removing the oldest one.", this, SLOT(onKeepChanged()), this);
  keep_property->setMin(1);
  keep_property->setMax(30000);

  // Position
  pose_property = new rviz::BoolProperty("Position", true, "", this, SLOT(onPositionChanged()));
  pose_property->setDisableChildrenIfFalse(true);

  shape_property = new rviz::EnumProperty("Shape", "Arrow", "Shape to display the position as.", pose_property, SLOT(onShapeChanged()), this);
  shape_property->addOption("Arrow", 0);
  shape_property->addOption("Axes", 1);

  //      Arrow
  QColor color        = QColor(255, 25, 0, 255);
  pose_color_property = new rviz::ColorProperty("Color", color, "Color of the position arrows.", pose_property, SLOT(onPoseColorChanged()), this);

  pose_opacity_property = new rviz::IntProperty("Opacity", 255, "Amount of opacity to apply to the arrow. 0 is fully transparent.", pose_property,
                                                SLOT(onPoseColorChanged()), this);
  pose_opacity_property->setMin(1);
  pose_opacity_property->setMax(255);

  pose_shaft_len_property =
      new rviz::FloatProperty("Shaft length", 1, "Length of the each arrow's shaft, in meters.", pose_property, SLOT(onPoseParamsChanged()), this);
  pose_shaft_rad_property =
      new rviz::FloatProperty("Shaft radius", 0.05, "Radius of the each arrow's shaft, in meters.", pose_property, SLOT(onPoseParamsChanged()), this);
  pose_head_len_property =
      new rviz::FloatProperty("Head length", 0.3, "Length of the each arrow's head, in meters.", pose_property, SLOT(onPoseParamsChanged()), this);
  pose_head_rad_property =
      new rviz::FloatProperty("Head radius", 0.1, "Radius of the each arrow's head, in meters.", pose_property, SLOT(onPoseParamsChanged()), this);

  //      Axes
  pose_axes_len_property = new rviz::FloatProperty("Axes length", 1, "Length of each axis, in meters.", pose_property, SLOT(onAxesParamsChanged()), this);
  pose_axes_rad_property = new rviz::FloatProperty("Axes radius", 0.1, "Radius of each axis, in meters", pose_property, SLOT(onAxesParamsChanged()), this);
  pose_axes_len_property->hide();
  pose_axes_rad_property->hide();

  // Velocity
  vel_property = new rviz::BoolProperty("Velocity", false, "", this, SLOT(onVelocityChanged()));
  vel_property->setDisableChildrenIfFalse(true);
  color = QColor(0, 25, 255, 255);

  vel_color_property = new rviz::ColorProperty("Color", color, "Color of the velocity arrows.", vel_property, SLOT(onVelColorChanged()), this);

  vel_opacity_property =
      new rviz::IntProperty("Opacity", 255, "Amount of opacity to apply to the arrow. 0 is fully transparent.", vel_property, SLOT(onVelColorChanged()), this);
  vel_opacity_property->setMin(1);
  vel_opacity_property->setMax(255);

  vel_shaft_len_property =
      new rviz::FloatProperty("Shaft length", 1, "Length of the each arrow's shaft, in meters", vel_property, SLOT(onVelParamsChanged()), this);
  vel_shaft_rad_property =
      new rviz::FloatProperty("Shaft radius", 0.1, "Radius of the each arrow's shaft, in meters.", vel_property, SLOT(onVelParamsChanged()), this);
  vel_head_len_property =
      new rviz::FloatProperty("Head length", 0.3, "Length of the each arrow's head, in meters.", vel_property, SLOT(onVelParamsChanged()), this);
  vel_head_rad_property =
      new rviz::FloatProperty("Head radius", 0.2, "Radius of the each arrow's head, in meters.", vel_property, SLOT(onVelParamsChanged()), this);
  vel_scale_property = new rviz::FloatProperty("Arrow scale", 0.3, "Scale of velocity arrow", vel_property, SLOT(onVelParamsChanged()), this);

  // Covariance
  covariance_property = new rviz::CovarianceProperty("Covariance", true, "Whether or not the covariances of the messages should be shown.", this);
}

void OdomViz::onInitialize() {
  MFDClass::onInitialize();
}

void OdomViz::reset() {
  MFDClass::reset();
  for (auto entity : entities) {
    delete entity;
  }
  entities.clear();

  covariance_property->clearVisual();

  ROS_INFO("[OdometryWithVelocity] reset called");
}

void OdomViz::processMessage(const nav_msgs::Odometry::ConstPtr& msg) {
  bool drop_arrow = false;
  if (!entities.empty()) {
    const Ogre::Vector3      prev_point      = Ogre::Vector3(last_msg->pose.pose.position.x, last_msg->pose.pose.position.y, last_msg->pose.pose.position.z);
    const Ogre::Vector3      new_point       = Ogre::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const Eigen::Quaternionf old_orientation = Eigen::Quaternionf(last_msg->pose.pose.orientation.w, last_msg->pose.pose.orientation.x,
                                                                  last_msg->pose.pose.orientation.y, last_msg->pose.pose.orientation.z);
    const Eigen::Quaternionf new_orientation =
        Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    if ((prev_point - new_point).length() >= pose_tolerance_property->getFloat() ||
        old_orientation.angularDistance(new_orientation) >= angle_tolerance_property->getFloat()) {
      drop_arrow = true;
    }

  } else {

    drop_arrow = true;
  }

  if (!drop_arrow) {
    return;
  }
  last_msg = msg;

  // Keep only certain number of visual entities (including covariances)
  if (entities.size() >= keep_property->getInt() && !entities.empty()) {
    if(entities.front()->getHasCov()){
        covariance_property->popFrontVisual();
    }
    delete entities.front();
    entities.erase(entities.begin());
  }

  // Read parameters of pose arrow
  QColor color = pose_color_property->getColor();
  color.setAlpha(pose_opacity_property->getInt());
  float shaft_length   = pose_shaft_len_property->getFloat();
  float shaft_diameter = pose_shaft_rad_property->getFloat();
  float head_length    = pose_head_len_property->getFloat();
  float head_diameter  = pose_head_rad_property->getFloat();

  PoseType type;
  if (shape_property->getOptionInt() == 0) {
    type = PoseType::ARROW;
  } else if (shape_property->getOptionInt() == 1) {
    type = PoseType::AXES;
  }

  if (!pose_property->getBool()) {
    type = PoseType::INVISIBLE;
  }

  // Set parameters of pose arrow
  VisualEntity* entity = new mrs_rviz_plugins::VisualEntity(context_->getSceneManager());
  entity->setPoseArrowColor(color);
  entity->setPoseArrowParams(shaft_length, shaft_diameter, head_length, head_diameter);
  entity->setAxesParams(pose_axes_len_property->getFloat(), pose_axes_rad_property->getFloat());
  entity->setPoseType(type);

  // Read parameters of vel arrow
  color = vel_color_property->getColor();
  color.setAlpha(vel_opacity_property->getInt());
  shaft_length           = vel_shaft_len_property->getFloat();
  shaft_diameter         = vel_shaft_rad_property->getFloat();
  head_length            = vel_head_len_property->getFloat();
  head_diameter          = vel_head_rad_property->getFloat();
  const float scale      = vel_scale_property->getFloat();
  const bool  is_visible = vel_property->getBool();

  // Set parameters of velocity arrow
  entity->setVelArrowColor(color);
  entity->setVelArrowParams(shaft_length, shaft_diameter, head_length, head_diameter, vel_scale_property->getFloat());
  entity->setVelVisible(is_visible);

  // Associate a CovarianceVisual in covariance_property
  const bool is_cov_null = isArrayAllNull(msg->pose.covariance) && isArrayAllNull(msg->twist.covariance);
  entity->setHasCov(!is_cov_null);

  // Covariances are stored in covariance_property
  typedef rviz::CovarianceProperty::CovarianceVisualPtr CovarianceVisualPtr;
  CovarianceVisualPtr cov; 
  if(!is_cov_null){
    cov = covariance_property->createAndPushBackVisual(context_->getSceneManager(), context_->getSceneManager()->getRootSceneNode());
  }

  // Read data from message
  Ogre::Vector3    position = Ogre::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Ogre::Quaternion orientation =
      Ogre::Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  std::string         body_frame = msg->child_frame_id;
  ros::Time           time       = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  geometry_msgs::Pose pose;
  Ogre::Quaternion    quat;
  quat                   = quat.IDENTITY;
  pose.position.x        = msg->twist.twist.linear.x;
  pose.position.y        = msg->twist.twist.linear.y;
  pose.position.z        = msg->twist.twist.linear.z;
  pose.orientation.w     = quat.w;
  pose.orientation.x     = quat.x;
  pose.orientation.y     = quat.y;
  pose.orientation.z     = quat.z;
  Ogre::Vector3 velocity = Ogre::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  float         vel_abs  = velocity.length();

  // Transforming pose
  if (!context_->getFrameManager()->transform(msg->header, msg->pose.pose, position, orientation)) {
    ROS_ERROR("Error transforming odometry '%s' from frame '%s' to frame '%s'", qPrintable(getName()), msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }
  // Transform twist
  if (!context_->getFrameManager()->transform(body_frame, time, pose, velocity, quat)) {
    ROS_ERROR("Error transforming odometry from frame '%s' to frame '%s'", body_frame.c_str(), qPrintable(fixed_frame_));
    return;
  }
  velocity = velocity - position;

  if(!is_cov_null){
    cov->setPosition(position);
    cov->setOrientation(orientation);
    cov->setCovariance(msg->pose);
  }
  entity->set(position, velocity, orientation, vel_abs);
  entities.push_back(entity);
}

template<size_t a> bool OdomViz::isArrayAllNull(const boost::array<double, a> arr) {
  for (auto covariance_value : arr) {
    if (covariance_value != 0.0) {
      return false;
    }
  }
  return true;
}

void OdomViz::onDisable() {
  MFDClass::onDisable();
  ROS_INFO("[OdometryWithVelocity] Disabled");
}

void OdomViz::onEnable() {
  MFDClass::onEnable();
  ROS_INFO("[OdometryWithVelocity] Enabled");
}

void OdomViz::onKeepChanged() {
  int to_delete = int(entities.size()) - keep_property->getInt();
  if (to_delete <= 0) {
    return;
  }
  for (int i = 0; i < to_delete; i++) {
    delete entities[i];
  }
  entities.erase(entities.begin(), entities.begin() + to_delete);
}

void OdomViz::onPositionChanged() {
  PoseType type;
  if (shape_property->getOptionInt() == 0) {
    type = PoseType::ARROW;
  } else if (shape_property->getOptionInt() == 1) {
    type = PoseType::AXES;
  }
  if (!pose_property->getBool()) {
    type = PoseType::INVISIBLE;
  }

  for (const auto& entity : entities) {
    entity->setPoseType(type);
  }
}

void OdomViz::onShapeChanged() {
  PoseType new_type = PoseType::ARROW;

  if (shape_property->getOptionInt() == 0) {
    ROS_INFO("[OdometryWithVelocity] shape has been changed to Arrow (0)");
    new_type = PoseType::ARROW;

    // Hide axes properties:
    pose_axes_len_property->hide();
    pose_axes_rad_property->hide();

    // Show arrow properties:
    pose_color_property->setHidden(false);
    pose_opacity_property->setHidden(false);
    pose_shaft_len_property->setHidden(false);
    pose_shaft_rad_property->setHidden(false);
    pose_head_len_property->setHidden(false);
    pose_head_rad_property->setHidden(false);

  } else {

    ROS_INFO("[OdometryWithVelocity] shape has been changed to Axes (1)");
    new_type = PoseType::AXES;

    // Hide arrow properties:
    pose_color_property->hide();
    pose_opacity_property->hide();
    pose_shaft_len_property->hide();
    pose_shaft_rad_property->hide();
    pose_head_len_property->hide();
    pose_head_rad_property->hide();

    // Show axes properties:
    pose_axes_len_property->setHidden(false);
    pose_axes_rad_property->setHidden(false);
  }

  for (const auto& entity : entities) {
    entity->setPoseType(new_type);
  }
}

void OdomViz::onPoseColorChanged() {

  QColor color = pose_color_property->getColor();
  color.setAlpha(pose_opacity_property->getInt());

  for (const auto& entity : entities) {
    entity->setPoseArrowColor(color);
  }
}

void OdomViz::onPoseParamsChanged() {
  const float shaft_length   = pose_shaft_len_property->getFloat();
  const float shaft_diameter = pose_shaft_rad_property->getFloat();
  const float head_length    = pose_head_len_property->getFloat();
  const float head_diameter  = pose_head_rad_property->getFloat();

  for (const auto& entity : entities) {
    entity->setPoseArrowParams(shaft_length, shaft_diameter, head_length, head_diameter);
  }
}

void OdomViz::onAxesParamsChanged() {
  const float len = pose_axes_len_property->getFloat();
  const float rad = pose_axes_rad_property->getFloat();
  for (const auto& entity : entities) {
    entity->setAxesParams(len, rad);
  }
}

void OdomViz::onVelocityChanged() {
  const bool value = vel_property->getBool();
  for (const auto& entity : entities) {
    entity->setVelVisible(value);
  }
}

void OdomViz::onVelColorChanged() {
  QColor color = vel_color_property->getColor();
  color.setAlpha(vel_opacity_property->getInt());

  for (const auto& entity : entities) {
    entity->setVelArrowColor(color);
  }
}

void OdomViz::onVelParamsChanged() {
  const float shaft_length   = vel_shaft_len_property->getFloat();
  const float shaft_diameter = vel_shaft_rad_property->getFloat();
  const float head_length    = vel_head_len_property->getFloat();
  const float head_diameter  = vel_head_rad_property->getFloat();
  const float scale          = vel_scale_property->getFloat();

  for (const auto& entity : entities) {
    entity->setVelArrowParams(shaft_length, shaft_diameter, head_length, head_diameter, scale);
  }
}

}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::OdomViz, rviz::Display)
