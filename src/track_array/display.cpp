#include "track_array/display.h"


namespace mrs_rviz_plugins
{

namespace track_array 
{

/* DisplaySelectionHandler and some functions from 
        https://github.com/ctu-mrs/mrs_rviz_plugins/blob/master/src/pose_with_covariance_array/display.cpp/ */

class DisplaySelectionHandler : public rviz::SelectionHandler {

public:
  DisplaySelectionHandler(Display* display, rviz::DisplayContext* context) : SelectionHandler(context), root_(std::make_unique<rviz::Property>()), display_(display) {}

  void createProperties([[maybe_unused]] const rviz::Picked& obj, [[maybe_unused]] rviz::Property* parent_property) {
    properties_.push_back(new rviz::Property(display_->getName(), QVariant(), display_->getDescription(), parent_property));
    rviz::Property*& root = properties_.back();
    rviz::Property* tmp = new rviz::StringProperty("frame ID", (message_->header.frame_id).c_str(), "name of the frame", root);
    tmp->setReadOnly(true);

    for (const auto& cur_track : message_->tracks)
    {
      rviz::Property* sub_root = new rviz::Property(("TrackWithCovariance " + std::to_string(cur_track.id)).c_str(), QVariant(), "position, orientation and velocity with covariance", root);
      
      tmp = new rviz::IntProperty("ID", cur_track.id, "ID", sub_root);
      tmp->setReadOnly(true);
      tmp = new rviz::VectorProperty("position", Ogre::Vector3(cur_track.position.x, cur_track.position.y, 
              cur_track.position.z), "position", sub_root);
      tmp->setReadOnly(true);
      tmp = new rviz::QuaternionProperty("orientation", Ogre::Quaternion(cur_track.orientation.w, 
              cur_track.orientation.x, cur_track.orientation.y, cur_track.orientation.z), "orientation", sub_root);
      tmp->setReadOnly(true);
    }
    root_->setParent(parent_property);
  }

  void getAABBs([[maybe_unused]] const rviz::Picked& obj, rviz::V_AABB& aabbs) {

    for (const auto& cur_track : display_->disp_data_) {
      if (display_->pose_valid_) {
        aabbs.push_back(cur_track.arrow_vel_->getHead()->getEntity()->getWorldBoundingBox());
        aabbs.push_back(cur_track.arrow_vel_->getShaft()->getEntity()->getWorldBoundingBox());

        aabbs.push_back(cur_track.axes_pose_->getXShape()->getEntity()->getWorldBoundingBox());
        aabbs.push_back(cur_track.axes_pose_->getYShape()->getEntity()->getWorldBoundingBox());
        aabbs.push_back(cur_track.axes_pose_->getZShape()->getEntity()->getWorldBoundingBox());

        if (display_->pose_covariance_property_->getBool()) {
          if (display_->pose_covariance_property_->getPositionBool()) {
            aabbs.push_back(cur_track.covariance_pose_->getPositionShape()->getEntity()->getWorldBoundingBox());
          }
          if (display_->pose_covariance_property_->getOrientationBool()) {
            aabbs.push_back(
                cur_track.covariance_pose_->getOrientationShape(rviz::CovarianceVisual::kRoll)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(
                cur_track.covariance_pose_->getOrientationShape(rviz::CovarianceVisual::kPitch)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(
                cur_track.covariance_pose_->getOrientationShape(rviz::CovarianceVisual::kYaw)->getEntity()->getWorldBoundingBox());
          }
        }

        if (display_->velocity_covariance_property_->getBool()) {
          if (display_->velocity_covariance_property_->getPositionBool()) {
            aabbs.push_back(cur_track.covariance_vel_->getPositionShape()->getEntity()->getWorldBoundingBox());
          }
          if (display_->velocity_covariance_property_->getOrientationBool()) {
            aabbs.push_back(
                cur_track.covariance_vel_->getOrientationShape(rviz::CovarianceVisual::kRoll)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(
                cur_track.covariance_vel_->getOrientationShape(rviz::CovarianceVisual::kPitch)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(
                cur_track.covariance_vel_->getOrientationShape(rviz::CovarianceVisual::kYaw)->getEntity()->getWorldBoundingBox());
          }
        }
      }
    }
  }

  void setMessage(const mrs_msgs::TrackArrayStampedConstPtr& message) {
    message_ = message;
    tracked_objects_.clear();
    /* updateProperties(); */
  }

private:
  std::unique_ptr<rviz::Property> root_;
  mrs_msgs::TrackArrayStampedConstPtr message_;
  Display* display_;
};


Display::Display() : pose_valid_(false) {

  /* Orientation Axes*/
  axes_bool_property_ = std::make_unique<rviz::BoolProperty>("Axes", true, "Whether or not axes should be shown.", 
          this, SLOT(updateAxesVisibility()));

  axes_length_property_ = std::make_unique<rviz::FloatProperty>("Axes Length", 1, "Length of each axis, in meters.", 
          this, SLOT(updateAxesGeometry()));

  axes_radius_property_ = std::make_unique<rviz::FloatProperty>("Axes Radius", 0.1, "Radius of each axis, in meters.", 
          this, SLOT(updateAxesGeometry()));


  /* Velocity Arrow */
  velocity_arrow_bool_property_ = std::make_unique<rviz::BoolProperty>("Velocity Arrow", true, "Whether or not the velocity arrow should be shown.", 
          this, SLOT(updateVelocityArrowVisibility()));

  velocity_color_property_ = std::make_unique<rviz::ColorProperty>("Velocity Color", QColor(255, 100, 0), "Color to draw the arrow and it's covariance.", 
          this, SLOT(updateVelocityColorAndAlpha()));

  velocity_arrow_alpha_property_ = std::make_unique<rviz::FloatProperty>("Velocity Arrow Alpha", 1, "Amount of transparency to apply to the velocity arrow.", 
          this, SLOT(updateVelocityColorAndAlpha()));
  velocity_arrow_alpha_property_->setMin(0);
  velocity_arrow_alpha_property_->setMax(1);

  velocity_arrow_length_scale_property_ = std::make_unique<rviz::FloatProperty>("Velocity Arrow Length Scale", 1, "Scale of the arrow's length.", 
          this, SLOT(updateVelocityArrowGeometry()));

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  velocity_arrow_radius_property_ = std::make_unique<rviz::FloatProperty>("Velocity Arrow Radius", 0.05, "Radius of the arrow's shaft, in meters.", 
          this, SLOT(updateVelocityArrowGeometry()));

  velocity_arrow_head_length_property_ = std::make_unique<rviz::FloatProperty>("Velocity Arrow Head Length", 0.3, "Length of the arrow's head, in meters.", 
          this, SLOT(updateVelocityArrowGeometry()));

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  velocity_arrow_head_radius_property_ = std::make_unique<rviz::FloatProperty>("Velocity Arrow Head Radius", 0.1, "Radius of the arrow's head, in meters.", 
          this, SLOT(updateVelocityArrowGeometry()));


  /* Pose Covariance */
  pose_covariance_property_ = std::make_unique<rviz::CovarianceProperty>("Covariance Pose", true, 
        "Whether or not the covariances of the pose should be shown.", this, SLOT(queueRender()));


  /* Velocity Covariance */
  velocity_covariance_property_ = std::make_unique<rviz::CovarianceProperty>("Covariance Velocity", true, 
        "Whether or not the covariances of velocity should be shown.", this, SLOT(queueRender()));
  if (velocity_covariance_property_->childAt(0)) velocity_covariance_property_->childAt(0)->hide(); //Hide Position
  if (velocity_covariance_property_->childAt(1)) velocity_covariance_property_->childAt(1)->hide(); //Hide Orientation

  velocity_covariance_alpha_ = std::make_unique<rviz::FloatProperty>("Covariance Velocity Alpha", 0.3, "Amount of transparency to apply to the Covariance Velocity.", 
          this, SLOT(updateVelocityColorAndAlpha()));
  velocity_covariance_alpha_->setMin(0);
  velocity_covariance_alpha_->setMax(1);


  /* Text ID */
  id_text_bool_property_ = std::make_unique<rviz::BoolProperty>("Text ID", true, "Whether or not the text id should be shown.", 
          this, SLOT(updateTextIDVisibility()));

  id_text_color_property_ = std::make_unique<rviz::ColorProperty>("Text ID Color", QColor(255, 255, 255), "Color to draw the text ids.", 
          this, SLOT(updateTextIDColorAndAlpha()));

  id_text_alpha_property_ = std::make_unique<rviz::FloatProperty>("Text ID Alpha", 1, "Amount of transparency to apply to the text id.", 
          this, SLOT(updateTextIDColorAndAlpha()));
  id_text_alpha_property_->setMin(0);
  id_text_alpha_property_->setMax(1);
  
  id_text_size_property_ = std::make_unique<rviz::FloatProperty>("Text ID Size", 0.5, "Height of letters, in meters.", 
          this, SLOT(updateTextIDSize()));

  id_text_shift_x_property_ = std::make_unique<rviz::FloatProperty>("Text ID Shift X", 0, "Length of the X axis shift, in meters.", 
          this, SLOT(updateTextIDShift()));

  id_text_shift_y_property_ = std::make_unique<rviz::FloatProperty>("Text ID Shift Y", 0.5, "Length of the Y axis shift, in meters.", 
          this, SLOT(updateTextIDShift()));

  id_text_shift_z_property_ = std::make_unique<rviz::FloatProperty>("Text ID Shift Z", 0.5, "Length of the Z axis shift, in meters.", 
          this, SLOT(updateTextIDShift()));        
}


void Display::onInitialize() {
  MFDClass::onInitialize();

  coll_handler_ = std::make_unique<DisplaySelectionHandler>(this, context_);
  for (auto& d : disp_data_) {
    d.arrow_vel_ = boost::make_shared<rviz::Arrow>(scene_manager_, scene_node_, velocity_arrow_length_scale_property_->getFloat(), 
            velocity_arrow_radius_property_->getFloat(), velocity_arrow_head_length_property_->getFloat(), 
            velocity_arrow_head_radius_property_->getFloat());
    // Arrow points in -Z direction, so rotate the orientation before display.
    // TODO: is it safe to change Arrow to point in +X direction?
    d.arrow_vel_->setOrientation(Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

    d.arrow_vel_len_ = 1.0;

    d.axes_pose_ = boost::make_shared<rviz::Axes>(scene_manager_, scene_node_, axes_length_property_->getFloat(), 
            axes_radius_property_->getFloat());

    d.covariance_pose_ = pose_covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

    d.covariance_vel_ = velocity_covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

    d.text_id_ = boost::make_shared<TextID>(scene_manager_, scene_node_);
  }

  updateVelocityColorAndAlpha();
  updateTextIDColorAndAlpha();
  updateVelocityArrowVisibility();
  updateAxesVisibility();
  updateTextIDVisibility();
  updateCovariancePoseVisibility();
  updateCovarianceVelocityVisibility();
}


Display::~Display() {
    pose_covariance_property_->clearVisual();
    velocity_covariance_property_->clearVisual();
}


void Display::onEnable() {
  MFDClass::onEnable();
  updateCovariancePoseVisibility();
  updateCovarianceVelocityVisibility();
}


void Display::updateTextIDColorAndAlpha() {
  Ogre::ColourValue color = id_text_color_property_->getOgreColor();
  color.a                 = id_text_alpha_property_->getFloat();

  for (auto& d : disp_data_) {
    d.text_id_->setColor(color); 
  }

  context_->queueRender();
}


void Display::updateVelocityColorAndAlpha() {
  Ogre::ColourValue color = velocity_color_property_->getOgreColor();
  color.a                 = velocity_arrow_alpha_property_->getFloat();
  Ogre::ColourValue color_cov = color;
  color_cov.a = velocity_covariance_alpha_->getFloat();;

  for (auto& d : disp_data_) {
    d.arrow_vel_->setColor(color); 
    d.covariance_vel_->setPositionColor(color_cov);
    d.covariance_vel_->setOrientationColor(color_cov);
  }

  context_->queueRender();
}


void Display::updateVelocityArrowGeometry() {
  for (auto& d : disp_data_) {
    d.arrow_vel_->set(velocity_arrow_length_scale_property_->getFloat()*d.arrow_vel_len_, velocity_arrow_radius_property_->getFloat(), 
            velocity_arrow_head_length_property_->getFloat(), velocity_arrow_head_radius_property_->getFloat());
  }
  context_->queueRender();
}


void Display::updateAxesGeometry() {
  for (auto& d : disp_data_) {
    d.axes_pose_->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
  }
  context_->queueRender();
}


void Display::updateTextIDSize() {
  for (auto& d : disp_data_) {
    d.text_id_->setSize(id_text_size_property_->getFloat());
  }
  context_->queueRender();
}


void Display::updateTextIDShift() {
  for (auto& d : disp_data_) {
    Ogre::Vector3 tmp = d.text_id_->getPosition();
    d.text_id_->setPosition(tmp, id_text_shift_x_property_->getFloat(), id_text_shift_y_property_->getFloat(),
            id_text_shift_z_property_->getFloat());
  }
  context_->queueRender();
}


void Display::updateVelocityArrowVisibility() {
  bool is_visible = velocity_arrow_bool_property_->getBool() && pose_valid_ && velocity_valid_;

  for (auto& d : disp_data_) {
    d.arrow_vel_->getSceneNode()->setVisible(is_visible);
  }
}


void Display::updateAxesVisibility() {
  bool is_visible = axes_bool_property_->getBool() && pose_valid_;

  for (auto& d : disp_data_) {
    d.axes_pose_->getSceneNode()->setVisible(is_visible);
  }
}


void Display::updateTextIDVisibility() {
  bool is_visible = id_text_bool_property_->getBool() && pose_valid_;

  for (auto& d : disp_data_) {
    d.text_id_->getSceneNode()->setVisible(is_visible);
  }
}


void Display::updateCovariancePoseVisibility() {
  if (!pose_valid_) {
    for (auto& d : disp_data_) {
      d.covariance_pose_->setVisible(false);
    }
  } else {
    pose_covariance_property_->updateVisibility();
  }
}


void Display::updateCovarianceVelocityVisibility() {
  if (!(velocity_valid_ && pose_valid_)) {
    for (auto& d : disp_data_) {
      d.covariance_vel_->setVisible(false);
    }
  } else {
    velocity_covariance_property_->updateVisibility();
  }
}


void Display::reset() {
  MFDClass::reset();
  pose_valid_ = false;
  velocity_valid_ = false;
  updateCovariancePoseVisibility();
  updateCovarianceVelocityVisibility();
}


void Display::processMessage(const mrs_msgs::TrackArrayStamped::ConstPtr& message) {
  disp_data_.clear();

  while (pose_covariance_property_->sizeVisual() > 0) {
    pose_covariance_property_->popFrontVisual();
  }

  while (velocity_covariance_property_->sizeVisual() > 0) {
    velocity_covariance_property_->popFrontVisual();
  }

  coll_handler_->setMessage(message);

  for (int i = 0; i < (int)(message->tracks.size()); i++) {

    /* Set pose */
    if (!rviz::validateFloats(message->tracks[i].position) || !rviz::validateFloats(message->tracks[i].position_covariance)) {
      setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point position values (nans or infs)");
      return;
    }

    geometry_msgs::Pose pose_msg;
    pose_msg.position = message->tracks[i].position;
    pose_msg.orientation = message->tracks[i].orientation;

    if (!rviz::validateQuaternions(pose_msg)) {
      ROS_WARN_ONCE_NAMED("quaternions",
                          "Track '%s' contains unnormalized quaternions. "
                          "This warning will only be output once but may be true for others; "
                          "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                          qPrintable(getName()));
      ROS_DEBUG_NAMED("quaternions", "Track '%s' contains unnormalized quaternions.", qPrintable(getName()));
    }

    Ogre::Vector3    position_pose;
    Ogre::Quaternion orientation_pose;
    if (!context_->getFrameManager()->transform(message->header, pose_msg, position_pose, orientation_pose)) {
      ROS_ERROR("Error transforming track '%s' from frame '%s' to frame '%s'", qPrintable(getName()), message->header.frame_id.c_str(),
                qPrintable(fixed_frame_));
      return;
    }

    pose_valid_ = true;

    disp_data_.push_back(display_object());
    auto& d = disp_data_.back();

    d.axes_pose_ = boost::make_shared<rviz::Axes>(scene_manager_, scene_node_, axes_length_property_->getFloat(), 
            axes_radius_property_->getFloat());

    d.axes_pose_->setPosition(position_pose);
    d.axes_pose_->setOrientation(orientation_pose);


    /* Set pose covariance */
    d.covariance_pose_ = pose_covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

    d.covariance_pose_->setPosition(position_pose);
    d.covariance_pose_->setOrientation(orientation_pose);

    geometry_msgs::PoseWithCovariance pwc_msg;
    boost::array<double, 36> tmp_cov_pose{0}; //36 is size of covariance array from geometry_msgs/PoseWithCovariance

    int c = 0;
    for (int j = 0; j < 9; ++j) { //9 is position_covariance size
      if (j > 0 && j%3 == 0) c += 3;
      tmp_cov_pose[j+c] = message->tracks[i].position_covariance[j];
    }

    c = 21;
    for (int j = 0; j < 9; ++j) { //9 is orientation_covariance size
      if (j > 0 && j%3 == 0) c += 3;
      tmp_cov_pose[j+c] = message->tracks[i].orientation_covariance[j];
    }

    pwc_msg.covariance = tmp_cov_pose;
    pwc_msg.pose       = pose_msg;
    d.covariance_pose_->setCovariance(pwc_msg);


    /* Set velocity */
    if (!rviz::validateFloats(message->tracks[i].velocity) || !rviz::validateFloats(message->tracks[i].velocity_covariance)) {
      setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point velocity values (nans or infs)");
      return;
    }

    velocity_valid_ = true;

    const geometry_msgs::Vector3 tmp = message->tracks[i].velocity;
    Ogre::Vector3 direction = Ogre::Vector3(tmp.x, tmp.y, tmp.z);
    
    d.arrow_vel_len_ = direction.length();
    float shaft_length = velocity_arrow_length_scale_property_->getFloat() * d.arrow_vel_len_;
    d.arrow_vel_ = boost::make_shared<rviz::Arrow>(scene_manager_, scene_node_, shaft_length, velocity_arrow_radius_property_->getFloat(), 
            velocity_arrow_head_length_property_->getFloat(), velocity_arrow_head_radius_property_->getFloat());

    d.arrow_vel_->setPosition(position_pose);
    d.arrow_vel_->setDirection(direction);


    /* Set velocity covariance */
    d.covariance_vel_ = velocity_covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

    d.covariance_vel_->setPosition(position_pose);
    d.covariance_vel_->setOrientation(orientation_pose);

    geometry_msgs::PoseWithCovariance vel_msg;
    boost::array<double, 36> tmp_cov_vel{0}; //36 is size of covariance array from geometry_msgs/PoseWithCovariance

    c = 0;
    for (int j = 0; j < 9; ++j) { //9 is velocity_covariance size
      if (j > 0 && j%3 == 0) c += 3;
      tmp_cov_vel[j+c] = message->tracks[i].velocity_covariance[j];
    }

    vel_msg.covariance = tmp_cov_vel;
    vel_msg.pose       = pose_msg;
    d.covariance_vel_->setCovariance(vel_msg);


    /* Create text marker with id */
    d.text_id_ = boost::make_shared<TextID>(scene_manager_, scene_node_);
    d.text_id_->setPosition(position_pose, id_text_shift_x_property_->getFloat(), id_text_shift_y_property_->getFloat(),
            id_text_shift_z_property_->getFloat());

    Ogre::ColourValue color = id_text_color_property_->getOgreColor();
    color.a                 = id_text_alpha_property_->getFloat();
    d.text_id_->setText(std::to_string(message->tracks[i].id), color, id_text_size_property_->getFloat());


    coll_handler_->addTrackedObjects(d.arrow_vel_->getSceneNode());
    coll_handler_->addTrackedObjects(d.axes_pose_->getSceneNode());
    coll_handler_->addTrackedObjects(d.covariance_pose_->getPositionSceneNode());
    coll_handler_->addTrackedObjects(d.covariance_pose_->getOrientationSceneNode());
    coll_handler_->addTrackedObjects(d.covariance_vel_->getPositionSceneNode());
    coll_handler_->addTrackedObjects(d.covariance_vel_->getOrientationSceneNode());
    coll_handler_->addTrackedObjects(d.text_id_->getSceneNode());
    
    context_->queueRender();
  }

  updateCovariancePoseVisibility();
  updateCovarianceVelocityVisibility();
  updateVelocityArrowVisibility();
  updateAxesVisibility();
  updateTextIDVisibility();
  updateVelocityColorAndAlpha();
}


/* Class Text ID */
TextID::TextID(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node){
  scene_manager_ = scene_manager;
  if (!parent_node) {
     parent_node = scene_manager_->getRootSceneNode();
  }
 
  scene_node_ = parent_node->createChildSceneNode();

  text_ = nullptr;
}

TextID::~TextID(){
  if (text_) delete text_;
  scene_manager_->destroySceneNode(scene_node_->getName());
}

void TextID::setPosition(Ogre::Vector3& position, float x, float y, float z){
  position_ = position; //saves original position without shift
  Ogre::Vector3 tmp = position;
  tmp.x += x;
  tmp.y += y;
  tmp.z += z;

  scene_node_->setPosition(tmp);
}

void TextID::setSize(const float size){
  text_->setCharacterHeight(size);
}

void TextID::setColor(const Ogre::ColourValue color){
  text_->setColor(color);
}

void TextID::setText(const std::string& text, Ogre::ColourValue color, float size){
  if (!text_) {
    text_ = new rviz::MovableText(text);
    text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
    scene_node_->attachObject(text_);
  }

  scene_node_->setVisible(true);
  text_->setCharacterHeight(size); 
  text_->setColor(color); 

  text_->setCaption(text);
}

Ogre::SceneNode* TextID::getSceneNode(){
  return scene_node_;
}

Ogre::Vector3 TextID::getPosition(){
  return position_;
}

}  // namespace track_array

}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::track_array::Display, rviz::Display)