#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/validate_quaternions.h>

#include <pose_with_covariance_array/display.h>

#include <covariance/visual.h>
#include <covariance/property.h>


#include <Eigen/Dense>

namespace mrs_rviz_plugins
{

namespace pose_with_covariance_array
{

class DisplaySelectionHandler : public rviz::SelectionHandler {

public:
  DisplaySelectionHandler(Display* display, rviz::DisplayContext* context) : SelectionHandler(context), root_(std::make_unique<rviz::Property>()), display_(display) {}

  void createProperties([[maybe_unused]] const rviz::Picked& obj, [[maybe_unused]] rviz::Property* parent_property)
  {
    properties_.push_back(new rviz::Property(display_->getName(), QVariant(), display_->getDescription(), parent_property));
    rviz::Property*& root = properties_.back();
    rviz::Property* tmp = new rviz::StringProperty("frame ID", (message_->header.frame_id).c_str(), "name of the frame", root);
    tmp->setReadOnly(true);

    for (const auto& cur_pose : message_->poses)
    {
      rviz::Property* sub_root = new rviz::Property(("PoseWithCovariance " + std::to_string(cur_pose.id)).c_str(), QVariant(), "position and orientation with covariance", root);
      
      tmp = new rviz::IntProperty("ID", cur_pose.id, "ID", sub_root);
      tmp->setReadOnly(true);
      tmp = new rviz::VectorProperty("position", Ogre::Vector3(cur_pose.pose.position.x, cur_pose.pose.position.y, cur_pose.pose.position.z), "position", sub_root);
      tmp->setReadOnly(true);
      tmp = new rviz::QuaternionProperty("orientation", Ogre::Quaternion(cur_pose.pose.orientation.w, cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z), "orientation", sub_root);
      tmp->setReadOnly(true);
    }
    root_->setParent(parent_property);
  }

  void getAABBs([[maybe_unused]] const rviz::Picked& obj, rviz::V_AABB& aabbs) {

    for (const auto& cur_pose : display_->disp_data)
    {
      if (display_->pose_valid_) {
        if (display_->shape_property_->getOptionInt() == Display::Arrow) {
          aabbs.push_back(cur_pose.arrow_->getHead()->getEntity()->getWorldBoundingBox());
          aabbs.push_back(cur_pose.arrow_->getShaft()->getEntity()->getWorldBoundingBox());
        } else if (display_->shape_property_->getOptionInt() == Display::Axes) {
          aabbs.push_back(cur_pose.axes_->getXShape()->getEntity()->getWorldBoundingBox());
          aabbs.push_back(cur_pose.axes_->getYShape()->getEntity()->getWorldBoundingBox());
          aabbs.push_back(cur_pose.axes_->getZShape()->getEntity()->getWorldBoundingBox());
        }
        /* else { */
        /*   aabbs.push_back(cur_pose.fast_arrow_->getShaft()->getEntity()->getWorldBoundingBox()); */
        /*   aabbs.push_back(cur_pose.fast_arrow_->getHeadL()->getEntity()->getWorldBoundingBox()); */
        /*   aabbs.push_back(cur_pose.fast_arrow_->getHeadR()->getEntity()->getWorldBoundingBox()); */
        /* } */

        if (display_->covariance_property_->getBool()) {
          if (display_->covariance_property_->getPositionBool()) {
            aabbs.push_back(cur_pose.covariance_->getPositionShape()->getEntity()->getWorldBoundingBox());
          }
          if (display_->covariance_property_->getOrientationBool()) {
            aabbs.push_back(
                cur_pose.covariance_->getOrientationShape(mrs_rviz_plugins::covariance::Visual::kRoll)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(
                cur_pose.covariance_->getOrientationShape(mrs_rviz_plugins::covariance::Visual::kPitch)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(
                cur_pose.covariance_->getOrientationShape(mrs_rviz_plugins::covariance::Visual::kYaw)->getEntity()->getWorldBoundingBox());
          }
        }
      }
    }
  }

  void setMessage(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& message)
  {
    message_ = message;
    tracked_objects_.clear();
    /* updateProperties(); */
  }

private:
  std::unique_ptr<rviz::Property> root_;
  mrs_msgs::PoseWithCovarianceArrayStampedConstPtr message_;
  Display* display_;
};

Display::Display() : pose_valid_(false) {
  shape_property_ = std::make_unique<rviz::EnumProperty>("Shape", "FastArrow", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", Arrow);
  shape_property_->addOption("Axes", Axes);
  shape_property_->addOption("FastArrow", FastArrow);

  color_property_ = std::make_unique<rviz::ColorProperty>("Color", QColor(255, 25, 0), "Color to draw the arrow.", this, SLOT(updateColorAndAlpha()));

  alpha_property_ = std::make_unique<rviz::FloatProperty>("Alpha", 1, "Amount of transparency to apply to the arrow.", this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  shaft_length_property_ = std::make_unique<rviz::FloatProperty>("Shaft Length", 1, "Length of the arrow's shaft, in meters.", this, SLOT(updateArrowGeometry()));

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = std::make_unique<rviz::FloatProperty>("Shaft Radius", 0.05, "Radius of the arrow's shaft, in meters.", this, SLOT(updateArrowGeometry()));

  head_length_property_ = std::make_unique<rviz::FloatProperty>("Head Length", 0.3, "Length of the arrow's head, in meters.", this, SLOT(updateArrowGeometry()));

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = std::make_unique<rviz::FloatProperty>("Head Radius", 0.1, "Radius of the arrow's head, in meters.", this, SLOT(updateArrowGeometry()));

  axes_length_property_ = std::make_unique<rviz::FloatProperty>("Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  axes_radius_property_ = std::make_unique<rviz::FloatProperty>("Axes Radius", 0.1, "Radius of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  covariance_property_ =
      std::make_unique<covariance::Property>("Covariance", true, "Whether or not the covariances of the messages should be shown.", this, SLOT(queueRender()));
}

void Display::onInitialize() {
  MFDClass::onInitialize();

  coll_handler_ = std::make_unique<DisplaySelectionHandler>(this, context_);
  for (auto& d : disp_data) {
    d.arrow_ = boost::make_shared<rviz::Arrow>(scene_manager_, scene_node_, shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(),
                               head_length_property_->getFloat(), head_radius_property_->getFloat());
    // Arrow points in -Z direction, so rotate the orientation before display.
    // TODO: is it safe to change Arrow to point in +X direction?
    d.arrow_->setOrientation(Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a                 = alpha_property_->getFloat();
    d.arrow_->setColor(color);

    d.axes_ = boost::make_shared<rviz::Axes>(scene_manager_, scene_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat());

    d.fast_arrow_ = boost::make_shared<rviz::FastArrow>(scene_manager_, scene_node_, shaft_length_property_->getFloat(), head_length_property_->getFloat(),
                                        head_radius_property_->getFloat());
    d.fast_arrow_->setColor(color);

    d.covariance_ = covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

  }
  updateShapeChoice();
  updateColorAndAlpha();
}

Display::~Display()
{
}

void Display::onEnable() {
  MFDClass::onEnable();
  updateShapeVisibility();
}

/* update methods implementations //{ */

void Display::updateColorAndAlpha() {
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a                 = alpha_property_->getFloat();

  for (auto& d : disp_data) {
    d.arrow_->setColor(color);
    d.fast_arrow_->setColor(color);
  }

  context_->queueRender();
}

void Display::updateArrowGeometry() {
  for (auto& d : disp_data) {
    d.arrow_->set(shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(), head_length_property_->getFloat(), head_radius_property_->getFloat());
    d.fast_arrow_->set(shaft_length_property_->getFloat(), head_length_property_->getFloat(), head_radius_property_->getFloat());
  }
  context_->queueRender();
}

void Display::updateAxisGeometry() {
  for (auto& d : disp_data) {
    d.axes_->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
  }
  context_->queueRender();
}

void Display::updateShapeChoice() {
  bool use_mesh_arrow = (shape_property_->getOptionInt() == FastArrow);
  bool use_arrow = use_mesh_arrow || (shape_property_->getOptionInt() == FastArrow);

  color_property_->setHidden(!use_arrow);
  alpha_property_->setHidden(!use_arrow);
  shaft_length_property_->setHidden(!use_arrow);
  shaft_radius_property_->setHidden(!use_mesh_arrow);
  head_length_property_->setHidden(!use_arrow);
  head_radius_property_->setHidden(!use_arrow);

  axes_length_property_->setHidden(use_arrow);
  axes_radius_property_->setHidden(use_arrow);

  updateShapeVisibility();

  context_->queueRender();
}

void Display::updateShapeVisibility() {
  if (!pose_valid_) {
    for (auto& d : disp_data) {
      if (d.arrow_)
        d.arrow_->getSceneNode()->setVisible(false);
      if (d.axes_)
        d.axes_->getSceneNode()->setVisible(false);
      if (d.fast_arrow_)
        d.fast_arrow_->getSceneNode()->setVisible(false);

      d.covariance_->setVisible(false);
    }
  } else {
    /* bool use_arrow = (shape_property_->getOptionInt() == Arrow); */
    for (auto& d : disp_data) {
      switch (shape_property_->getOptionInt()){
        case (Arrow):
          if (d.arrow_)
            d.arrow_->getSceneNode()->setVisible(true);
          if (d.axes_)
            d.axes_->getSceneNode()->setVisible(false);
          if (d.fast_arrow_)
            d.fast_arrow_->getSceneNode()->setVisible(false);
          break;
        case (Axes):
          if (d.arrow_)
            d.arrow_->getSceneNode()->setVisible(false);
          if (d.axes_)
            d.axes_->getSceneNode()->setVisible(true);
          if (d.fast_arrow_)
            d.fast_arrow_->getSceneNode()->setVisible(false);
          break;
        default:
          if (d.arrow_)
            d.arrow_->getSceneNode()->setVisible(false);
          if (d.axes_)
            d.axes_->getSceneNode()->setVisible(false);
          if (d.fast_arrow_)
            d.fast_arrow_->getSceneNode()->setVisible(true);
      }
    }
    covariance_property_->updateVisibility();
  }
}

//}

void Display::processMessage(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& message) {
  disp_data.clear();
  while (covariance_property_->sizeVisual() > 0) {
    covariance_property_->popFrontVisual();
  }

  coll_handler_->setMessage(message);
  for (int i = 0; i < (int)(message->poses.size()); i++) {
    if (!rviz::validateFloats(message->poses[i].pose) || !rviz::validateFloats(message->poses[i].covariance)) {
      setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
      return;
    }

    if (!rviz::validateQuaternions(message->poses[i].pose)) {
      ROS_WARN_ONCE_NAMED("quaternions",
                          "PoseWithCovariance '%s' contains unnormalized quaternions. "
                          "This warning will only be output once but may be true for others; "
                          "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                          qPrintable(getName()));
      ROS_DEBUG_NAMED("quaternions", "PoseWithCovariance '%s' contains unnormalized quaternions.", qPrintable(getName()));
    }

    Ogre::Vector3    position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->transform(message->header, message->poses[i].pose, position, orientation)) {
      ROS_ERROR("Error transforming pose '%s' from frame '%s' to frame '%s'", qPrintable(getName()), message->header.frame_id.c_str(),
                qPrintable(fixed_frame_));
      return;
    }

    pose_valid_ = true;

    disp_data.push_back(display_object());
    auto& d = disp_data.back();

    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a                 = alpha_property_->getFloat();

    switch (shape_property_->getOptionInt()){
      case (Arrow):
        d.arrow_ = boost::make_shared<rviz::Arrow>(scene_manager_, scene_node_, shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(),
            head_length_property_->getFloat(), head_radius_property_->getFloat());

        d.arrow_->setColor(color);

        d.arrow_->setPosition(position);
        d.arrow_->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

        coll_handler_->addTrackedObjects(d.arrow_->getSceneNode());
        break;
      case (Axes):
        d.axes_ = boost::make_shared<rviz::Axes>(scene_manager_, scene_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat());

        d.axes_->setPosition(position);
        d.axes_->setOrientation(orientation);

        coll_handler_->addTrackedObjects(d.axes_->getSceneNode());
        break;
      default:
        d.fast_arrow_ = boost::make_shared<rviz::FastArrow>(scene_manager_, scene_node_, shaft_length_property_->getFloat(), head_length_property_->getFloat(),
            head_radius_property_->getFloat());
        d.fast_arrow_->setColor(color);


        d.fast_arrow_->setPosition(position);
        d.fast_arrow_->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

        coll_handler_->addTrackedObjects(d.fast_arrow_->getSceneNode());
    }
    if ( covariance_property_->getBool()){
      geometry_msgs::PoseWithCovariance pwc;
      pwc.covariance = message->poses[i].covariance;
      pwc.pose       = message->poses[i].pose;
      d.covariance_ = covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);
      d.covariance_->setPosition(position);
      d.covariance_->setOrientation(orientation);
      d.covariance_->setCovariance(pwc);
      coll_handler_->addTrackedObjects(d.covariance_->getPositionSceneNode());
      coll_handler_->addTrackedObjects(d.covariance_->getOrientationSceneNode());
    }

    context_->queueRender();
  }
  updateShapeVisibility();
}

void Display::reset() {
  MFDClass::reset();
  pose_valid_ = false;
  updateShapeVisibility();
}

}  // namespace pose_with_covariance_array

}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::pose_with_covariance_array::Display, rviz::Display)
