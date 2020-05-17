#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
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

struct display_property
{
  rviz::StringProperty*     frame_property_;
  rviz::VectorProperty*     position_property_;
  rviz::QuaternionProperty* orientation_property_;
  rviz::VectorProperty*     covariance_position_property_;
  rviz::VectorProperty*     covariance_orientation_property_;
};

class DisplaySelectionHandler : public rviz::SelectionHandler {

public:
  DisplaySelectionHandler(Display* display, rviz::DisplayContext* context) : SelectionHandler(context), display_(display) {
  }

  void createProperties(const rviz::Picked& obj, rviz::Property* parent_property) {
    /* stored_properties */
    /* Property* cat = new Property( "Pose " + display_->getName(), QVariant(), "", parent_property ); */
    /* properties_.push_back( cat ); */

    /* frame_property_ = new StringProperty( "Frame", "", "", cat ); */
    /* frame_property_->setReadOnly( true ); */

    /* position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO, "", cat ); */
    /* position_property_->setReadOnly( true ); */

    /* orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY, "", cat ); */
    /* orientation_property_->setReadOnly( true ); */

    /* covariance_position_property_ = new VectorProperty( "Covariance Position", Ogre::Vector3::ZERO, "", cat ); */
    /* covariance_position_property_->setReadOnly( true ); */

    /* covariance_orientation_property_ = new VectorProperty( "Covariance Orientation", Ogre::Vector3::ZERO, "", cat ); */
    /* covariance_orientation_property_->setReadOnly( true ); */
  }

  void getAABBs(const rviz::Picked& obj, rviz::V_AABB& aabbs) {
    for (int i = 0; i < stored_properties.size(); i++) {
      if (display_->pose_valid_) {
        if (display_->shape_property_->getOptionInt() == Display::Arrow) {
          aabbs.push_back(display_->disp_data[i].arrow_->getHead()->getEntity()->getWorldBoundingBox());
          aabbs.push_back(display_->disp_data[i].arrow_->getShaft()->getEntity()->getWorldBoundingBox());
        } else {
          aabbs.push_back(display_->disp_data[i].axes_->getXShape()->getEntity()->getWorldBoundingBox());
          aabbs.push_back(display_->disp_data[i].axes_->getYShape()->getEntity()->getWorldBoundingBox());
          aabbs.push_back(display_->disp_data[i].axes_->getZShape()->getEntity()->getWorldBoundingBox());
        }

        if (display_->covariance_property_->getBool()) {
          if (display_->covariance_property_->getPositionBool()) {
            aabbs.push_back(display_->disp_data[i].covariance_->getPositionShape()->getEntity()->getWorldBoundingBox());
          }
          if (display_->covariance_property_->getOrientationBool()) {
            aabbs.push_back(display_->disp_data[i].covariance_->getOrientationShape(rviz::CovarianceVisual::kRoll)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(display_->disp_data[i].covariance_->getOrientationShape(rviz::CovarianceVisual::kPitch)->getEntity()->getWorldBoundingBox());
            aabbs.push_back(display_->disp_data[i].covariance_->getOrientationShape(rviz::CovarianceVisual::kYaw)->getEntity()->getWorldBoundingBox());
          }
        }
      }
    }
  }

  void setMessage(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& message) {
    // properties_.size() should only be > 0 after createProperties()
    // and before destroyProperties(), during which frame_property_,
    // position_property_, and orientation_property_ should be valid
    // pointers.
    if (properties_.size() > 0) {
      stored_properties.clear();
      for (int i = 0; i < (int)(message->poses.size()); i++) {
        stored_properties.push_back(display_property());
        stored_properties.back().frame_property_->setStdString(message->header.frame_id);
        stored_properties.back().position_property_->setVector(
            Ogre::Vector3(message->poses[i].pose.position.x, message->poses[i].pose.position.y, message->poses[i].pose.position.z));
        stored_properties.back().orientation_property_->setQuaternion(
            Ogre::Quaternion(message->poses[i].pose.orientation.w, message->poses[i].pose.orientation.x, message->poses[i].pose.orientation.y,
                             message->poses[i].pose.orientation.z));
        stored_properties.back().covariance_position_property_->setVector(
            Ogre::Vector3(message->poses[i].covariance[0 + 0 * 6], message->poses[i].covariance[1 + 1 * 6], message->poses[i].covariance[2 + 2 * 6]));

        stored_properties.back().covariance_orientation_property_->setVector(
            Ogre::Vector3(message->poses[i].covariance[3 + 3 * 6], message->poses[i].covariance[4 + 4 * 6], message->poses[i].covariance[5 + 5 * 6]));
      }
    }
  }

private:
  std::vector<display_property> stored_properties;
  Display*                      display_;
};

Display::Display() : pose_valid_(false) {
  std::cout << "SHIT BOYZ!";

  shape_property_ = new rviz::EnumProperty("Shape", "Arrow", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", Arrow);
  shape_property_->addOption("Axes", Axes);

  color_property_ = new rviz::ColorProperty("Color", QColor(255, 25, 0), "Color to draw the arrow.", this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1, "Amount of transparency to apply to the arrow.", this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  shaft_length_property_ = new rviz::FloatProperty("Shaft Length", 1, "Length of the arrow's shaft, in meters.", this, SLOT(updateArrowGeometry()));

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = new rviz::FloatProperty("Shaft Radius", 0.05, "Radius of the arrow's shaft, in meters.", this, SLOT(updateArrowGeometry()));

  head_length_property_ = new rviz::FloatProperty("Head Length", 0.3, "Length of the arrow's head, in meters.", this, SLOT(updateArrowGeometry()));

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = new rviz::FloatProperty("Head Radius", 0.1, "Radius of the arrow's head, in meters.", this, SLOT(updateArrowGeometry()));

  axes_length_property_ = new rviz::FloatProperty("Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  axes_radius_property_ = new rviz::FloatProperty("Axes Radius", 0.1, "Radius of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  covariance_property_ =
      new rviz::CovarianceProperty("Covariance", true, "Whether or not the covariances of the messages should be shown.", this, SLOT(queueRender()));
}

void Display::onInitialize() {
  MFDClass::onInitialize();

  coll_handler_.reset(new DisplaySelectionHandler(this, context_));
  for (auto& d : disp_data) {
    d.arrow_ = new rviz::Arrow(scene_manager_, scene_node_, shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(),
                               head_length_property_->getFloat(), head_radius_property_->getFloat());
    // Arrow points in -Z direction, so rotate the orientation before display.
    // TODO: is it safe to change Arrow to point in +X direction?
    d.arrow_->setOrientation(Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

    d.axes_ = new rviz::Axes(scene_manager_, scene_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat());

    d.covariance_ = covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

    coll_handler_->addTrackedObjects(d.arrow_->getSceneNode());
    coll_handler_->addTrackedObjects(d.axes_->getSceneNode());
    coll_handler_->addTrackedObjects(d.covariance_->getPositionSceneNode());
    coll_handler_->addTrackedObjects(d.covariance_->getOrientationSceneNode());
  }
  updateShapeChoice();
  updateColorAndAlpha();
}

Display::~Display() {
  if (initialized()) {
    while (disp_data.size() > 0) {
      delete disp_data.back().arrow_;
      delete disp_data.back().axes_;
      disp_data.pop_back();
    }
  }
}

void Display::onEnable() {
  MFDClass::onEnable();
  updateShapeVisibility();
}

void Display::updateColorAndAlpha() {
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a                 = alpha_property_->getFloat();

  for (auto& d : disp_data) {
    d.arrow_->setColor(color);
  }

  context_->queueRender();
}

void Display::updateArrowGeometry() {
  for (auto& d : disp_data) {
    d.arrow_->set(shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(), head_length_property_->getFloat(), head_radius_property_->getFloat());
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
  bool use_arrow = (shape_property_->getOptionInt() == Arrow);

  color_property_->setHidden(!use_arrow);
  alpha_property_->setHidden(!use_arrow);
  shaft_length_property_->setHidden(!use_arrow);
  shaft_radius_property_->setHidden(!use_arrow);
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
      d.arrow_->getSceneNode()->setVisible(false);
      d.axes_->getSceneNode()->setVisible(false);
      d.covariance_->setVisible(false);
    }
  } else {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    for (auto& d : disp_data) {
      d.arrow_->getSceneNode()->setVisible(use_arrow);
      d.axes_->getSceneNode()->setVisible(!use_arrow);
    }
    covariance_property_->updateVisibility();
  }
}

void Display::processMessage(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& message) {
  while (disp_data.size() > 0) {
    delete disp_data.back().arrow_;
    delete disp_data.back().axes_;
    disp_data.pop_back();
  }
  while (covariance_property_->sizeVisual() > 0) {
    covariance_property_->popFrontVisual();
  }

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
    updateShapeVisibility();

    disp_data.push_back(PWC_display_object());

    disp_data[i].arrow_ = new rviz::Arrow(scene_manager_, scene_node_, shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(),
                                          head_length_property_->getFloat(), head_radius_property_->getFloat());

    disp_data[i].axes_ = new rviz::Axes(scene_manager_, scene_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat());

    disp_data[i].covariance_ = covariance_property_->createAndPushBackVisual(scene_manager_, scene_node_);

    disp_data[i].axes_->setPosition(position);
    disp_data[i].axes_->setOrientation(orientation);

    disp_data[i].arrow_->setPosition(position);
    disp_data[i].arrow_->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

    disp_data[i].covariance_->setPosition(position);
    disp_data[i].covariance_->setOrientation(orientation);
    geometry_msgs::PoseWithCovariance pwc;
    pwc.covariance = message->poses[i].covariance;
    pwc.pose       = message->poses[i].pose;
    disp_data[i].covariance_->setCovariance(pwc);

    coll_handler_->setMessage(message);

    context_->queueRender();
  }
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
