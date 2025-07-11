/* includes //{ */

#include <mrs_rviz_plugins/sphere/display.h>

//}

namespace mrs_rviz_plugins
{
  namespace sphere
  {

    /* Display::Display() //{ */

    // Display::Display()
    // {
    //   std::cout << "calling empty constructor" << std::endl;
    // }

    //}

    // After the top-level rviz_common::properties::Display::initialize() does its own setup,
    // it calls the subclass's onInitialize() function.  This is where we
    // instantiate all the workings of the class.  We make sure to also
    // call our immediate super-class's onInitialize() function, since it
    // does important stuff setting up the message filter.
    //
    //  Note that "MFDClass" is a typedef of
    // ``MessageFilterDisplay<message type>``, to save typing that long
    // templated class name every time you need to refer to the
    // superclass.
    void Display::onInitialize()
    {
      color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Color",
                                                                                 QColor(255, 0, 255),
                                                                                 "Color to draw the shapes.",
                                                                                 this,
                                                                                 SLOT(updateColorAndAlpha()));

      alpha_property_ = std::make_unique<rviz_common::properties::FloatProperty>("Alpha",
                                                                                 1.0,
                                                                                 "0 is fully transparent, 1.0 is fully "
                                                                                 "opaque.",
                                                                                 this,
                                                                                 SLOT(updateColorAndAlpha()));
      alpha_property_->setMin(0.0);
      alpha_property_->setMax(1.0);

      draw_dynamic_property_ = std::make_unique<rviz_common::properties::BoolProperty>("Draw dynamic circle",
                                                                                       true,
                                                                                       "Whether the circle always "
                                                                                       "facing the user should "
                                                                                       "be drawn or not.",
                                                                                       this,
                                                                                       SLOT(updateDrawDynamic()));
      draw_static_property_  = std::make_unique<rviz_common::properties::BoolProperty>("Draw static circles",
                                                                                      true,
                                                                                      "Whether the circles always "
                                                                                       "oriented with the "
                                                                                       "message's coordinate frame axes "
                                                                                       "should be drawn or "
                                                                                       "not.",
                                                                                      this,
                                                                                      SLOT(updateDrawStatic()));

      dashed_dynamic_property_ = std::make_unique<rviz_common::properties::BoolProperty>("Dash dynamic circle",
                                                                                         true,
                                                                                         "Whether the circle always "
                                                                                         "facing the user "
                                                                                         "should be dashed or not.",
                                                                                         this,
                                                                                         SLOT(updateDashedDynamic()));
      dashed_static_property_  = std::make_unique<rviz_common::properties::BoolProperty>("Dash static circles",
                                                                                        true,
                                                                                        "Whether the circles always "
                                                                                         "oriented with the "
                                                                                         "message's coordinate frame "
                                                                                         "axes should be dashed "
                                                                                         "or not.",
                                                                                        this,
                                                                                        SLOT(updateDashedStatic()));


      visual_ = std::make_unique<Visual>(this->context_, this->scene_node_);
      MFDClass::onInitialize();
    }

    // Display::~Display() {}

    // Clear the visual by deleting its object.
    void Display::reset()
    {
      MFDClass::reset();
      visual_ = nullptr;
    }

    // Set the current color and alpha values the visual.
    void Display::updateColorAndAlpha()
    {
      const float             alpha = alpha_property_->getFloat();
      const Ogre::ColourValue color = color_property_->getOgreColor();
      if (visual_)
        visual_->setColor(color.r, color.g, color.b, alpha);
    }

    void Display::updateDrawDynamic()
    {
      const bool draw = draw_dynamic_property_->getBool();
      if (visual_)
        visual_->setDrawDynamic(draw);
    }

    void Display::updateDrawStatic()
    {
      const bool draw = draw_static_property_->getBool();
      if (visual_)
        visual_->setDrawStatic(draw);
    }

    void Display::updateDashedDynamic()
    {
      const bool draw = dashed_dynamic_property_->getBool();
      if (visual_)
        visual_->setDashedDynamic(draw);
    }

    void Display::updateDashedStatic()
    {
      const bool draw = dashed_static_property_->getBool();
      if (visual_)
        visual_->setDashedStatic(draw);
    }

    // This is our callback to handle an incoming message.
    void Display::processMessage(mrs_msgs::msg::Sphere::ConstSharedPtr msg)
    {
      // Here we call the rviz_common::properties::FrameManager to get the transform from the
      // fixed frame to the frame in the header of this message.  If
      // it fails, we can't do anything else so we return.
      Ogre::Quaternion orientation;
      Ogre::Vector3    position;
      if (!this->context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                           msg->header.stamp,
                                                           position,
                                                           orientation)) {
        RVIZ_COMMON_LOG_INFO_STREAM("[Display]: Error transforming from frame '"
                                    << msg->header.frame_id.c_str() << "' to frame '" << qPrintable(this->fixed_frame_)
                                    << "'");
        return;
      }

      if (!visual_) {
        visual_ = std::make_unique<Visual>(this->context_, this->scene_node_);
        // initialize the set values
        updateColorAndAlpha();
        updateDrawDynamic();
        updateDrawStatic();
        updateDashedDynamic();
        updateDashedStatic();
      }

      // Now set or update the contents of the chosen visual.
      visual_->setFramePosition(position);
      visual_->setFrameOrientation(orientation);
      visual_->setMessage(msg);
    }

  }  // namespace sphere

}  // end namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::sphere::Display,
                       rviz_common::Display)
