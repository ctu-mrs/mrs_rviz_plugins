// clang: MatousFormat

/* includes //{ */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <sphere/visual.h>
#include <sphere/display.h>

//}

namespace mrs_rviz_plugins
{

  namespace sphere
  {

    /* Display::Display() //{ */

    Display::Display()
    {
    }

    //}

    // After the top-level rviz::Display::initialize() does its own setup,
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
      MFDClass::onInitialize();
    }

    Display::~Display()
    {
    }

    // Clear the visuals by deleting their objects.
    void Display::reset()
    {
      MFDClass::reset();
      visual_ = nullptr;
    }

    // This is our callback to handle an incoming message.
    void Display::processMessage(const mrs_msgs::Sphere::ConstPtr& msg)
    {
      // Here we call the rviz::FrameManager to get the transform from the
      // fixed frame to the frame in the header of this bumper_ message.  If
      // it fails, we can't do anything else so we return.
      Ogre::Quaternion orientation;
      Ogre::Vector3 position;
      if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
      {
        ROS_DEBUG("[Visual]: Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
      }

      if (!visual_)
        visual_ = boost::make_shared<Visual>(context_, scene_node_);

      // Now set or update the contents of the chosen visual.
      visual_->setFramePosition(position);
      visual_->setFrameOrientation(orientation);
      visual_->setMessage(msg);
    }

  }  // namespace sphere

}  // end namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::sphere::Display, rviz::Display)
