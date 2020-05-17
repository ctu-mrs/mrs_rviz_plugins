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

#include <bumper/visual.h>
#include <bumper/display.h>

//}

namespace mrs_rviz_plugins
{

  namespace bumper
  {

    Display::Display()
    {
      color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204), "Color to draw the shapes.", this, SLOT(updateColorAndAlpha()));

      alpha_property_ = new rviz::FloatProperty("Alpha", 0.1, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));
      alpha_property_->setMin(0.0);
      alpha_property_->setMax(1.0);

      collision_colorize_property_ =
          new rviz::BoolProperty("Colorize collisions", true, "If true, sectors with obstacles closer than Collision threshold will be colored differently.",
                                 this, SLOT(updateCollisions()));

      horizontal_collision_threshold_property_ = new rviz::FloatProperty(
          "Horizontal collision threshold", 1.0, "If an obstacle is closer than this threshold, the respective sector is colored differently.", this,
          SLOT(updateCollisions()));

      vertical_collision_threshold_property_ = new rviz::FloatProperty(
          "Vertical collision threshold", 1.0, "If an obstacle is closer than this threshold, the respective sector is colored differently.", this,
          SLOT(updateCollisions()));

      collision_color_property_ =
          new rviz::ColorProperty("Collision color", QColor(255, 0, 0), "Color to draw sectors with collision.", this, SLOT(updateCollisions()));

      collision_alpha_property_ =
          new rviz::FloatProperty("Collision alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateCollisions()));

      history_length_property_ = new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.", this, SLOT(updateHistoryLength()));
      history_length_property_->setMin(1);
      history_length_property_->setMax(100000);

      display_mode_property_ = new rviz::EnumProperty("Display mode", "sensor types", "How to display the bumper message.", this, SLOT(updateDisplayMode()));
      display_mode_property_->addOptionStd("whole sectors", Visual::display_mode_t::WHOLE_SECTORS);
      display_mode_property_->addOptionStd("sensor types", Visual::display_mode_t::SENSOR_TYPES);
      show_undetected_property_ = new rviz::BoolProperty("Show undetected obstacles", true,
                                                         "Whether to show sectors, corresponding to no obstacle detection (might clutter the draw space).",
                                                         this, SLOT(updateShowUndetected()));
      show_no_data_property_ = new rviz::BoolProperty("Show sectors with no data", false, "Whether to show sectors, for which no sensory data is available.",
                                                      this, SLOT(updateShowUndetected()));
    }

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
      updateHistoryLength();
      updateCollisions();
    }

    Display::~Display()
    {
    }

    // Clear the visuals by deleting their objects.
    void Display::reset()
    {
      MFDClass::reset();
      visuals_.clear();
    }

    // Set the current color and alpha values for each visual.
    void Display::updateColorAndAlpha()
    {
      float alpha = alpha_property_->getFloat();
      Ogre::ColourValue color = color_property_->getOgreColor();

      for (size_t i = 0; i < visuals_.size(); i++)
      {
        visuals_[i]->setColor(color.r, color.g, color.b, alpha);
      }
    }

    // Set the current color and alpha values for each visual.
    void Display::updateShowUndetected()
    {
      bool show_undetected = show_undetected_property_->getBool();

      for (size_t i = 0; i < visuals_.size(); i++)
      {
        visuals_[i]->setShowUndetected(show_undetected);
      }
    }

    // Set the number of past visuals to show.
    void Display::updateHistoryLength()
    {
      visuals_.rset_capacity(history_length_property_->getInt());
    }

    // Set the number of past visuals to show.
    void Display::updateDisplayMode()
    {
      int option = display_mode_property_->getOptionInt();

      for (size_t i = 0; i < visuals_.size(); i++)
      {
        visuals_[i]->setDisplayMode((Visual::display_mode_t)option);
      }
    }

    void Display::updateCollisions()
    {
      bool collision_colorize = collision_colorize_property_->getBool();
      float horizontal_collision_threshold = horizontal_collision_threshold_property_->getFloat();
      float vertical_collision_threshold = vertical_collision_threshold_property_->getFloat();
      Ogre::ColourValue color = collision_color_property_->getOgreColor();
      float collision_alpha = collision_alpha_property_->getFloat();

      if (collision_colorize)
      {
        horizontal_collision_threshold_property_->show();
        vertical_collision_threshold_property_->show();
        collision_color_property_->show();
        collision_alpha_property_->show();
      } else
      {
        horizontal_collision_threshold_property_->hide();
        vertical_collision_threshold_property_->hide();
        collision_color_property_->hide();
        collision_alpha_property_->hide();
      }

      for (size_t i = 0; i < visuals_.size(); i++)
      {
        visuals_[i]->setCollisionOptions(collision_colorize, horizontal_collision_threshold, vertical_collision_threshold, color.r, color.g, color.b,
                                         collision_alpha);
      }
    }

    // This is our callback to handle an incoming message.
    void Display::processMessage(const mrs_msgs::ObstacleSectors::ConstPtr& msg)
    {
      // Sanitize the message to prevent Rviz crashes
      if (msg->n_horizontal_sectors != msg->sectors.size() - 2)
      {
        ROS_DEBUG("[Visual]: n_horizontal_sectors (%u) is not equal to length of sectors (%lu)-2 in the ObstacleSectors message!", msg->n_horizontal_sectors,
                  msg->sectors.size());
        return;
      }
      for (const auto cur_len : msg->sectors)
      {
        if (std::isinf(cur_len) || std::isnan(cur_len))
        {
          ROS_DEBUG("[Visual]: Invalid obstacle distance encountered in mrs_msgs::ObstacleSectors message: %.2f, skipping message", cur_len);
          return;
        }
      }

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

      // We are keeping a circular buffer of visual pointers.  This gets
      // the next one, or creates and stores it if the buffer is not full
      boost::shared_ptr<Visual> visual;
      if (visuals_.full())
      {
        visual = visuals_.front();
      } else
      {
        visual = boost::make_shared<Visual>(context_->getSceneManager(), scene_node_);
      }


      float alpha = alpha_property_->getFloat();
      Ogre::ColourValue color = color_property_->getOgreColor();
      visual->setColor(color.r, color.g, color.b, alpha);

      bool collision_colorize = collision_colorize_property_->getBool();
      float horizontal_collision_threshold = horizontal_collision_threshold_property_->getFloat();
      float vertical_collision_threshold = vertical_collision_threshold_property_->getFloat();
      Ogre::ColourValue collision_color = collision_color_property_->getOgreColor();
      float collision_alpha = collision_alpha_property_->getFloat();
      visual->setCollisionOptions(collision_colorize, horizontal_collision_threshold, vertical_collision_threshold, collision_color.r, collision_color.g,
                                  collision_color.b, collision_alpha);

      int option = display_mode_property_->getOptionInt();
      visual->setDisplayMode((Visual::display_mode_t)option);

      bool show_undetected = show_undetected_property_->getBool();
      visual->setShowUndetected(show_undetected);

      bool show_no_data = show_no_data_property_->getBool();
      visual->setShowNoData(show_no_data);

      // Now set or update the contents of the chosen visual.
      visual->setMessage(msg);
      visual->setFramePosition(position);
      visual->setFrameOrientation(orientation);

      // And send it to the end of the circular buffer
      visuals_.push_back(visual);
    }

  }  // namespace bumper

}  // end namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::bumper::Display, rviz::Display)
