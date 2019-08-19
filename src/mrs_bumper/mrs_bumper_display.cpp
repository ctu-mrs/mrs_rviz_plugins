// clang: MatousFormat
/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "mrs_bumper/mrs_bumper_visual.h"

#include "mrs_bumper/mrs_bumper_display.h"

namespace mrs_rviz_plugins
{

  // BEGIN_TUTORIAL
  // The constructor must have no arguments, so we can't give the
  // constructor the parameters it needs to fully initialize.
  MRS_Bumper_Display::MRS_Bumper_Display()
  {
    color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204), "Color to draw the shapes.", this, SLOT(updateColorAndAlpha()));

    alpha_property_ = new rviz::FloatProperty("Alpha", 0.1, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    collision_colorize_property_ =
        new rviz::BoolProperty("Colorize collisions", false, "If true, sectors with obstacles closer than Collision threshold will be colored differently.",
                               this, SLOT(updateCollisions()));

    horizontal_collision_threshold_property_ =
        new rviz::FloatProperty("Horizontal collision threshold", 0.4,
                                "If an obstacle is closer than this threshold, the respective sector is colored differently.", this, SLOT(updateCollisions()));

    vertical_collision_threshold_property_ =
        new rviz::FloatProperty("Vertical collision threshold", 0.4,
                                "If an obstacle is closer than this threshold, the respective sector is colored differently.", this, SLOT(updateCollisions()));

    collision_color_property_ =
        new rviz::ColorProperty("Collision color", QColor(255, 0, 0), "Color to draw sectors with collision.", this, SLOT(updateCollisions()));

    collision_alpha_property_ = new rviz::FloatProperty("Collision alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateCollisions()));

    history_length_property_ = new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.", this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(100000);

    display_mode_property_ = new rviz::EnumProperty("Display mode", "whole sectors", "How to display the bumper message.", this, SLOT(updateDisplayMode()));
    display_mode_property_->addOptionStd("whole sectors", MRS_Bumper_Visual::display_mode_t::WHOLE_SECTORS);
    display_mode_property_->addOptionStd("sensor types", MRS_Bumper_Visual::display_mode_t::SENSOR_TYPES);
    show_undetected_property_ = new rviz::BoolProperty("Show undetected obstacles", true,
                                                       "Whether to show sectors, corresponding to no obstacle detection (might clutter the draw space).", this,
                                                       SLOT(updateShowUndetected()));
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
  void MRS_Bumper_Display::onInitialize()
  {
    MFDClass::onInitialize();
    updateHistoryLength();
    updateCollisions();
  }

  MRS_Bumper_Display::~MRS_Bumper_Display()
  {
  }

  // Clear the visuals by deleting their objects.
  void MRS_Bumper_Display::reset()
  {
    MFDClass::reset();
    visuals_.clear();
  }

  // Set the current color and alpha values for each visual.
  void MRS_Bumper_Display::updateColorAndAlpha()
  {
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();

    for (size_t i = 0; i < visuals_.size(); i++)
    {
      visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    }
  }

  // Set the current color and alpha values for each visual.
  void MRS_Bumper_Display::updateShowUndetected()
  {
    bool show_undetected = show_undetected_property_->getBool();

    for (size_t i = 0; i < visuals_.size(); i++)
    {
      visuals_[i]->setShowUndetected(show_undetected);
    }
  }

  // Set the number of past visuals to show.
  void MRS_Bumper_Display::updateHistoryLength()
  {
    visuals_.rset_capacity(history_length_property_->getInt());
  }

  // Set the number of past visuals to show.
  void MRS_Bumper_Display::updateDisplayMode()
  {
    int option = display_mode_property_->getOptionInt();

    for (size_t i = 0; i < visuals_.size(); i++)
    {
      visuals_[i]->setDisplayMode((MRS_Bumper_Visual::display_mode_t)option);
    }
  }

  void MRS_Bumper_Display::updateCollisions()
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
  void MRS_Bumper_Display::processMessage(const mrs_msgs::ObstacleSectors::ConstPtr& msg)
  {
    // Sanitize the message to prevent Rviz crashes
    if (msg->n_horizontal_sectors != msg->sectors.size() - 2)
    {
      ROS_DEBUG("[MRS_Bumper_Visual]: n_horizontal_sectors (%u) is not equal to length of sectors (%lu)-2 in the ObstacleSectors message!",
                msg->n_horizontal_sectors, msg->sectors.size());
      return;
    }
    for (const auto cur_len : msg->sectors)
    {
      if (std::isinf(cur_len) || std::isnan(cur_len))
      {
        ROS_DEBUG("[MRS_Bumper_Visual]: Invalid obstacle distance encountered in mrs_msgs::ObstacleSectors message: %.2f, skipping message", cur_len);
        return;
      }
    }

    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this MRS_Bumper_ message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
    {
      ROS_DEBUG("[MRS_Bumper_Visual]: Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    // We are keeping a circular buffer of visual pointers.  This gets
    // the next one, or creates and stores it if the buffer is not full
    boost::shared_ptr<MRS_Bumper_Visual> visual;
    if (visuals_.full())
    {
      visual = visuals_.front();
    } else
    {
      visual = boost::make_shared<MRS_Bumper_Visual>(context_->getSceneManager(), scene_node_);
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
    visual->setDisplayMode((MRS_Bumper_Visual::display_mode_t)option);

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

}  // end namespace mrs_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::MRS_Bumper_Display, rviz::Display)
// END_TUTORIAL
