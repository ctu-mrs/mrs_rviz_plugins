/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
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

#ifndef POSE_WITH_COVARIANCE_DISPLAY_H
#define POSE_WITH_COVARIANCE_DISPLAY_H

#include <boost/shared_ptr.hpp>

/* #include <geometry_msgs/PoseWithCovarianceStamped.h> */
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include "rviz/message_filter_display.h"
#include "rviz/selection/forwards.h"

namespace rviz
{

  class Arrow;
  class Axes;
  class ColorProperty;
  class EnumProperty;
  class FloatProperty;
  class BoolProperty;
  class Shape;

  class CovarianceVisual;
  class CovarianceProperty;
}

namespace mrs_rviz_plugins
{
  struct PWC_display_object{
    rviz::Arrow* arrow_;
    rviz::Axes* axes_;
    boost::shared_ptr<rviz::CovarianceVisual> covariance_;
  };

class PoseWithCovarianceArrayDisplaySelectionHandler;
typedef boost::shared_ptr<PoseWithCovarianceArrayDisplaySelectionHandler> PoseWithCovarianceArrayDisplaySelectionHandlerPtr;

/** @brief Displays the pose from a geometry_msgs::PoseWithCovarianceStamped message. */
class PoseWithCovarianceArrayDisplay: public rviz::MessageFilterDisplay<mrs_msgs::PoseWithCovarianceArrayStamped>
  {
    Q_OBJECT
    public:
      enum Shape
      {
        Arrow,
        Axes,
      };

  PoseWithCovarianceArrayDisplay();
  virtual ~PoseWithCovarianceArrayDisplay();

  virtual void onInitialize();
  virtual void reset();

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  virtual void onEnable();

private Q_SLOTS:
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateShapeChoice();
  void updateAxisGeometry();
  void updateArrowGeometry();

private:
  void clear();

  virtual void processMessage( const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& message );

  std::vector<PWC_display_object> disp_data;

  bool pose_valid_;
  PoseWithCovarianceArrayDisplaySelectionHandlerPtr coll_handler_;

  rviz::EnumProperty* shape_property_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;

  rviz::FloatProperty* head_radius_property_;
  rviz::FloatProperty* head_length_property_;
  rviz::FloatProperty* shaft_radius_property_;
  rviz::FloatProperty* shaft_length_property_;

  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;

  rviz::CovarianceProperty* covariance_property_;

  friend class PoseWithCovarianceArrayDisplaySelectionHandler;
};

} // namespace mrs_rviz_plugins

#endif // POSE_WITH_COVARIANCE_DISPLAY_H
