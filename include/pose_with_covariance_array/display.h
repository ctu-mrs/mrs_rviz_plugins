#ifndef POSE_WITH_COVARIANCE_DISPLAY_H
#define POSE_WITH_COVARIANCE_DISPLAY_H

#include <boost/shared_ptr.hpp>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <rviz/message_filter_display.h>
#include <rviz/selection/forwards.h>

#include <covariance/property.h>
#include <covariance/visual.h>

namespace rviz
{
class Arrow;
class Axes;
class ColorProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class Shape;
}  // namespace rviz

namespace mrs_rviz_plugins
{

namespace pose_with_covariance_array
{

struct display_object
{
  boost::shared_ptr<rviz::Arrow> arrow_;
  boost::shared_ptr<rviz::Axes> axes_;
  boost::shared_ptr<mrs_rviz_plugins::covariance::Visual> covariance_;
};

class DisplaySelectionHandler;

class Display : public rviz::MessageFilterDisplay<mrs_msgs::PoseWithCovarianceArrayStamped> {
  Q_OBJECT
public:
  enum Shape
  {
    Arrow,
    Axes,
  };

  Display();
  virtual ~Display();

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

  virtual void processMessage(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& message);

  std::vector<display_object> disp_data;

  bool                       pose_valid_;
  std::unique_ptr<DisplaySelectionHandler> coll_handler_;

  std::unique_ptr<rviz::EnumProperty> shape_property_;

  std::unique_ptr<rviz::ColorProperty> color_property_;
  std::unique_ptr<rviz::FloatProperty> alpha_property_;

  std::unique_ptr<rviz::FloatProperty> head_radius_property_;
  std::unique_ptr<rviz::FloatProperty> head_length_property_;
  std::unique_ptr<rviz::FloatProperty> shaft_radius_property_;
  std::unique_ptr<rviz::FloatProperty> shaft_length_property_;

  std::unique_ptr<rviz::FloatProperty> axes_length_property_;
  std::unique_ptr<rviz::FloatProperty> axes_radius_property_;

  std::unique_ptr<covariance::Property> covariance_property_;

  friend class DisplaySelectionHandler;
};

}  // namespace pose_with_covariance_array

}  // namespace mrs_rviz_plugins

#endif  // POSE_WITH_COVARIANCE_DISPLAY_H
