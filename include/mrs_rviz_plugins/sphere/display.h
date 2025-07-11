// clang: MatousFormat

#ifndef MRS_RVIZ_PLUGINS_SHPERE_DISPLAY_H
#define MRS_RVIZ_PLUGINS_SHPERE_DISPLAY_H

#include <mrs_rviz_plugins/sphere/visual.h>
#include <memory>
#include <mrs_msgs/msg/sphere.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/visualization_manager.hpp>

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace mrs_rviz_plugins
{
  namespace sphere
  {
    class Display : public rviz_common::MessageFilterDisplay<mrs_msgs::msg::Sphere>
    {
      Q_OBJECT
    public:
      // Constructor.  pluginlib::ClassLoader creates instances by calling
      // the default constructor, so make sure you have one.
      // Display();
      // virtual ~Display();

      // Overrides of protected virtual functions from Display.  As much
      // as possible, when Displays are not enabled, they should not be
      // subscribed to incoming data and should not show anything in the
      // 3D view.  These functions are where these connections are made
      // and broken.
    protected:
      void onInitialize() override;

      // A helper to clear this display back to the initial state.
      void reset() override;
      // Function to handle an incoming ROS message.
      void processMessage(mrs_msgs::msg::Sphere::ConstSharedPtr msg) override;

      // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
      void updateColorAndAlpha();
      void updateDrawDynamic();
      void updateDrawStatic();
      void updateDashedDynamic();
      void updateDashedStatic();

    private:
      // Storage for the list of visuals.  It is a circular buffer where
      // data gets popped from the front (oldest) and pushed to the back (newest)
      std::unique_ptr<Visual> visual_;

      // User-editable property variables.
      std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
      std::unique_ptr<rviz_common::properties::FloatProperty> alpha_property_;
      std::unique_ptr<rviz_common::properties::BoolProperty>  draw_dynamic_property_;
      std::unique_ptr<rviz_common::properties::BoolProperty>  draw_static_property_;
      std::unique_ptr<rviz_common::properties::BoolProperty>  dashed_dynamic_property_;
      std::unique_ptr<rviz_common::properties::BoolProperty>  dashed_static_property_;
    };
  }  // namespace sphere
}  // end namespace mrs_rviz_plugins

#endif
