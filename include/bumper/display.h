// clang: MatousFormat

#ifndef MRS_BUMPER_DISPLAY_H
#define MRS_BUMPER_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <mrs_msgs/ObstacleSectors.h>
#endif

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class ColorProperty;
  class EnumProperty;
  class FloatProperty;
  class IntProperty;
  class BoolProperty;
}  // namespace rviz

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace mrs_rviz_plugins
{

  namespace bumper
  {

    class Visual;

    class Display : public rviz::MessageFilterDisplay<mrs_msgs::ObstacleSectors>
    {
      Q_OBJECT
    public:
      // Constructor.  pluginlib::ClassLoader creates instances by calling
      // the default constructor, so make sure you have one.
      Display();
      virtual ~Display();

      // Overrides of protected virtual functions from Display.  As much
      // as possible, when Displays are not enabled, they should not be
      // subscribed to incoming data and should not show anything in the
      // 3D view.  These functions are where these connections are made
      // and broken.
    protected:
      virtual void onInitialize();

      // A helper to clear this display back to the initial state.
      virtual void reset();

      // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
      void updateColorAndAlpha();
      void updateHistoryLength();
      void updateDisplayMode();
      void updateShowUndetected();
      void updateShowNoData();
      void updateCollisions();

      // Function to handle an incoming ROS message.
    private:
      void processMessage(const mrs_msgs::ObstacleSectors::ConstPtr& msg);

      // Storage for the list of visuals.  It is a circular buffer where
      // data gets popped from the front (oldest) and pushed to the back (newest)
      boost::circular_buffer<boost::shared_ptr<Visual>> visuals_;

      // User-editable property variables.
      rviz::ColorProperty* color_property_;
      rviz::FloatProperty* alpha_property_;
      rviz::BoolProperty* collision_colorize_property_;
      rviz::FloatProperty* horizontal_collision_threshold_property_;
      rviz::FloatProperty* vertical_collision_threshold_property_;
      rviz::FloatProperty* collision_alpha_property_;
      rviz::ColorProperty* collision_color_property_;
      rviz::IntProperty* history_length_property_;
      rviz::EnumProperty* display_mode_property_;
      rviz::BoolProperty* show_undetected_property_;
      rviz::BoolProperty* show_no_data_property_;
    };

  }  // namespace bumper

}  // end namespace mrs_rviz_plugins

#endif
