// clang: MatousFormat

#ifndef MRS_BUMPER_DISPLAY_H
#define MRS_BUMPER_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <mrs_msgs/Sphere.h>
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

  namespace sphere
  {
    class Visual;

    class Display : public rviz::MessageFilterDisplay<mrs_msgs::Sphere>
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

      // Function to handle an incoming ROS message.
    private:
      void processMessage(const mrs_msgs::Sphere::ConstPtr& msg);

      // Storage for the list of visuals.  It is a circular buffer where
      // data gets popped from the front (oldest) and pushed to the back (newest)
      boost::shared_ptr<Visual> visual_;

      // User-editable property variables.
      rviz::ColorProperty* color_property_;
      rviz::FloatProperty* alpha_property_;
    };

  }  // namespace sphere

}  // end namespace mrs_rviz_plugins

#endif
