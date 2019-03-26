#ifndef MRS_BUMPER_VISUAL_H
#define MRS_BUMPER_VISUAL_H

#include <mrs_msgs/ObstacleSectors.h>

namespace Ogre
{
  class Vector3;
  class Quaternion;
}

namespace rviz
{
  class Arrow;
  class MeshShape;
  class Object;
}

namespace mrs_rviz_plugins
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of MRS_Bumper_Visual represents the visualization of a single
// sensor_msgs::MRS_Bumper_ message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class MRS_Bumper_Visual
{
  public:
    enum display_mode_t
    {
      WHOLE_SECTORS,
      SENSOR_TYPES
    };

    using msg_t = mrs_msgs::ObstacleSectors;
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  MRS_Bumper_Visual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~MRS_Bumper_Visual();

  // Configure the visual to show the data in the message.
  void setMessage( const msg_t::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way MRS_Bumper_Visual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the MRS_Bumper_ message.
  void setColor( float r, float g, float b, float a );

  void setDisplayMode( display_mode_t option );

  void setShowUndetected( bool show_undetected );

  void setShowNoData( bool show_no_data );

private:
  void draw_message( const msg_t::ConstPtr& msg, display_mode_t display_mode );
  std::shared_ptr<rviz::Object> draw_no_data(const unsigned sector_it, const unsigned n_horizontal_sectors);
  std::shared_ptr<rviz::Object> draw_sensor(const double dist, const double vfov, const double hfov, const int sensor_type, const unsigned sector_it, const unsigned n_horizontal_sectors);
  std::shared_ptr<rviz::Object> draw_sector(const double dist, const double vfov, const double hfov, const unsigned sector_it, const unsigned n_horizontal_sectors);
  std::shared_ptr<rviz::Object> draw_horizontal_sector(const double dist, const double vfov, const double hfov, const double yaw);
  std::shared_ptr<rviz::Object> draw_topdown_sector(const double dist, const double vfov, const unsigned n_horizontal_sectors);
  std::shared_ptr<rviz::Object> draw_lidar_1d(const double dist, const unsigned sector_it, const unsigned n_horizontal_sectors);
  std::shared_ptr<rviz::Object> draw_lidar_2d(const double dist, const unsigned sector_it, const unsigned n_horizontal_sectors);

  double m_arr_head_diameter;
  double m_arr_shaft_diameter;
  float m_color_r, m_color_g, m_color_b, m_color_a;
  bool m_show_undetected;
  bool m_show_no_data;

  msg_t::ConstPtr m_msg;
  display_mode_t m_display_mode;
  // The object implementing the actual Object
  std::vector<std::shared_ptr<rviz::Object>> m_sectors;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the MRS_Bumper_ message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

} // end namespace mrs_rviz_plugins


#endif // MRS_BUMPER_VISUAL_H
