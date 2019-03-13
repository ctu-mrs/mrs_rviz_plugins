#ifndef MRS_BUMPER_VISUAL_H
#define MRS_BUMPER_VISUAL_H

#include <mrs_bumper/ObstacleSectors.h>

namespace Ogre
{
  class Vector3;
  class Quaternion;
}

namespace rviz
{
  class Arrow;
  class MeshShape;
  class Shape;
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
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  MRS_Bumper_Visual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~MRS_Bumper_Visual();

  // Configure the visual to show the data in the message.
  void setMessage( const mrs_bumper::ObstacleSectors::ConstPtr& msg );

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

private:
  void draw_sector(const boost::shared_ptr<rviz::MeshShape>& mesh_ptr, const Ogre::Vector3 pts[]);
  void draw_topdown_sector(const boost::shared_ptr<rviz::MeshShape>& mesh_ptr, const double distance, const double vfov, const unsigned n_horizontal_sectors);

  // The object implementing the actual shape
  std::vector<boost::shared_ptr<rviz::Shape>> m_sectors;

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
