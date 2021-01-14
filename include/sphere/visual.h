// clang: MatousFormat

#ifndef SPHERE_H
#define SPHERE_H

#include <mrs_msgs/Sphere.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <rviz/message_filter_display.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>

namespace Ogre
{
  class Vector3;
  class Quaternion;
}  // namespace Ogre

namespace rviz
{
  class Arrow;
  class MeshShape;
  class Object;
}  // namespace rviz

namespace mrs_rviz_plugins
{

  namespace sphere
  {

    class Visual : public QObject
    {
Q_OBJECT
    public:
      // Constructor.  Creates the visual stuff and puts it into the
      // scene, but in an unconfigured state.
      Visual(rviz::DisplayContext* context, Ogre::SceneNode* parent_node);

      // Destructor.  Removes the visual stuff from the scene.
      virtual ~Visual();

      void setMessage(const mrs_msgs::Sphere::ConstPtr& msg);

      // Set the pose of the coordinate frame the message refers to.
      // These could be done inside setMessage(), but that would require
      // calls to FrameManager and error handling inside setMessage(),
      // which doesn't seem as clean.  This way Visual is only
      // responsible for visualization.
      void setFramePosition(const Ogre::Vector3& position);
      void setFrameOrientation(const Ogre::Quaternion& orientation);
 private Q_SLOTS:
      void onViewControllerChanged()
      {
        if (!camera_)
          return;

        camera_ = context_->getViewManager()->getCurrent()->getCamera();
        camera_->addListener(mylistener_);
        std::cout << "changed current camera to " << camera_->getName() << std::endl;
      }

    private:
      bool draw_xy_;
      bool draw_yz_;
      bool draw_xz_;
      double radius_;
      Ogre::Vector3 position_;

      Ogre::ManualObject* circle_;

      Ogre::Camera* camera_;

      rviz::DisplayContext* context_;

      // A SceneNode whose pose is set to match the coordinate frame of
      // the sphere message header.
      Ogre::SceneNode* frame_node_;

      // The SceneManager, kept here only so the destructor can ask it to
      // destroy the ``frame_node_``.
      Ogre::SceneManager* scene_manager_;

      class MyListener;
      MyListener* mylistener_;

      class MyListener : public Ogre::Camera::Listener
      {
        Visual* vis_;
        Ogre::ManualObject*& circle_;

        public:
        MyListener(Visual* vis)
          : vis_(vis), circle_(vis_->circle_)
        {}

        float last_width, last_height, last_x, last_y;
        void cameraPreRenderScene(Ogre::Camera *cam)
        {
          const float n_pts = 35;

          float l, t, r, b;
          cam->projectSphere(Ogre::Sphere(vis_->position_, vis_->radius_), &l, &t, &r, &b);
          const float width = r-l;
          const float height = t-b;
          /* const float radius = std::max(width, height); */
          const float x = l + (width)/2.0f;
          const float y = b + (height)/2.0f;

          if (width == last_width
           && height == last_height
           && x == last_x
           && y == last_y)
            return;

          last_width = width;
          last_height = height;
          last_x = x;
          last_y = y;

          circle_->clear();
          circle_->setUseIdentityProjection(true);
          circle_->setUseIdentityView(true);
          circle_->setQueryFlags(0);
          circle_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
          unsigned point_index = 0;
          for (float theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / n_pts)
          {
            circle_->position(x + width * cos(theta), y + height * sin(theta), -1);
            circle_->index(point_index++);
          }
          circle_->index(0); // Rejoins the last point to the first.
          circle_->end();
        }
      };
    };

  }  // namespace sphere

}  // end namespace mrs_rviz_plugins

#endif // SPHERE_H
