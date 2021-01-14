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
        /* std::cout << "changed current camera to " << camera_->getName() << std::endl; */
      }

      void fillCircle(Ogre::ManualObject* circle, float x, float y, float cx, float cy, int n_pts = 32)
      {
        unsigned point_index = 0;
        for (float theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / n_pts)
        {
          circle->position(x + cx * cos(theta), y + cy * sin(theta), -1);
          circle->index(point_index++);
        }
        circle->index(0); // Rejoins the last point to the first.
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

        float last_x, last_y, last_cx, last_cy;
        void cameraPreRenderScene(Ogre::Camera *cam)
        {
          const Ogre::Vector3 eyeSpacePos = cam->getViewMatrix(true) * vis_->position_;
          const auto projMat = cam->getProjectionMatrix();
          float x, y, cx, cy;
          // z < 0 means in front of cam
          if (eyeSpacePos.z < 0.0f)
          {
            // calculate projected pos
            const Ogre::Vector3 screenSpacePos = projMat * eyeSpacePos;
            x = screenSpacePos.x;
            y = screenSpacePos.y;
            // calculate projected size
            const Ogre::Vector3 sphere(vis_->radius_, vis_->radius_, eyeSpacePos.z);
            const Ogre::Vector3 spheresize = cam->getProjectionMatrix() * sphere;
            cx = spheresize.x/2.0f;
            cy = spheresize.y/2.0f;
          }
          else
          {
            x = y = cx = cy = 0.0f;
          }

          if (
              x == last_x
           && y == last_y
           && cx == last_cx
           && cy == last_cy)
            return;

          last_x = x;
          last_y = y;
          last_cx = cx;
          last_cy = cy;

          circle_->beginUpdate(0);
          vis_->fillCircle(circle_, x, y, cx, cy);
          circle_->end();

          std::cout << "\tx\ty\tcx\tcy" << std::endl;
          std::cout << "\t" << x << "\t" << y << "\t" << cx << "\t" << cy << std::endl;
          std::cout << "visible: " << circle_->isVisible() << std::endl;
        }
      };
    };

  }  // namespace sphere

}  // end namespace mrs_rviz_plugins

#endif // SPHERE_H
