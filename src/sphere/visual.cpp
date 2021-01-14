// clang: MatousFormat

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/display_context.h>
#include <rviz/failed_view_controller.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>

#include "rviz/view_manager.h"
#include <sphere/visual.h>

namespace mrs_rviz_plugins
{

  namespace sphere
  {

    // BEGIN_TUTORIAL
    Visual::Visual(rviz::DisplayContext* context, Ogre::SceneNode* parent_node)
      : circle_(nullptr), camera_(nullptr), mylistener_(nullptr)
    {
      context_ = context;
      scene_manager_ = context_->getSceneManager();

/* RenderPanelCamera0 */
/* SelectionRect0_camera */
/* ViewControllerCamera1 */

      mylistener_ = new MyListener(this);

      // Ogre::SceneNode s form a tree, with each node storing the
      // transform (position and orientation) of itself relative to its
      // parent.  Ogre does the math of combining those transforms when it
      // is time to render.
      //
      // Here we create a node to store the pose of the bumper_'s header frame
      // relative to the RViz fixed frame.
      frame_node_ = parent_node->createChildSceneNode();

      // We create the arrow object within the frame node so that we can
      // set its position and direction relative to its header frame.
      /* m_sector_lines.reset(new rviz::BillboardLine( scene_manager_, frame_node_ )); */
      rviz::ViewManager::connect(context_->getViewManager(), SIGNAL(currentChanged()), this, SLOT(onViewControllerChanged()));
    }

    Visual::~Visual()
    {
      // Destroy the frame node since we don't need it anymore.
      scene_manager_->destroySceneNode(frame_node_);

      if (mylistener_)
      {
        camera_->removeListener(mylistener_);
        delete mylistener_;
        mylistener_ = nullptr;
      }

      if (circle_)
      {
        scene_manager_->destroyManualObject(circle_);
        circle_ = nullptr;
      }
    }

    void Visual::setMessage(const mrs_msgs::Sphere::ConstPtr& msg)
    {
      radius_ = msg->radius;
      position_.x = msg->position.x;
      position_.y = msg->position.y;
      position_.z = msg->position.z;

      if (!circle_)
      {
        static size_t circ_n = 0;
        circle_ = scene_manager_->createManualObject("circle_name" + std::to_string(circ_n++));
        circle_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
        fillCircle(circle_, 0, 0, 0, 0);
        circle_->end();
        circle_->setUseIdentityProjection(true);
        circle_->setUseIdentityView(true);
        frame_node_->attachObject(circle_);
      }

      if (!camera_)
      {
        camera_ = context_->getViewManager()->getCurrent()->getCamera();
        camera_->addListener(mylistener_);
      }

      /* // Assuming scene_mgr is your SceneManager. */

      /* // Assuming scene_mgr is your SceneManager. */
      /* Ogre::ManualObject* circle_xy = scene_manager_->createManualObject("circle_name"); */
      /* const float radius = m_radius; */
      /* const float n_pts = 35; */

      /* if (draw_xy_) */
      /* { */
      /*   circle_xy->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP); */
      /*   unsigned point_index = 0; */
      /*   for (float theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / n_pts) */
      /*   { */
      /*     circle_xy->position(radius * cos(theta), radius * sin(theta), 0); */
      /*     circle_xy->index(point_index++); */
      /*   } */
      /*   circle_xy->index(0); // Rejoins the last point to the first. */
      /*   circle_xy->end(); */
      /* } */

      /* if (draw_yz_) */
      /* { */
      /*   circle_xy->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP); */
      /*   unsigned point_index = 0; */
      /*   for (float theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / n_pts) */
      /*   { */
      /*     circle_xy->position(0, radius * cos(theta), radius * sin(theta)); */
      /*     circle_xy->index(point_index++); */
      /*   } */
      /*   circle_xy->index(0); // Rejoins the last point to the first. */
      /*   circle_xy->end(); */
      /* } */

      /* if (draw_xz_) */
      /* { */
      /*   circle_xy->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP); */
      /*   unsigned point_index = 0; */
      /*   for(float theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / n_pts) */
      /*   { */
      /*     circle_xy->position(0, radius * cos(theta), radius * sin(theta)); */
      /*     circle_xy->index(point_index++); */
      /*   } */
      /*   circle_xy->index(0); // Rejoins the last point to the first. */
      /*   circle_xy->end(); */
      /* } */
      
      /* frame_node_->attachObject(circle); */
    }

    // Position and orientation are passed through to the SceneNode.
    void Visual::setFramePosition(const Ogre::Vector3& position)
    {
      frame_node_->setPosition(position);
    }

    void Visual::setFrameOrientation(const Ogre::Quaternion& orientation)
    {
      frame_node_->setOrientation(orientation);
    }

  }  // namespace bumper

}  // end namespace mrs_rviz_plugins
