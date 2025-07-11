#include <mrs_rviz_plugins/sphere/visual.h>

namespace mrs_rviz_plugins
{
  namespace sphere
  {
    int Visual::sphere_idx = 0;

    Visual::Visual(rviz_common::DisplayContext* context,
                   Ogre::SceneNode*             parent_node)
    {
      Visual::sphere_idx++;
      context_       = context;
      scene_manager_ = context_->getSceneManager();

      // Ogre::SceneNode s form a tree, with each node storing the
      // transform (position and orientation) of itself relative to its
      // parent.  Ogre does the math of combining those transforms when it
      // is time to render.
      //
      // Here we create a node to store the pose of the sphere's header frame
      // relative to the RViz fixed frame.
      frame_node_ = parent_node->createChildSceneNode();

      // We create the arrow object within the frame node so that we can
      // set its position and direction relative to its header frame.
      /* m_sector_lines.reset(new rviz::BillboardLine( scene_manager_, frame_node_ )); */
      rviz_common::ViewManager::connect(context_->getViewManager(),
                                        SIGNAL(currentChanged()),
                                        this,
                                        SLOT(onViewControllerChanged()));

      // initialize circle orientations
      Ogre::Quaternion q{};  // unit quaternion
      circle_quats_[1] = q;
      q.FromAngleAxis(Ogre::Radian(M_PI_2), Ogre::Vector3(0, 1, 0));  // rotation around y axis by 90 degrees
      circle_quats_[2] = q;
      q.FromAngleAxis(Ogre::Radian(M_PI_2), Ogre::Vector3(1, 0, 0));  // rotation around x axis by 90 degrees
      circle_quats_[3] = q;
    }

    void Visual::freeCircle(Ogre::ManualObject*& circle_ptr)
    {
      if (!circle_ptr)
        return;
      scene_manager_->destroyManualObject(circle_ptr);
      circle_ptr = nullptr;
    }

    Visual::~Visual()
    {
      // Destroy the frame node since we don't need it anymore.
      scene_manager_->destroySceneNode(frame_node_);

      if (camera_) {
        camera_->removeListener(cam_listener_);
      }
      delete cam_listener_;
      cam_listener_ = nullptr;

      for (auto& circ : circles_)
        freeCircle(circ);
    }

    void Visual::setDrawDynamic(const bool draw)
    {
      std::lock_guard<std::mutex> lck(circles_quat_mtx_);
      draw_dynamic_ = draw;
      // if shouldn't be drawn and is drawn, delete it
      if (!draw && circle_dyn_) {
        freeCircle(circle_dyn_);
        camera_->removeListener(cam_listener_);
        camera_ = nullptr;
      }
      // if should be drawn and isn't drawn, create it
      else if (draw && !circle_dyn_ && got_msg_) {
        circle_dyn_ = initCircle(circle_names_.at(0),
                                 position_.x,
                                 position_.y,
                                 position_.z,
                                 radius_,
                                 circle_quats_.at(0),
                                 dashed_dynamic_);
        // the cam_listener_ gets callbacks before the camera is rendered to redraw the dynamic circle
        if (!camera_) {
          camera_ = context_->getViewManager()->getCurrent()->getCamera();
          camera_->addListener(cam_listener_);
        }
      }
    }

    void Visual::setDrawStatic(const bool draw)
    {
      std::lock_guard<std::mutex> lck(circles_quat_mtx_);
      draw_static_ = draw;
      for (int it = 1; it < circles_.size(); it++) {
        auto& circ = circles_.at(it);
        // if shouldn't be drawn and is drawn, delete it
        if (!draw && circ)
          freeCircle(circ);
        // if should be drawn and isn't drawn, create it
        else if (draw && !circ && got_msg_)
          circ = initCircle(circle_names_.at(it),
                            position_.x,
                            position_.y,
                            position_.z,
                            radius_,
                            circle_quats_.at(it),
                            dashed_static_);
      }
    }

    void Visual::setDashedDynamic(const bool dashed)
    {
      std::lock_guard<std::mutex> lck(circles_quat_mtx_);
      dashed_dynamic_ = dashed;
      // if shouldn't be drawn and is drawn, delete it
      if (circle_dyn_) {
        freeCircle(circle_dyn_);
        circle_dyn_ = initCircle(circle_names_.at(0),
                                 position_.x,
                                 position_.y,
                                 position_.z,
                                 radius_,
                                 circle_quats_.at(0),
                                 dashed_dynamic_);
      }
    }

    void Visual::setDashedStatic(const bool dashed)
    {
      std::lock_guard<std::mutex> lck(circles_quat_mtx_);
      dashed_static_ = dashed;
      // if shouldn't be drawn and is drawn, delete it
      for (int it = 1; it < circles_.size(); it++) {
        auto& circ = circles_.at(it);
        if (circ) {
          freeCircle(circ);
          circ = initCircle(circle_names_.at(it),
                            position_.x,
                            position_.y,
                            position_.z,
                            radius_,
                            circle_quats_.at(it),
                            dashed_static_);
        }
      }
    }

    void Visual::setColor(const float red,
                          const float green,
                          const float blue,
                          const float alpha)
    {
      red_   = red;
      green_ = green;
      blue_  = blue;
      alpha_ = alpha;
      std::lock_guard<std::mutex> lck(circles_quat_mtx_);
      for (int it = 0; it < circles_.size(); it++) {
        auto& circ = circles_.at(it);
        if (!circ)
          continue;
        circ->beginUpdate(0);
        fillCircle(circ, position_.x, position_.y, position_.z, radius_, circle_quats_.at(it));
        circ->end();
      }
    }

    void Visual::onViewControllerChanged()
    {
      if (!camera_)
        return;
      camera_ = context_->getViewManager()->getCurrent()->getCamera();
      camera_->addListener(cam_listener_);
    }

    void Visual::fillCircle(Ogre::ManualObject*     circle,
                            float                   x,
                            float                   y,
                            float                   z,
                            float                   r,
                            const Ogre::Quaternion& q,
                            int                     n_pts)
    {
      circle->colour(Ogre::ColourValue(red_, green_, blue_, alpha_));
      unsigned point_index = 0;
      for (float theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / n_pts) {
        const Ogre::Vector3 pos = q * Ogre::Vector3(r * cos(theta), r * sin(theta), 0) + Ogre::Vector3(x, y, z);
        circle->position(pos);
        circle->index(point_index++);
      }
      circle->index(0);  // Rejoins the last point to the first.
    }

    Ogre::ManualObject* Visual::initCircle(const std::string&      name,
                                           float                   x,
                                           float                   y,
                                           float                   z,
                                           float                   r,
                                           const Ogre::Quaternion& q,
                                           bool                    dashed,
                                           int                     n_pts)
    {
      Ogre::ManualObject*                        circle = scene_manager_->createManualObject(name);
      const Ogre::RenderOperation::OperationType op =
          dashed ? Ogre::RenderOperation::OT_LINE_LIST : Ogre::RenderOperation::OT_LINE_STRIP;
      circle->begin("BaseWhiteNoLighting", op);
      fillCircle(circle, x, y, z, r, q, n_pts);
      circle->end();
      frame_node_->attachObject(circle);
      return circle;
    }

    void Visual::setMessage(mrs_msgs::msg::Sphere::ConstSharedPtr msg)
    {
      std::lock_guard<std::mutex> lck(circles_quat_mtx_);
      radius_     = msg->radius;
      position_.x = msg->position.x;
      position_.y = msg->position.y;
      position_.z = msg->position.z;

      // dynamic circle is the one which is always oriented towards the viewer in Rviz
      if (draw_dynamic_ && !circle_dyn_) {
        circle_dyn_ = initCircle(circle_names_.at(0),
                                 position_.x,
                                 position_.y,
                                 position_.z,
                                 radius_,
                                 circle_quats_.at(0),
                                 dashed_dynamic_);
        // the cam_listener_ gets callbacks before the camera is rendered to redraw the dynamic circle
        if (!camera_) {
          camera_ = context_->getViewManager()->getCurrent()->getCamera();
          camera_->addListener(cam_listener_);
        }
      }

      // static circles are the ones which are always oriented according to the message's coordinate frame axes
      if (draw_static_) {
        for (int it = 1; it < circles_.size(); it++) {
          auto& circ = circles_.at(it);
          if (!circ) {
            // if the circle is not initialized yet, do it
            circ = initCircle(circle_names_.at(it),
                              position_.x,
                              position_.y,
                              position_.z,
                              radius_,
                              circle_quats_.at(it),
                              dashed_static_);
          }
          else {
            // otherwise just update it
            circ->beginUpdate(0);
            fillCircle(circ, position_.x, position_.y, position_.z, radius_, circle_quats_.at(it));
            circ->end();
          }
        }
      }

      got_msg_ = true;
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

  }  // namespace sphere

}  // end namespace mrs_rviz_plugins
