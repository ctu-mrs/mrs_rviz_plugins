// clang: MatousFormat

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/object.h>

#include <bumper/visual.h>

namespace mrs_rviz_plugins
{

  namespace bumper
  {

    // BEGIN_TUTORIAL
    Visual::Visual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    {
      m_arr_head_diameter = 0.2;
      m_arr_shaft_diameter = 0.1;
      m_display_mode = display_mode_t::WHOLE_SECTORS;
      m_msg = nullptr;

      scene_manager_ = scene_manager;

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
    }

    Visual::~Visual()
    {
      // Destroy the frame node since we don't need it anymore.
      scene_manager_->destroySceneNode(frame_node_);
    }

    /* draw_no_data() method //{ */
    /*  //{ */
    // compensate vetrical (used to make horizontal shapes into vertical)
    Ogre::Vector3 cove(const Ogre::Vector3& vec, bool compensate = false, bool up = true)
    {
      if (!compensate)
        return vec;
      if (up)
        return {vec.z, vec.y, vec.x};
      else
        return {vec.z, vec.y, -vec.x};
    }
    //}

    // should draw a nice little question mark
    std::shared_ptr<rviz::Object> Visual::draw_no_data(const unsigned sector_it, const unsigned n_horizontal_sectors)
    {
      const bool v = sector_it >= n_horizontal_sectors;  // whether the sector is vertical
      const bool u = sector_it > n_horizontal_sectors;   // whether the sector is up
      const float hfov = 2.0 * M_PI / n_horizontal_sectors;
      const float yaw = v ? 0.0f : hfov * sector_it;
      std::shared_ptr<rviz::BillboardLine> ret = std::make_shared<rviz::BillboardLine>(scene_manager_, frame_node_);
      constexpr float base_len = 2.0;
      constexpr int arc_pts = 10;
      constexpr float arc_r = 1.0;
      constexpr float arc_a_start = M_PI;
      constexpr float arc_a_end = arc_a_start + 3.0 / 2.0 * M_PI;
      ret->addPoint(cove({0.0, 0.0, 0.0}, v, u));
      ret->addPoint(cove({cos(yaw) * base_len, sin(yaw) * base_len, 0.0}, v, u));
      for (int it = 0; it < arc_pts; it++)
      {
        const float angle = yaw + arc_a_start + (arc_a_end - arc_a_start) / arc_pts * it;
        ret->addPoint(cove({cos(yaw) * (base_len + arc_r) + cos(angle) * arc_r, sin(yaw) * (base_len + arc_r) + sin(angle) * arc_r, 0.0}, v, u));
      }
      return ret;
    }
    //}

    /* draw_sector() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_sector(const double dist, const double vfov, const double hfov, const unsigned sector_it,
                                                      const unsigned n_horizontal_sectors)
    {
      std::shared_ptr<rviz::Object> ret = nullptr;
      if (sector_it < n_horizontal_sectors)
      {
        const double yaw = hfov * sector_it;
        ret = draw_horizontal_sector(dist, vfov, hfov, yaw);
      } else if (sector_it == n_horizontal_sectors)
      {
        ret = draw_topdown_sector(-dist, vfov, n_horizontal_sectors);
      } else
      {
        ret = draw_topdown_sector(dist, vfov, n_horizontal_sectors);
      }
      return ret;
    }
    //}

    /* draw_horizontal_sector() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_horizontal_sector(const double dist, const double vfov, const double hfov, const double yaw)
    {
      std::shared_ptr<rviz::MeshShape> mesh_ptr = std::make_shared<rviz::MeshShape>(scene_manager_, frame_node_);

      Ogre::Vector3 pts[] = {Ogre::Vector3(0, 0, 0),
                             Ogre::Vector3(cos(yaw - hfov / 2.0) * dist, sin(yaw - hfov / 2.0) * dist, tan(+vfov / 2.0) * dist),
                             Ogre::Vector3(cos(yaw - hfov / 2.0) * dist, sin(yaw - hfov / 2.0) * dist, tan(-vfov / 2.0) * dist),
                             Ogre::Vector3(cos(yaw + hfov / 2.0) * dist, sin(yaw + hfov / 2.0) * dist, tan(-vfov / 2.0) * dist),
                             Ogre::Vector3(cos(yaw + hfov / 2.0) * dist, sin(yaw + hfov / 2.0) * dist, tan(+vfov / 2.0) * dist)};

      mesh_ptr->beginTriangles();

      mesh_ptr->addVertex(pts[0]);
      mesh_ptr->addVertex(pts[1]);
      mesh_ptr->addVertex(pts[2]);

      mesh_ptr->addVertex(pts[0]);
      mesh_ptr->addVertex(pts[2]);
      mesh_ptr->addVertex(pts[3]);

      mesh_ptr->addVertex(pts[0]);
      mesh_ptr->addVertex(pts[3]);
      mesh_ptr->addVertex(pts[4]);

      mesh_ptr->addVertex(pts[0]);
      mesh_ptr->addVertex(pts[4]);
      mesh_ptr->addVertex(pts[1]);

      mesh_ptr->addVertex(pts[1]);
      mesh_ptr->addVertex(pts[2]);
      mesh_ptr->addVertex(pts[3]);

      mesh_ptr->addVertex(pts[3]);
      mesh_ptr->addVertex(pts[4]);
      mesh_ptr->addVertex(pts[1]);

      mesh_ptr->endTriangles();

      return mesh_ptr;
    }
    //}

    /* draw_topdown_sector() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_topdown_sector(const double dist, const double vfov, const unsigned n_horizontal_sectors)
    {
      std::shared_ptr<rviz::MeshShape> mesh_ptr = std::make_shared<rviz::MeshShape>(scene_manager_, frame_node_);

      const double hfov = 2.0 * M_PI / n_horizontal_sectors;
      Ogre::Vector3 pts[n_horizontal_sectors];
      const Ogre::Vector3 start_pt(0, 0, 0);
      const Ogre::Vector3 end_pt(0, 0, dist);
      for (unsigned sector_it = 0; sector_it < n_horizontal_sectors; sector_it++)
      {
        const double cur_yaw = hfov * sector_it;
        const Ogre::Vector3 cur_pt(cos(cur_yaw - hfov / 2.0) * dist / tan(vfov / 2.0), sin(cur_yaw - hfov / 2.0) * dist / tan(vfov / 2.0), dist);
        pts[sector_it] = cur_pt;
      }

      mesh_ptr->beginTriangles();
      for (unsigned sector_it = 0; sector_it < n_horizontal_sectors; sector_it++)
      {
        unsigned next_sector_it = sector_it + 1;
        if (next_sector_it >= n_horizontal_sectors)
          next_sector_it = 0;
        mesh_ptr->addVertex(start_pt);
        mesh_ptr->addVertex(pts[sector_it]);
        mesh_ptr->addVertex(pts[next_sector_it]);

        mesh_ptr->addVertex(pts[sector_it]);
        mesh_ptr->addVertex(pts[next_sector_it]);
        mesh_ptr->addVertex(end_pt);
      }
      mesh_ptr->endTriangles();

      return mesh_ptr;
    }
    //}

    /* draw_lidar1d() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_lidar1d(const double dist, const unsigned sector_it, const unsigned n_horizontal_sectors)
    {
      std::shared_ptr<rviz::Arrow> arr_ptr = std::make_shared<rviz::Arrow>(scene_manager_, frame_node_);

      Ogre::Vector3 dir;
      if (sector_it < n_horizontal_sectors)
      {
        const double yaw = 2.0 * M_PI / n_horizontal_sectors * sector_it;
        dir = Ogre::Vector3(cos(yaw), sin(yaw), 0);
      } else if (sector_it == n_horizontal_sectors)
      {
        dir = Ogre::Vector3(0, 0, -1);
      } else
      {
        dir = Ogre::Vector3(0, 0, 1);
      }
      arr_ptr->setDirection(dir);
      arr_ptr->set(0.9 * dist, m_arr_shaft_diameter, 0.1 * dist, m_arr_head_diameter);

      return arr_ptr;
    }
    //}

    /* draw_lidar2d() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_lidar2d(const double dist, const unsigned sector_it, const unsigned n_horizontal_sectors)
    {
      const double vfov = 2.5 / 180.0 * M_PI;
      return draw_lidar3d(dist, vfov, sector_it, n_horizontal_sectors);
    }
    //}

    /* draw_lidar3d() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_lidar3d(const double dist, const double vfov, const unsigned sector_it, const unsigned n_horizontal_sectors)
    {
      /* // so far, this method can only cope with horizontal measurements - relay the rest as 1D lidar */
      const double hfov = 2.0 * M_PI / n_horizontal_sectors;
      if (sector_it >= n_horizontal_sectors)
        return draw_sector(dist, vfov, hfov, sector_it, n_horizontal_sectors);

      std::shared_ptr<rviz::MeshShape> mesh_ptr = std::make_shared<rviz::MeshShape>(scene_manager_, frame_node_);

      constexpr unsigned n_segments = 10;
      const double ang_step = hfov / (n_segments - 1);
      const double h = tan(vfov / 2.0) * dist;
      const double yaw = hfov * sector_it;
      const double yaw_cos = cos(yaw);
      const double yaw_sin = sin(yaw);

      double pts2d[n_segments][2];
      const Ogre::Vector3 start_pt(0, 0, 0);
      for (unsigned seg_it = 0; seg_it < n_segments; seg_it++)
      {
        const double cur_ang = -hfov / 2.0 + ang_step * seg_it;
        const double x = dist * cos(cur_ang);
        const double y = dist * sin(cur_ang);
        pts2d[seg_it][0] = yaw_cos * x - yaw_sin * y;
        pts2d[seg_it][1] = yaw_sin * x + yaw_cos * y;
      }

      mesh_ptr->beginTriangles();

      for (unsigned seg_it = 0; seg_it < n_segments - 1; seg_it++)
      {
        const double pt1[2] = {pts2d[seg_it][0], pts2d[seg_it][1]};
        const double pt2[2] = {pts2d[seg_it + 1][0], pts2d[seg_it + 1][1]};
        /* const Ogre::Vector2 pt1(yaw_cos*(pts2d[seg_it][0]), yaw_sin*(pts2d[seg_it][1])); */
        /* const Ogre::Vector2 pt2(yaw_cos*(pts2d[seg_it+1][0]), yaw_sin*(pts2d[seg_it+1][1])); */
        Ogre::Vector3 pt1_top(pt1[0], pt1[1], +h);
        Ogre::Vector3 pt2_top(pt2[0], pt2[1], +h);
        Ogre::Vector3 pt1_bot(pt1[0], pt1[1], -h);
        Ogre::Vector3 pt2_bot(pt2[0], pt2[1], -h);
        // top vertices
        mesh_ptr->addVertex(start_pt);
        mesh_ptr->addVertex(pt1_top);
        mesh_ptr->addVertex(pt2_top);

        // bottom vertices
        mesh_ptr->addVertex(start_pt);
        mesh_ptr->addVertex(pt1_bot);
        mesh_ptr->addVertex(pt2_bot);

        // top/bot connections
        mesh_ptr->addVertex(pt1_top);
        mesh_ptr->addVertex(pt2_top);
        mesh_ptr->addVertex(pt1_bot);

        mesh_ptr->addVertex(pt2_top);
        mesh_ptr->addVertex(pt1_bot);
        mesh_ptr->addVertex(pt2_bot);
      }
      // add left wall
      {
        const Ogre::Vector3 pt_left_top(pts2d[0][0], pts2d[0][1], +h);
        const Ogre::Vector3 pt_left_bot(pts2d[0][0], pts2d[0][1], -h);

        mesh_ptr->addVertex(start_pt);
        mesh_ptr->addVertex(pt_left_top);
        mesh_ptr->addVertex(pt_left_bot);
      }
      // add right wall
      {
        const Ogre::Vector3 pt_right_top(pts2d[n_segments - 1][0], pts2d[n_segments - 1][1], +h);
        const Ogre::Vector3 pt_right_bot(pts2d[n_segments - 1][0], pts2d[n_segments - 1][1], -h);

        mesh_ptr->addVertex(start_pt);
        mesh_ptr->addVertex(pt_right_top);
        mesh_ptr->addVertex(pt_right_bot);
      }

      mesh_ptr->endTriangles();

      return mesh_ptr;
    }
    //}

    /* draw_sensor() method //{ */
    std::shared_ptr<rviz::Object> Visual::draw_sensor(const double dist, const double vfov, const double hfov, const int sensor_type, const unsigned sector_it,
                                                      const unsigned n_horizontal_sectors)
    {
      std::shared_ptr<rviz::Object> ret = nullptr;
      switch (sensor_type)
      {
        case msg_t::SENSOR_NONE:
        default:
          break;
        case msg_t::SENSOR_DEPTH:
          ret = draw_sector(dist, vfov, hfov, sector_it, n_horizontal_sectors);
          break;
        case msg_t::SENSOR_LIDAR1D:
          ret = draw_lidar1d(dist, sector_it, n_horizontal_sectors);
          break;
        case msg_t::SENSOR_LIDAR2D:
          ret = draw_lidar2d(dist, sector_it, n_horizontal_sectors);
          break;
        case msg_t::SENSOR_LIDAR3D:
          ret = draw_lidar3d(dist, vfov, sector_it, n_horizontal_sectors);
          break;
      }
      return ret;
    }
    //}

    /* setMessage() method //{ */
    void Visual::setMessage(const msg_t::ConstPtr& msg)
    {
      m_msg = msg;
      draw_message(m_msg, m_display_mode);
    }
    //}

    /* draw_message() method //{ */
    void Visual::draw_message(const msg_t::ConstPtr& msg, display_mode_t display_mode)
    {
      if (msg == nullptr)
        return;
    
      const auto n_hor_sectors = msg->n_horizontal_sectors;
      const double hfov = 2.0 * M_PI / n_hor_sectors;
      const double vfov = msg->sectors_vertical_fov;
      m_sectors.clear();
      m_sectors.reserve(n_hor_sectors + 2);
    
      for (unsigned sector_it = 0; sector_it < n_hor_sectors + 2; sector_it++)
      {
        constexpr double max_len = 666.0;
        double cur_len = msg->sectors.at(sector_it);
        std::shared_ptr<rviz::Object> object_ptr = nullptr;
    
        if (cur_len == mrs_msgs::ObstacleSectors::OBSTACLE_NOT_DETECTED)
        {
          if (m_show_undetected)
            cur_len = max_len;
          else
            continue;
        }
    
        if (cur_len == msg_t::OBSTACLE_NO_DATA)
        {
          if (m_show_no_data)
            object_ptr = draw_no_data(sector_it, n_hor_sectors);
          else
            continue;
        } else
        {
          assert(cur_len >= 0.0);
          switch (display_mode)
          {
            default:
            case display_mode_t::WHOLE_SECTORS:
            {
              object_ptr = draw_sector(cur_len, vfov, hfov, sector_it, n_hor_sectors);
              break;
            }
            case display_mode_t::SENSOR_TYPES:
            {
              const auto cur_sensor = msg->sector_sensors.at(sector_it);
              object_ptr = draw_sensor(cur_len, vfov, hfov, cur_sensor, sector_it, n_hor_sectors);
              break;
            }
          }
        }
    
        if (object_ptr != nullptr)
        {
    
          if (sector_it < n_hor_sectors && m_collision_colorize && cur_len >= 0.0 && cur_len <= m_collision_horizontal_threshold)
            object_ptr->setColor(m_collision_color_r, m_collision_color_g, m_collision_color_b, m_collision_color_a);
          else if (sector_it >= n_hor_sectors && m_collision_colorize && cur_len >= 0.0 && cur_len <= m_collision_vertical_threshold)
            object_ptr->setColor(m_collision_color_r, m_collision_color_g, m_collision_color_b, m_collision_color_a);
          else
            object_ptr->setColor(m_color_r, m_color_g, m_color_b, m_color_a);
    
          m_sectors.push_back(object_ptr);
        }
      }
    }
    //}

    // Position and orientation are passed through to the SceneNode.
    void Visual::setFramePosition(const Ogre::Vector3& position)
    {
      frame_node_->setPosition(position);
    }

    void Visual::setFrameOrientation(const Ogre::Quaternion& orientation)
    {
      frame_node_->setOrientation(orientation);
    }

    // Color is passed through to the Arrow object.
    void Visual::setColor(float r, float g, float b, float a)
    {
      m_color_r = r;
      m_color_g = g;
      m_color_b = b;
      m_color_a = a;
      draw_message(m_msg, m_display_mode);
    }

    // Color is passed through to the Arrow object.

    void Visual::setDisplayMode(display_mode_t display_mode)
    {
      m_display_mode = display_mode;
      draw_message(m_msg, m_display_mode);
    }

    void Visual::setShowUndetected(bool show_undetected)
    {
      m_show_undetected = show_undetected;
      draw_message(m_msg, m_display_mode);
    }

    void Visual::setShowNoData(bool show_no_data)
    {
      m_show_no_data = show_no_data;
      draw_message(m_msg, m_display_mode);
    }

    void Visual::setCollisionOptions(bool colorize, float horizontal_threshold, float vertical_threshold, float r, float g, float b, float a)
    {
      m_collision_color_r = r;
      m_collision_color_g = g;
      m_collision_color_b = b;
      m_collision_color_a = a;
      m_collision_colorize = colorize;
      m_collision_horizontal_threshold = horizontal_threshold;
      m_collision_vertical_threshold = vertical_threshold;
      draw_message(m_msg, m_display_mode);
    }

  }  // namespace bumper

}  // end namespace mrs_rviz_plugins
