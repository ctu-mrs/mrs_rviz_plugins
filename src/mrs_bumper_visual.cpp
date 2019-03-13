/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/object.h>

#include "mrs_bumper_visual.h"

namespace mrs_rviz_plugins
{

// BEGIN_TUTORIAL
MRS_Bumper_Visual::MRS_Bumper_Visual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
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
  // Here we create a node to store the pose of the MRS_Bumper_'s header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  /* m_sector_lines.reset(new rviz::BillboardLine( scene_manager_, frame_node_ )); */
}

MRS_Bumper_Visual::~MRS_Bumper_Visual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

/* draw_sector() method //{ */
std::shared_ptr<rviz::Object> MRS_Bumper_Visual::draw_sector(const double dist, const double vfov, const double hfov, const unsigned sector_it, const unsigned n_horizontal_sectors)
{
  std::shared_ptr<rviz::Object> ret = nullptr;
  if (sector_it < n_horizontal_sectors)
  {
    const double yaw = hfov*sector_it;
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
std::shared_ptr<rviz::Object> MRS_Bumper_Visual::draw_horizontal_sector(const double dist, const double vfov, const double hfov, const double yaw)
{
  std::shared_ptr<rviz::MeshShape> mesh_ptr = std::make_shared<rviz::MeshShape>( scene_manager_, frame_node_ );

  Ogre::Vector3 pts[] = {
    Ogre::Vector3( 0, 0, 0 ),
    Ogre::Vector3( cos(yaw-hfov/2.0)*dist, sin(yaw-hfov/2.0)*dist, tan(+vfov/2.0)*dist ),
    Ogre::Vector3( cos(yaw-hfov/2.0)*dist, sin(yaw-hfov/2.0)*dist, tan(-vfov/2.0)*dist ),
    Ogre::Vector3( cos(yaw+hfov/2.0)*dist, sin(yaw+hfov/2.0)*dist, tan(-vfov/2.0)*dist ),
    Ogre::Vector3( cos(yaw+hfov/2.0)*dist, sin(yaw+hfov/2.0)*dist, tan(+vfov/2.0)*dist )
  };

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
std::shared_ptr<rviz::Object> MRS_Bumper_Visual::draw_topdown_sector(const double dist, const double vfov, const unsigned n_horizontal_sectors)
{
  std::shared_ptr<rviz::MeshShape> mesh_ptr = std::make_shared<rviz::MeshShape>( scene_manager_, frame_node_ );

  const double hfov = 2.0*M_PI/n_horizontal_sectors;
  Ogre::Vector3 pts[n_horizontal_sectors];
  const Ogre::Vector3 start_pt( 0, 0, 0);
  const Ogre::Vector3 end_pt( 0, 0, dist);
  for (unsigned sector_it = 0; sector_it < n_horizontal_sectors; sector_it++)
  {
    const double cur_yaw = hfov*sector_it;
    const Ogre::Vector3 cur_pt( cos(cur_yaw-hfov/2.0)*dist/tan(vfov/2.0), sin(cur_yaw-hfov/2.0)*dist/tan(vfov/2.0), dist );
    pts[sector_it] = cur_pt;
  }

  mesh_ptr->beginTriangles();
  for (unsigned sector_it = 0; sector_it < n_horizontal_sectors; sector_it++)
  {
    unsigned next_sector_it = sector_it+1;
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

/* draw_lidar_1d() method //{ */
std::shared_ptr<rviz::Object> MRS_Bumper_Visual::draw_lidar_1d(const double dist, const unsigned sector_it, const unsigned n_horizontal_sectors)
{
  std::shared_ptr<rviz::Arrow> arr_ptr = std::make_shared<rviz::Arrow>( scene_manager_, frame_node_ );

  Ogre::Vector3 dir;
  if (sector_it < n_horizontal_sectors)
  {
    const double yaw = 2.0*M_PI/n_horizontal_sectors * sector_it;
    dir = Ogre::Vector3(cos(yaw), sin(yaw), 0);
  } else if (sector_it == n_horizontal_sectors)
  {
    dir = Ogre::Vector3(0, 0, -1);
  } else
  {
    dir = Ogre::Vector3(0, 0, 1);
  }
  arr_ptr->setDirection(dir);
  arr_ptr->set(0.9*dist, m_arr_shaft_diameter, 0.1*dist, m_arr_head_diameter);

  return arr_ptr;
}
//}

/* draw_lidar_2d() method //{ */
std::shared_ptr<rviz::Object> MRS_Bumper_Visual::draw_lidar_2d(const double dist, const unsigned sector_it, const unsigned n_horizontal_sectors)
{
  /* // so far, this method can only cope with horizontal measurements - relay the rest as 1D lidar */
  /* if (sector_it >= n_horizontal_sectors) */
  /*   return draw_lidar_1d(dist, sector_it, n_horizontal_sectors); */

  std::shared_ptr<rviz::MeshShape> mesh_ptr = std::make_shared<rviz::MeshShape>( scene_manager_, frame_node_ );

  constexpr unsigned n_segments = 10;
  const double hfov = 2.0*M_PI/n_horizontal_sectors;
  const double ang_step = hfov/(n_segments-1);
  const double vfov = 5.0/180.0*M_PI;
  const double h = tan(vfov/2.0)*dist;
  const double yaw = hfov*sector_it;
  const double yaw_cos = cos(yaw);
  const double yaw_sin = sin(yaw);

  double pts2d[n_segments][2];
  const Ogre::Vector3 start_pt( 0, 0, 0 );
  for (unsigned seg_it = 0; seg_it < n_segments; seg_it++)
  {
    const double cur_ang = -hfov/2.0 + ang_step*seg_it;
    const double x = dist*cos(cur_ang);
    const double y = dist*sin(cur_ang);
    pts2d[seg_it][0] = yaw_cos*x - yaw_sin*y;
    pts2d[seg_it][1] = yaw_sin*x + yaw_cos*y;
  }

  mesh_ptr->beginTriangles();

  for (unsigned seg_it = 0; seg_it < n_segments-1; seg_it++)
  {
    const double pt1[2] = { pts2d[seg_it][0], pts2d[seg_it][1] };
    const double pt2[2] = { pts2d[seg_it+1][0], pts2d[seg_it+1][1] };
    /* const Ogre::Vector2 pt1(yaw_cos*(pts2d[seg_it][0]), yaw_sin*(pts2d[seg_it][1])); */
    /* const Ogre::Vector2 pt2(yaw_cos*(pts2d[seg_it+1][0]), yaw_sin*(pts2d[seg_it+1][1])); */
    Ogre::Vector3 pt1_top(pt1[0], pt1[1], h/2.0);
    Ogre::Vector3 pt2_top(pt2[0], pt2[1], h/2.0);
    Ogre::Vector3 pt1_bot(pt1[0], pt1[1], -h/2.0);
    Ogre::Vector3 pt2_bot(pt2[0], pt2[1], -h/2.0);
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
    const Ogre::Vector3 pt_left_top(pts2d[0][0], pts2d[0][1], h/2.0);
    const Ogre::Vector3 pt_left_bot(pts2d[0][0], pts2d[0][1], -h/2.0);

    mesh_ptr->addVertex(start_pt);
    mesh_ptr->addVertex(pt_left_top);
    mesh_ptr->addVertex(pt_left_bot);
  }
  // add right wall
  {
    const Ogre::Vector3 pt_right_top(pts2d[n_segments-1][0], pts2d[n_segments-1][1], h/2.0);
    const Ogre::Vector3 pt_right_bot(pts2d[n_segments-1][0], pts2d[n_segments-1][1], -h/2.0);

    mesh_ptr->addVertex(start_pt);
    mesh_ptr->addVertex(pt_right_top);
    mesh_ptr->addVertex(pt_right_bot);
  }

  mesh_ptr->endTriangles();

  return mesh_ptr;
}
//}

std::shared_ptr<rviz::Object> MRS_Bumper_Visual::draw_sensor(const double dist, const double vfov, const double hfov, const int sensor_type, const unsigned sector_it, const unsigned n_horizontal_sectors)
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
    case msg_t::SENSOR_LIDAR_1D:
      ret = draw_lidar_1d(dist, sector_it, n_horizontal_sectors);
      break;
    case msg_t::SENSOR_LIDAR_2D:
      ret = draw_lidar_2d(dist, sector_it, n_horizontal_sectors);
      break;
  }
  return ret;
}

void MRS_Bumper_Visual::setMessage( const msg_t::ConstPtr& msg )
{
  m_msg = msg;
  draw_message( m_msg, m_display_mode );
}

void MRS_Bumper_Visual::draw_message( const msg_t::ConstPtr& msg, display_mode_t display_mode )
{
  if (msg == nullptr)
    return;

  const auto n_hor_sectors = msg->n_horizontal_sectors;
  const double hfov = 2.0*M_PI/n_hor_sectors;
  const double vfov = msg->sectors_vertical_fov;
  m_sectors.clear();
  m_sectors.reserve(n_hor_sectors+2);
  
  for (unsigned sector_it = 0; sector_it < n_hor_sectors+2; sector_it++)
  {
    const double cur_len = msg->sectors.at(sector_it);
    if (cur_len == msg_t::OBSTACLE_UNKNOWN)
      continue;

    std::shared_ptr<rviz::Object> shape_ptr = nullptr;

    switch (display_mode)
    {
      default:
      case display_mode_t::WHOLE_SECTORS:
        {
          shape_ptr = draw_sector(cur_len, vfov, hfov, sector_it, n_hor_sectors);
          break;
        }
      case display_mode_t::SENSOR_TYPES:
        {
          const auto cur_sensor = msg->sector_sensors.at(sector_it);
          shape_ptr = draw_sensor(cur_len, vfov, hfov, cur_sensor, sector_it, n_hor_sectors);
          break;
        }
    }

    if (shape_ptr != nullptr)
    {
      shape_ptr->setColor(m_color_r, m_color_g, m_color_b, m_color_a);
      m_sectors.push_back(shape_ptr);
    }
  }
}

// Position and orientation are passed through to the SceneNode.
void MRS_Bumper_Visual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void MRS_Bumper_Visual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void MRS_Bumper_Visual::setColor( float r, float g, float b, float a )
{
  m_color_r = r;
  m_color_g = g;
  m_color_b = b;
  m_color_a = a;
  for (auto& sector_ptr : m_sectors)
    sector_ptr->setColor( r, g, b, a );
}

// Color is passed through to the Arrow object.

void MRS_Bumper_Visual::setDisplayMode( display_mode_t display_mode )
{
  m_display_mode = display_mode;
  draw_message( m_msg, m_display_mode );
}
// END_TUTORIAL

} // end namespace mrs_rviz_plugins

