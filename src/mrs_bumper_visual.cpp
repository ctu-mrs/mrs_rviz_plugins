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
#include <rviz/ogre_helpers/shape.h>

#include <boost/smart_ptr/make_shared.hpp>

#include "mrs_bumper_visual.h"

namespace mrs_rviz_plugins
{

// BEGIN_TUTORIAL
MRS_Bumper_Visual::MRS_Bumper_Visual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
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

void MRS_Bumper_Visual::draw_sector(const boost::shared_ptr<rviz::MeshShape>& mesh_ptr, const Ogre::Vector3 pts[])
{
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
}

void MRS_Bumper_Visual::draw_topdown_sector(const boost::shared_ptr<rviz::MeshShape>& mesh_ptr, const double distance, const double vfov, const unsigned n_horizontal_sectors)
{
  const double hfov = 2.0*M_PI/n_horizontal_sectors;
  Ogre::Vector3 pts[n_horizontal_sectors];
  const Ogre::Vector3 start_pt( 0, 0, 0);
  const Ogre::Vector3 end_pt( 0, 0, distance);
  for (unsigned sector_it = 0; sector_it < n_horizontal_sectors; sector_it++)
  {
    const double cur_yaw = hfov*sector_it;
    const Ogre::Vector3 cur_pt( cos(cur_yaw-hfov/2.0)*distance/tan(vfov/2.0), sin(cur_yaw-hfov/2.0)*distance/tan(vfov/2.0), distance );
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
}

void MRS_Bumper_Visual::setMessage( const mrs_bumper::ObstacleSectors::ConstPtr& msg )
{
  const double hfov = 2.0*M_PI/msg->n_horizontal_sectors;
  const double vfov = msg->sectors_vertical_fov;
  m_sectors.clear();
  m_sectors.reserve(msg->n_horizontal_sectors+2);
  
  for (unsigned sector_it = 0; sector_it < msg->n_horizontal_sectors; sector_it++)
  {
    const double cur_len = msg->sectors.at(sector_it);
    if (cur_len == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN)
      continue;

    boost::shared_ptr<rviz::MeshShape> sector_ptr = boost::make_shared<rviz::MeshShape>( scene_manager_, frame_node_ );
    const double cur_yaw = hfov*sector_it;

    Ogre::Vector3 pts[] = {
      Ogre::Vector3( 0, 0, 0 ),
      Ogre::Vector3( cos(cur_yaw-hfov/2.0)*cur_len, sin(cur_yaw-hfov/2.0)*cur_len, tan(+vfov/2.0)*cur_len ),
      Ogre::Vector3( cos(cur_yaw-hfov/2.0)*cur_len, sin(cur_yaw-hfov/2.0)*cur_len, tan(-vfov/2.0)*cur_len ),
      Ogre::Vector3( cos(cur_yaw+hfov/2.0)*cur_len, sin(cur_yaw+hfov/2.0)*cur_len, tan(-vfov/2.0)*cur_len ),
      Ogre::Vector3( cos(cur_yaw+hfov/2.0)*cur_len, sin(cur_yaw+hfov/2.0)*cur_len, tan(+vfov/2.0)*cur_len )
    };

    draw_sector(sector_ptr, pts);
    m_sectors.push_back(sector_ptr);
  }

  const double bottom_len = msg->sectors.at(msg->n_horizontal_sectors);
  if (bottom_len != mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN)
  {
    boost::shared_ptr<rviz::MeshShape> sector_ptr = boost::make_shared<rviz::MeshShape>( scene_manager_, frame_node_ );
    draw_topdown_sector(sector_ptr, -bottom_len, vfov, msg->n_horizontal_sectors);
    m_sectors.push_back(sector_ptr);
  }

  const double top_len = msg->sectors.at(msg->n_horizontal_sectors+1);
  if (top_len != mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN)
  {
    boost::shared_ptr<rviz::MeshShape> sector_ptr = boost::make_shared<rviz::MeshShape>( scene_manager_, frame_node_ );
    draw_topdown_sector(sector_ptr, top_len, vfov, msg->n_horizontal_sectors);
    m_sectors.push_back(sector_ptr);
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
  for (auto& sector_ptr : m_sectors)
    sector_ptr->setColor( r, g, b, a );
}
// END_TUTORIAL

} // end namespace mrs_rviz_plugins

