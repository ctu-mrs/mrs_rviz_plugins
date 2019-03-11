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

#include <boost/smart_ptr/make_shared.hpp>

#include "mrs_bumper_visual.h"

namespace vis_mrs_bumper
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

void MRS_Bumper_Visual::setMessage( const mrs_bumper::ObstacleSectors::ConstPtr& msg )
{
  const double hfov = 2.0*M_PI/msg->n_horizontal_sectors;
  const double vfov = msg->sectors_vertical_fov;
  m_sector_lines.reserve(msg->n_horizontal_sectors+2);
  
  for (unsigned sector_it = 0; sector_it < msg->n_horizontal_sectors; sector_it++)
  {
    const double cur_len = msg->sectors.at(sector_it);
    if (cur_len == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN)
      continue;

    boost::shared_ptr<rviz::BillboardLine> line_ptr = boost::make_shared<rviz::BillboardLine>( scene_manager_, frame_node_ );
    const double cur_yaw = hfov*sector_it;
    line_ptr->addPoint(Ogre::Vector3( 0, 0, 0 ));
    line_ptr->addPoint(Ogre::Vector3( cos(cur_yaw)*cur_len, sin(cur_yaw)*cur_len, tan(vfov/2.0)*cur_len ));
    line_ptr->addPoint(Ogre::Vector3( cos(cur_yaw)*cur_len, sin(cur_yaw)*cur_len, tan(-vfov/2.0)*cur_len ));
    line_ptr->addPoint(Ogre::Vector3( 0, 0, 0 ));

    m_sector_lines.push_back(line_ptr);
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
  for (auto& line_ptr : m_sector_lines)
    line_ptr->setColor( r, g, b, a );
}
// END_TUTORIAL

} // end namespace vis_mrs_bumper

