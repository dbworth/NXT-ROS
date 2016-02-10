/*
 * Copyright (c) 2016, David Butterworth
 * All rights reserved.
 *
 * An Rviz plugin to display data from the 
 * LEGO Mindstorms NXT ultrasonic sensor.
 *
 * Based on nxt_rviz_plugin & rviz_plugin_tutorials by Willow Garage.
 * 
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

#include <rviz/ogre_helpers/shape.h> // Cone
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

#include "nxt_ultrasonic_visual.h"

namespace nxt_rviz_plugin
{

NXTUltrasonicVisual::NXTUltrasonicVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Create a SceneNode, which stores the transform of itself relative to
  // the fixed frame so things get rendered in the correct position.
  frame_node_ = parent_node->createChildSceneNode();

  // Create a Shape object within the frame node
  cone_.reset(new rviz::Shape(rviz::Shape::Cone, scene_manager_, frame_node_));
}

NXTUltrasonicVisual::~NXTUltrasonicVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

// Position and orientation from the msg header are passed through to the SceneNode
void NXTUltrasonicVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void NXTUltrasonicVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

// Set the color of the Shape object
void NXTUltrasonicVisual::setColor(float r, float g, float b, float a)
{
  cone_->setColor(r, g, b, a);
}

// Set the pose and size of the Shape object
void NXTUltrasonicVisual::setScale(float range_distance, float spread_angle)
{
  Ogre::Vector3 position;
  position.z = 0.0;
  position.y = 0.0;
  position.x = range_distance/2.0;
  cone_->setPosition(position);

  Ogre::Quaternion orientation = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z);
  cone_->setOrientation(orientation);

  // Set the cone size
  float diameter = sin(spread_angle) * range_distance;
  Ogre::Vector3 scale(diameter, range_distance, diameter);
  cone_->setScale(scale);
}

} // end namespace nxt_rviz_plugin

