/*
 * Copyright (c) 2016, David Butterworth
 * All rights reserved.
 *
 * An Rviz plugin to display data from the 
 * LEGO Mindstorms NXT 2.0 color sensor.
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

#ifndef NXT_COLOR_VISUAL_H
#define NXT_COLOR_VISUAL_H

#include <nxt_msgs/Color.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape; // Cylinder
}

namespace nxt_rviz_plugin
{

// Declare the visual class for this Rviz Display.
// Each instance represents the visualization of a single msg.
class NXTColorVisual
{
public:
  // Constructor, create the visual and add it to the scene
  // in an un-configured state
  NXTColorVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  virtual ~NXTColorVisual();

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);
  void setColor(float r, float g, float b, float a);
  void setScale(float length);

private:
  boost::shared_ptr<rviz::Shape> cylinder_;
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here so the destructor can
  // ask it to destroy the frame node
  Ogre::SceneManager* scene_manager_;
};

} // end namespace nxt_rviz_plugin

#endif // NXT_COLOR_VISUAL_H

