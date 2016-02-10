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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "nxt_color_visual.h"
#include "nxt_color_display.h"

namespace nxt_rviz_plugin
{
NXTColorDisplay::NXTColorDisplay()
{
  // Properties for this Display that the user
  // can change from within Rviz:
  cylinder_length_property_ = new rviz::FloatProperty("Length", 0.003,
                                                      "Length of the cylinder visual.",
                                                      this, SLOT(updateCylinderLength()));
  alpha_property_ = new rviz::FloatProperty("Alpha", 0.5,
                                            "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateAlpha()));
  history_length_property_ = new rviz::IntProperty("History Length", 1,
                                                   "Number of prior measurements to display.",
                                                   this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void NXTColorDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

NXTColorDisplay::~NXTColorDisplay()
{
}

// Clear this display back to its initial state
void NXTColorDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Qt slot: set the current color and alpha values for each visual.
void NXTColorDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  for(size_t i=0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColor(color_.r, color_.g, color_.b, alpha);
  }
}

// Qt slot: set the number of past visuals to show
void NXTColorDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// Qt slot: set the length of the cylinder visual
void NXTColorDisplay::updateCylinderLength()
{
  float length = cylinder_length_property_->getFloat();

  for(size_t i=0; i < visuals_.size(); i++)
  {
    visuals_[i]->setScale(length);
  }
}

// Callback to handle an incoming ROS message
void NXTColorDisplay::processMessage(const nxt_msgs::Color::ConstPtr& msg)
{
  float length = cylinder_length_property_->getFloat();

  // Get the transform from the fixed frame to the frame
  // in the header of this msg
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                 msg->header.stamp,
                                                 position,
                                                 orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // We are keeping a circular buffer of visual pointers.
  // Get the next ptr or make a new one.
  boost::shared_ptr<NXTColorVisual> visual;
  if(visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new NXTColorVisual(context_->getSceneManager(), scene_node_));
  }

  // Update the pose of the Ogre::SceneNode
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  // Set the pose and size of the cylinder visual
  visual->setScale(length);

  // Store the RGB color value from the msg
  // and set the color of the cylinder visual
  float alpha = alpha_property_->getFloat();
  color_.r = msg->r;
  color_.g = msg->g;
  color_.b = msg->b;
  visual->setColor(color_.r, color_.g, color_.b, alpha);

  // Add visual to end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace nxt_rviz_plugin

// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTColorDisplay,rviz::Display)

