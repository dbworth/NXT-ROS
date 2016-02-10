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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "nxt_ultrasonic_visual.h"
#include "nxt_ultrasonic_display.h"

namespace nxt_rviz_plugin
{
NXTUltrasonicDisplay::NXTUltrasonicDisplay()
{
  // Properties for this Display that the user
  // can change from within Rviz:
  color_property_ = new rviz::ColorProperty("Color", QColor(25, 255, 0), // green
                                            "Color to draw the distance.",
                                            this, SLOT(updateColorAndAlpha()));
  alpha_property_ = new rviz::FloatProperty("Alpha", 0.5,
                                            "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateColorAndAlpha()));
  history_length_property_ = new rviz::IntProperty("History Length", 1,
                                                   "Number of prior measurements to display.",
                                                   this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void NXTUltrasonicDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

NXTUltrasonicDisplay::~NXTUltrasonicDisplay()
{
}

// Clear this display back to its initial state
void NXTUltrasonicDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Qt slot: set the current color and alpha values for each visual.
void NXTUltrasonicDisplay::updateColorAndAlpha()
{
  // Get the values set by the user
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for(size_t i=0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
  }
}

// Qt slot: set the number of past visuals to show
void NXTUltrasonicDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// Callback to handle an incoming ROS message
void NXTUltrasonicDisplay::processMessage(const nxt_msgs::Range::ConstPtr& msg)
{
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
  boost::shared_ptr<NXTUltrasonicVisual> visual;
  if(visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new NXTUltrasonicVisual(context_->getSceneManager(), scene_node_));
  }

  // Update the pose of the Ogre::SceneNode
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  updateColorAndAlpha();

  // Set the pose and size of the cone visual,
  // based on data from the msg
  visual->setScale(msg->range, msg->spread_angle);

  // Add visual to end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace nxt_rviz_plugin

// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTUltrasonicDisplay,rviz::Display)

