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

#ifndef NXT_ULTRASONIC_DISPLAY_H
#define NXT_ULTRASONIC_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <nxt_msgs/Range.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace nxt_rviz_plugin
{

class NXTUltrasonicVisual;

// Define a new Rviz Display which can be added to the "Displays" panel
class NXTUltrasonicDisplay: public rviz::MessageFilterDisplay<nxt_msgs::Range>
{
Q_OBJECT
public:
  NXTUltrasonicDisplay();
  virtual ~NXTUltrasonicDisplay();

protected:
  // Overrides of protected virtual functions from Display
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  // Qt slots to handle changes in user-editable properties
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  void processMessage(const nxt_msgs::Range::ConstPtr& msg);
 
  // Circular buffer to store the list of visuals.
  // Data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer< boost::shared_ptr<NXTUltrasonicVisual> > visuals_;

  // User-editable property variables
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty* history_length_property_;
};

} // end namespace nxt_rviz_plugin

#endif // NXT_ULTRASONIC_DISPLAY_H

