/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Local Vision Plugin
 *
 * This plugin simulates local vision data
 *
 * @author Bart Slinger <bartslinger@gmail.com>
 *
 * Modification of gazebo_vision_plugin from:
 * @author Christoph Tobler <christoph@px4.io>
 */

#ifndef _GAZEBO_LOCAL_VISION_PLUGIN_HH_
#define _GAZEBO_LOCAL_VISION_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <odom.pb.h>

#define DEFAULT_PUB_RATE         10       // [Hz]

namespace gazebo
{
class GAZEBO_VISIBLE LocalVisionPlugin : public ModelPlugin
{
public:
  LocalVisionPlugin();
  virtual ~LocalVisionPlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  void getSdfParams(sdf::ElementPtr sdf);

private:
  std::string _namespace;
  physics::ModelPtr _model;
  physics::WorldPtr _world;
  event::ConnectionPtr _updateConnection;

  transport::NodePtr _nh;
  transport::PublisherPtr _pub_odom;

  common::Time _last_pub_time;
  common::Time _last_time;


  int _pub_rate;

};     // class GAZEBO_VISIBLE LocalVisionPlugin
}      // namespace gazebo
#endif // _GAZEBO_VISION_PLUGIN_HH_
