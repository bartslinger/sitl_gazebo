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

#include <gazebo_local_vision_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(LocalVisionPlugin)

LocalVisionPlugin::LocalVisionPlugin() : ModelPlugin()
{
}

LocalVisionPlugin::~LocalVisionPlugin()
{
  _updateConnection->~Connection();
}

void LocalVisionPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  _namespace.clear();
  if (sdf->HasElement("robotNamespace")) {
    _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_vision_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pub_rate")) {
    _pub_rate = sdf->GetElement("pub_rate")->Get<int>();
  } else {
    _pub_rate = DEFAULT_PUB_RATE;
    gzerr << "[gazebo_vision_plugin] Using default publication rate of " << DEFAULT_PUB_RATE << " Hz\n";
  }
}

void LocalVisionPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // Store model
  _model = model;

  _world = _model->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
#else
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = ignitionFromGazeboMath(_model->GetWorldPose());
#endif


  _nh = transport::NodePtr(new transport::Node());
  _nh->Init(_namespace);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&LocalVisionPlugin::OnUpdate, this, _1));

  _pub_odom = _nh->Advertise<odom_msgs::msgs::odom>("~/" + _model->GetName() + "/vision_odom", 10);
}

void LocalVisionPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = _world->SimTime();
#else
  common::Time current_time = _world->GetSimTime();
#endif
  double dt = (current_time - _last_pub_time).Double();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_model_world = _model->WorldPose();
#else
    ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(_model->GetWorldPose());
#endif

    // Fill odom msg
    odom_msgs::msgs::odom odom_msg;
    odom_msg.set_usec(current_time.Double() * 1e6);
    odom_msg.set_x(pose_model_world.Pos().X());
    odom_msg.set_y(pose_model_world.Pos().Y());
    odom_msg.set_z(pose_model_world.Pos().Z());
    odom_msg.set_roll(pose_model_world.Rot().Roll());
    odom_msg.set_pitch(pose_model_world.Rot().Pitch());
    odom_msg.set_yaw(pose_model_world.Rot().Yaw());

    _last_pub_time = current_time;

    // Only publish in this region to simulate visibility of the marker.
    // Could be more accurate but this is meant to test kalman filter mode switching
    if (abs(pose_model_world.Pos().X()) < 2.0 &&
        abs(pose_model_world.Pos().Y()) < 2.0 &&
        abs(pose_model_world.Pos().Z() < 5.0)) {
      // publish odom msg
      _pub_odom->Publish(odom_msg);
    }
  }
}

} // namespace gazebo
