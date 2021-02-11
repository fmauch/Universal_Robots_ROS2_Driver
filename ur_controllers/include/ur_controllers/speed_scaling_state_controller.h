// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-10
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED
#define UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64.hpp"

namespace ur_controllers
{
class SpeedScalingStateController : public controller_interface::ControllerInterface
{
public:
  SpeedScalingStateController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
  bool init_sensor_data();

protected:
  std::vector<std::string> sensor_names_;
  rclcpp::Time last_publish_time_;
  double publish_rate_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> speed_scaling_state_publisher_;
  std_msgs::msg::Float64 speed_scaling_state_msg_;

  std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
};
}  // namespace ur_controllers
#endif  // ifndef UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED
