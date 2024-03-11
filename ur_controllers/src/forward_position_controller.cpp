// Copyright 2024, Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jacob Larsen jala@universal-robots.com
 * \date    2024-03-11
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/forward_position_controller.hpp"

namespace forward_controller
{
controller_interface::CallbackReturn ForwardPositionController::on_init()
{
  forward_param_listener_ = std::make_shared<forward_position_controller::ParamListener>(get_node());
  forward_params_ = forward_param_listener_->get_params();

  return ControllerInterface::on_init();
}

controller_interface::InterfaceConfiguration ForwardPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;

  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  conf.names.push_back(forward_params_.speed_scaling_interface_name);

  return conf;
}

controller_interface::CallbackReturn ForwardPositionController::on_activate(const rclcpp_lifecycle::State& state)
{
  return ControllerInterface::on_activate(state);
}

controller_interface::return_type ForwardPositionController::update(const rclcpp::Time& time,
                                                                    const rclcpp::Duration& period)
{
  std::cout << state_interfaces_.back().get_value() << std::endl;
  return controller_interface::return_type::OK;
}

}  // namespace forward_controller
