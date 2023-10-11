// Copyright 2023, FZI Forschungszentrum Informatik
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
 * \author  Felix Exner <exner@fzi.de>
 * \date    2023-10-23
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__FORWARDING_CONTROLLER_HPP_
#define UR_CONTROLLERS__FORWARDING_CONTROLLER_HPP_

#include <memory>
#include <controller_interface/controller_interface.hpp>

#include "forwarding_controller_parameters.hpp"

namespace ur_controllers
{
class ForwardingController : public controller_interface::ControllerInterface
{
public:
  ForwardingController() = default;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

private:
  // Parameters from ROS for joint_trajectory_controller
  std::shared_ptr<forwarding_controller::ParamListener> param_listener_;
  forwarding_controller::Params params_;
};
}  // namespace ur_controllers
#endif  // UR_CONTROLLERS__FORWARDING_CONTROLLER_HPP_
