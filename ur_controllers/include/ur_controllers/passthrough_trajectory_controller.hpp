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

#ifndef UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
#define UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_

#include <stdint.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "passthrough_trajectory_controller_parameters.hpp"

namespace ur_controllers
{

const double TRANSFER_STATE_IDLE = 0.0;
const double TRANSFER_STATE_WAITING_FOR_POINT = 1.0;
const double TRANSFER_STATE_TRANSFERRING = 2.0;
const double TRANSFER_STATE_TRANSFER_DONE = 3.0;
const double TRANSFER_STATE_IN_MOTION = 4.0;
const double TRANSFER_STATE_DONE = 5.0;
using namespace std::chrono_literals;  // NOLINT

enum CommandInterfaces
{
  /* The PASSTHROUGH_TRAJECTORY_TRANSFER_STATE value is used to keep track of which stage the transfer is in.
  0.0: No trajectory to forward, the controller is idling and ready to receive a new trajectory.
  1.0: The controller has received and accepted a new trajecotry. When the state is 1.0, the controller will write a
  point to the hardware interface.
  2.0: The hardware interface will read the point written from the controller. The state will switch between 1.0
  and 2.0 until all points have been read by the hardware interface.
  3.0: The hardware interface has read all the points, and will now write all the points to the physical robot
  controller.
  4.0: The robot is moving through the trajectory.
  5.0: The robot finished executing the trajectory. */
  PASSTHROUGH_TRAJECTORY_TRANSFER_STATE = 0,
  /* The PASSTHROUGH_TRAJECTORY_ABORT value is used to indicate whether the trajectory has been cancelled from the
   * hardware interface.*/
  PASSTHROUGH_TRAJECTORY_ABORT = 1,
  /* Arrays to hold the values of a point. */
  PASSTHROUGH_TRAJECTORY_POSITIONS_ = 2,
  PASSTHROUGH_TRAJECTORY_VELOCITIES_ = 8,
  PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ = 14,
  PASSTHROUGH_TRAJECTORY_TIME_FROM_START = 20
};

enum StateInterfaces
{
  /* State interface 0 - 17 are joint state interfaces*/

  SPEED_SCALING_FACTOR = 18,
};

// Struct to hold data that has to be transferred between realtime thread and non-realtime threads
struct AsyncInfo
{
  double transfer_state = 0;
  double abort = 0;
  double controller_running = 0;
  bool info_updated = false;
};

class PassthroughTrajectoryController : public controller_interface::ControllerInterface
{
public:
  PassthroughTrajectoryController() = default;
  ~PassthroughTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  RealtimeGoalHandleBuffer rt_active_goal_;         ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;  ///< Timer to frequently check on the running goal

  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  /* Start an action server with an action called: /passthrough_trajectory_controller/forward_joint_trajectory. */
  void start_action_server(void);

  void end_goal();

  bool check_goal_tolerance();

  std::shared_ptr<passthrough_trajectory_controller::ParamListener> passthrough_param_listener_;
  passthrough_trajectory_controller::Params passthrough_params_;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr send_trajectory_action_server_;

  rclcpp_action::GoalResponse
  goal_received_callback(const rclcpp_action::GoalUUID& uuid,
                         std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  void goal_accepted_callback(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;

  std::vector<std::string> joint_state_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

  bool check_positions(std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  bool check_velocities(std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  bool check_accelerations(std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

  trajectory_msgs::msg::JointTrajectory active_joint_traj_;
  std::vector<control_msgs::msg::JointTolerance> path_tolerance_;
  std::vector<control_msgs::msg::JointTolerance> goal_tolerance_;
  rclcpp::Duration goal_time_tolerance_ = rclcpp::Duration::from_nanoseconds(0);

  std::atomic<size_t> current_index_;
  std::atomic<bool> trajectory_active_;
  rclcpp::Duration active_trajectory_elapsed_time_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Duration max_trajectory_time_ = rclcpp::Duration::from_nanoseconds(0);
  double scaling_factor_;
  uint32_t number_of_joints_;
  static constexpr double NO_VAL = std::numeric_limits<double>::quiet_NaN();

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> scaling_state_interface_;

  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace ur_controllers
#endif  // UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
