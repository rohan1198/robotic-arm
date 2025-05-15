// Copyright 2023
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

#include "robotic_arm/robotic_arm_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace robotic_arm
{
RoboticArmController::RoboticArmController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RoboticArmController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RoboticArmController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RoboticArmController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn RoboticArmController::on_configure(const rclcpp_lifecycle::State &)
{
  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
  {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };

  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboticArmController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  // Ensure point_interp vectors are correctly sized
  if (point_interp.positions.size() != point_1.positions.size()) {
    point_interp.positions.resize(point_1.positions.size(), 0.0);
  }
  if (point_interp.velocities.size() != point_1.velocities.size()) {
    point_interp.velocities.resize(point_1.velocities.size(), 0.0);
  }

  // Interpolate positions
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_1.positions[i];
  }
  
  // Interpolate velocities if available
  if (!point_1.velocities.empty() && !point_2.velocities.empty()) {
    for (size_t i = 0; i < point_1.velocities.size(); i++)
    {
      point_interp.velocities[i] =
        delta * point_2.velocities[i] + (1.0 - delta) * point_1.velocities[i];
    }
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
{
  // Check if the trajectory has enough points
  if (traj_msg.points.size() < 2) {
    // Not enough points to interpolate, just copy the first point if available
    if (!traj_msg.points.empty()) {
      point_interp = traj_msg.points[0];
    }
    return;
  }

  double traj_len = static_cast<double>(traj_msg.points.size());
  auto last_time = traj_msg.points[traj_msg.points.size() - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;
  
  // Check for division by zero
  if (total_time <= 0.0) {
    point_interp = traj_msg.points[0];
    return;
  }

  // Calculate the interpolation index and delta
  double ratio = cur_time.seconds() / total_time;
  size_t ind = static_cast<size_t>(ratio * (traj_len - 1));
  
  // Ensure we don't go out of bounds
  if (ind >= traj_msg.points.size() - 1) {
    ind = traj_msg.points.size() - 2;
  }
  
  double segment_duration = total_time / (traj_len - 1);
  double segment_start_time = ind * segment_duration;
  double delta = (cur_time.seconds() - segment_start_time) / segment_duration;
  
  // Ensure delta is between 0 and 1
  delta = std::max(0.0, std::min(1.0, delta));
  
  // Perform the interpolation
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type RoboticArmController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  try {
    if (new_msg_)
    {
      trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
      start_time_ = time;
      new_msg_ = false;
    }

    if (trajectory_msg_ != nullptr && !trajectory_msg_->points.empty())
    {
      // Ensure point_interp_ is correctly sized
      if (point_interp_.positions.size() != joint_position_command_interface_.size()) {
        point_interp_.positions.resize(joint_position_command_interface_.size(), 0.0);
      }
      if (point_interp_.velocities.size() != joint_velocity_command_interface_.size()) {
        point_interp_.velocities.resize(joint_velocity_command_interface_.size(), 0.0);
      }

      interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);
      
      // Apply position commands
      for (size_t i = 0; i < joint_position_command_interface_.size() && 
                          i < point_interp_.positions.size(); i++)
      {
        joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
      }
      
      // Apply velocity commands if available
      for (size_t i = 0; i < joint_velocity_command_interface_.size() && 
                          i < point_interp_.velocities.size(); i++)
      {
        joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
      }
    }
  } catch (const std::exception& e) {
    // Log error but don't crash
    RCLCPP_ERROR(rclcpp::get_logger("robotic_arm_controller"), 
                "Error in controller update: %s", e.what());
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RoboticArmController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

}  // namespace robotic_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robotic_arm::RoboticArmController, controller_interface::ControllerInterface
)
