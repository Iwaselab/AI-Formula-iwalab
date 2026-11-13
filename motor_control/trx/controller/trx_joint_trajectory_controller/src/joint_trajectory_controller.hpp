#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class TrxJointTrajectoryController : public controller_interface::ControllerInterface
{
public:
  TrxJointTrajectoryController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

  void topic_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg);

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_pub_;
  std::vector<double> target_positions_;
  std::vector<double> target_time_{0.0, 0.0};
};
