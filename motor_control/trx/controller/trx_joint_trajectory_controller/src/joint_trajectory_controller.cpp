#include "joint_trajectory_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"

TrxJointTrajectoryController::TrxJointTrajectoryController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn TrxJointTrajectoryController::on_init()
{
  target_positions_.resize(2, 0.0);
  target_time_.resize(2, 0.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TrxJointTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.push_back( std::string("thumb_joint1/") + hardware_interface::HW_IF_POSITION);
  conf.names.push_back( std::string("thumb_joint1/") + "time");
  conf.names.push_back( std::string("thumb_joint2/") + hardware_interface::HW_IF_POSITION);
  conf.names.push_back( std::string("thumb_joint2/") + "time");
  return conf;
}

controller_interface::InterfaceConfiguration TrxJointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.push_back( std::string("thumb_joint1/") + hardware_interface::HW_IF_POSITION);
  conf.names.push_back( std::string("thumb_joint2/") + hardware_interface::HW_IF_POSITION);
  return conf;
}

controller_interface::return_type TrxJointTrajectoryController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
bool ret0 = command_interfaces_[0].set_value(target_positions_[0]);
(void)ret0;

bool ret1 = command_interfaces_[1].set_value(target_time_[0]);
(void)ret1;

bool ret2 = command_interfaces_[2].set_value(target_positions_[1]);
(void)ret2;

bool ret3 = command_interfaces_[3].set_value(target_time_[1]);
(void)ret3;

  control_msgs::msg::JointTrajectoryControllerState state_msg;
  state_msg.joint_names = {"thumb_joint1", "thumb_joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint actual_pt, desired_pt, error_pt;

  actual_pt.positions = {target_positions_[0],target_positions_[1]};
  desired_pt.positions = {target_positions_[0],target_positions_[1]};
  error_pt.positions = {0.0, 0.0};

  state_msg.reference = desired_pt;
  state_msg.feedback  = actual_pt;
  state_msg.error     = error_pt;

  controller_state_pub_->publish(state_msg);
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TrxJointTrajectoryController::on_configure(const rclcpp_lifecycle::State &)
{
  // create subscriber and publishers
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&TrxJointTrajectoryController::topic_callback, this, std::placeholders::_1));

    controller_state_pub_ = get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
    "/joint_trajectory_controller/controller_state", rclcpp::SystemDefaultsQoS());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrxJointTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & iface : state_interfaces_)
  {
    const std::string joint = iface.get_prefix_name();
    const std::string type  = iface.get_interface_name();

    if (type != "position") continue;

    if (joint == "thumb_joint1")
      target_positions_[0] = iface.get_value();

    if (joint == "thumb_joint2")
      target_positions_[1] = iface.get_value();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrxJointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrxJointTrajectoryController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void TrxJointTrajectoryController::topic_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
for (size_t i = 0; i < msg->joint_names.size(); ++i) {
  if (msg->joint_names[i] == "thumb_joint1") {
    target_positions_[0] = msg->points[0].positions[i];
    target_time_[0] = rclcpp::Duration(msg->points[0].time_from_start).seconds();
  } else if (msg->joint_names[i] == "thumb_joint2") {
    target_positions_[1] = msg->points[0].positions[i];
    target_time_[1] = rclcpp::Duration(msg->points[0].time_from_start).seconds();
  }
}
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(TrxJointTrajectoryController, controller_interface::ControllerInterface)
