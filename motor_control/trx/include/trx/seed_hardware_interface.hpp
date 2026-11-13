#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "trx/seed_command.hpp"
#include "trx/srv/run_script.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


namespace trx
{

class SeedHardwareInterface : public hardware_interface::SystemInterface
{

public:

  RCLCPP_SHARED_PTR_DEFINITIONS(SeedHardwareInterface)


  ~SeedHardwareInterface() override;


  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo& info) override;


  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;


  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;


  hardware_interface::return_type start() override;


  hardware_interface::return_type stop() override;


  hardware_interface::return_type read() override;


  hardware_interface::return_type write() override;


  std::string get_name() const override;


  hardware_interface::status get_status() const override;

private:


  void runScriptCallback(
    const std::shared_ptr<trx::srv::RunScript::Request> request,
    std::shared_ptr<trx::srv::RunScript::Response> response);



  std::string serial_port_;

  int can_id1_;
  int can_id2_;

  double controller_rate_;



  std::vector<double> position_;

  std::vector<double> command_;

  std::vector<double> time_;



  hardware_interface::HardwareInfo info_;


  std::unique_ptr<SeedCommand> seed_;


  std::mutex serial_mtx_;



  rclcpp::Node::SharedPtr node_;


  rclcpp::Service<trx::srv::RunScript>::SharedPtr
    run_script_service_;


  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_trajectory_pub_;



  rclcpp::executors::SingleThreadedExecutor executor_;

  std::thread executor_thread_;

};


} // namespace trx