#include "trx/seed_hardware_interface.hpp"

#include <cmath>
#include <array>
#include <algorithm>
#include <functional>


// パルス値→ラジアン
double pulse_to_rad(int pulse)
{
    const double pulse_max = 59913.0;
    return -(2.0 / pulse_max) * pulse;
}


// ラジアン→パルス値
int rad_to_pulse(double rad)
{
    const double rad_min = -2.0;
    const double rad_max = 0.0;
    const double pulse_max = 59913.0;

    if(rad < rad_min)
        rad = rad_min;

    if(rad > rad_max)
        rad = rad_max;

    return static_cast<int>(-(pulse_max / 2.0) * rad);
}


namespace trx
{


SeedHardwareInterface::~SeedHardwareInterface()
{
    executor_.cancel();

    if(executor_thread_.joinable())
    {
        executor_thread_.join();
    }

    if(seed_)
    {
        seed_->COM_Close();
    }
}



hardware_interface::return_type
SeedHardwareInterface::configure(
    const hardware_interface::HardwareInfo& info)
{

    if(info.joints.size() != 8)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("SeedHW"),
            "Expected 8 joints, got %zu",
            info.joints.size());

        return hardware_interface::return_type::ERROR;
    }


    info_ = info;


    position_.resize(
        info.joints.size(),
        0.0);

    command_.resize(
        info.joints.size(),
        0.0);

    time_.resize(
        info.joints.size(),
        0.0);



    serial_port_ =
        info_.hardware_parameters["serial_port"];


    can_id1_ =
        std::stoi(
            info_.hardware_parameters["can_id1"]);


    can_id2_ =
        std::stoi(
            info_.hardware_parameters["can_id2"]);


    controller_rate_ =
        std::stod(
            info_.hardware_parameters["controller_rate"]);



    RCLCPP_INFO(
        rclcpp::get_logger("SeedHW"),
        "Hardware initialized serial=%s id1=%d id2=%d",
        serial_port_.c_str(),
        can_id1_,
        can_id2_);



    try
    {

        seed_ =
            std::make_unique<SeedCommand>(
                serial_port_,
                115200);


        if(!seed_->COM_Open())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("SeedHW"),
                "Failed to open serial port");

            return hardware_interface::return_type::ERROR;
        }


        RCLCPP_INFO(
            rclcpp::get_logger("SeedHW"),
            "Serial connection opened");

    }
    catch(const std::exception& e)
    {

        RCLCPP_ERROR(
            rclcpp::get_logger("SeedHW"),
            "Serial init failed: %s",
            e.what());

        return hardware_interface::return_type::ERROR;
    }




    node_ =
        std::make_shared<rclcpp::Node>(
            "trx_seed_hardware_interface");



    run_script_service_ =
        node_->create_service<trx::srv::RunScript>(
            "/trx/run_script",
            std::bind(
                &SeedHardwareInterface::runScriptCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));



    joint_trajectory_pub_ =
        node_->create_publisher<
            trajectory_msgs::msg::JointTrajectory>(
                "/joint_trajectory_controller/joint_trajectory",
                rclcpp::SystemDefaultsQoS());



    executor_.add_node(node_);



    executor_thread_ =
        std::thread(
            [this]()
            {
                executor_.spin();
            });



    return hardware_interface::return_type::OK;
}




hardware_interface::return_type
SeedHardwareInterface::start()
{

    RCLCPP_INFO(
        rclcpp::get_logger("SeedHW"),
        "Hardware started");


    if(seed_)
    {

        auto pos_result1 =
            seed_->getPosition(can_id1_);


        position_[3] =
            pulse_to_rad(pos_result1[2]);

        command_[3] =
            position_[3];



        auto pos_result2 =
            seed_->getPosition(can_id2_);


        position_[7] =
            pulse_to_rad(pos_result2[2]);

        command_[7] =
            position_[7];
    }


    return hardware_interface::return_type::OK;
}




hardware_interface::return_type
SeedHardwareInterface::stop()
{

    RCLCPP_INFO(
        rclcpp::get_logger("SeedHW"),
        "Hardware stopped");


    return hardware_interface::return_type::OK;
}





std::vector<hardware_interface::StateInterface>
SeedHardwareInterface::export_state_interfaces()
{

    std::vector<hardware_interface::StateInterface>
        interfaces;


    for(size_t i=0;i<info_.joints.size();i++)
    {

        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_[i]);

    }


    return interfaces;
}




std::vector<hardware_interface::CommandInterface>
SeedHardwareInterface::export_command_interfaces()
{

    std::vector<hardware_interface::CommandInterface>
        interfaces;


    for(size_t i=0;i<info_.joints.size();i++)
    {

        if(!info_.joints[i].command_interfaces.empty())
        {

            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &command_[i]);


            interfaces.emplace_back(
                info_.joints[i].name,
                "time",
                &time_[i]);

        }

    }


    return interfaces;
}






//part2
hardware_interface::return_type
SeedHardwareInterface::read()
{
    std::lock_guard<std::mutex> lock(serial_mtx_);


    auto pos_result1 =
        seed_->getPosition(can_id1_);

    int pulse1 = pos_result1[2];


    if(pos_result1[0] == can_id1_)
    {

        if(pulse1 <= 0)
        {
            position_[0] = 0.0;
            position_[1] = 0.0;
            position_[2] = 0.0;
            position_[3] = 0.0;
        }
        else if(pulse1 <= 52000)
        {
            position_[0] =
                pulse1 * 1.2 / 52000.0;

            position_[1] = 0.0;
            position_[2] = 0.0;

            position_[3] =
                pulse_to_rad(pulse1);
        }
        else if(pulse1 <= 61000)
        {
            position_[0] = 1.2;

            position_[1] =
                (pulse1 - 52000) * 0.8 / 8000.0;

            position_[2] =
                (pulse1 - 52000) * 0.8 / 8000.0;

            position_[3] =
                pulse_to_rad(pulse1);
        }
        else
        {
            position_[0] = 0.0;
            position_[1] = 0.0;
            position_[2] = 0.0;
            position_[3] = 0.0;
        }
    }



    auto pos_result2 =
        seed_->getPosition(can_id2_);

    int pulse2 =
        pos_result2[2];


    if(pos_result2[0] == can_id2_)
    {

        if(pulse2 <= 0)
        {
            position_[4] = 0.0;
            position_[5] = 0.0;
            position_[6] = 0.0;
            position_[7] = 0.0;
        }
        else if(pulse2 <= 52000)
        {
            position_[4] =
                pulse2 * 1.2 / 52000.0;

            position_[5] = 0.0;
            position_[6] = 0.0;

            position_[7] =
                pulse_to_rad(pulse2);
        }
        else if(pulse2 <= 61000)
        {
            position_[4] = 1.2;

            position_[5] =
                (pulse2 - 52000) * 0.8 / 8000.0;

            position_[6] =
                (pulse2 - 52000) * 0.8 / 8000.0;

            position_[7] =
                pulse_to_rad(pulse2);
        }
        else
        {
            position_[4] = 0.0;
            position_[5] = 0.0;
            position_[6] = 0.0;
            position_[7] = 0.0;
        }
    }


    return hardware_interface::return_type::OK;
}





hardware_interface::return_type
SeedHardwareInterface::write()
{
    std::lock_guard<std::mutex> lock(serial_mtx_);


    double lower = -2.0;
    double upper = 0.0;



    double cmd_saturated1 =
        std::max(
            lower,
            std::min(command_[3], upper));


    int cmd_val1 =
        rad_to_pulse(cmd_saturated1);



    seed_->setPositionPulse(
        time_[3] * 3000,
        cmd_val1,
        can_id1_);




    double cmd_saturated2 =
        std::max(
            lower,
            std::min(command_[7], upper));


    int cmd_val2 =
        rad_to_pulse(cmd_saturated2);



    seed_->setPositionPulse(
        time_[7] * 3000,
        cmd_val2,
        can_id2_);



    return hardware_interface::return_type::OK;
}





void SeedHardwareInterface::runScriptCallback(
    const std::shared_ptr<trx::srv::RunScript::Request> request,
    std::shared_ptr<trx::srv::RunScript::Response> response)
{

    std::lock_guard<std::mutex> lock(serial_mtx_);



    if(!seed_)
    {
        response->success = false;
        return;
    }



    RCLCPP_INFO(
        rclcpp::get_logger("SeedHW"),
        "Running script %d on id %d",
        request->script_no,
        request->id);



    seed_->Script_Go(
        request->id,
        request->script_no);



    bool finished =
        seed_->waitForScriptEnd(request->id);



    int idx = -1;

    std::string joint_name;



    if(request->id == can_id1_)
    {
        idx = 3;
        joint_name = "thumb_joint1";
    }
    else if(request->id == can_id2_)
    {
        idx = 7;
        joint_name = "thumb_joint2";
    }



    if(idx >= 0)
    {

        std::array<int,3> pos_result =
            {0,0,0};


        bool got_position = false;



        for(int attempt=0;
            attempt<5 && !got_position;
            attempt++)
        {

            pos_result =
                seed_->getPosition(request->id);


            got_position =
                (pos_result[0] == request->id);
        }




        if(got_position)
        {

            position_[idx] =
                pulse_to_rad(pos_result[2]);


            command_[idx] =
                position_[idx];



            trajectory_msgs::msg::JointTrajectory traj_msg;


            traj_msg.joint_names =
                {joint_name};



            trajectory_msgs::msg::JointTrajectoryPoint point;


            point.positions =
                {position_[idx]};



            point.time_from_start =
                rclcpp::Duration::from_seconds(0.0);



            traj_msg.points =
                {point};



            joint_trajectory_pub_->publish(traj_msg);


            RCLCPP_INFO(
                rclcpp::get_logger("SeedHW"),
                "Re-synced %s pulse=%d",
                joint_name.c_str(),
                pos_result[2]);

        }
    }


    response->success = finished;
}





std::string SeedHardwareInterface::get_name() const
{
    return "SeedHardwareInterface";
}





hardware_interface::status
SeedHardwareInterface::get_status() const
{
    return hardware_interface::status::UNKNOWN;
}


} // namespace trx





#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(
    trx::SeedHardwareInterface,
    hardware_interface::SystemInterface)