
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>
#include <cstdint>
#include <functional>
#include <chrono>

#include "trx/seed_command.hpp"


class RearSteeringNode : public rclcpp::Node
{
public:

    RearSteeringNode()
    : Node("rear_steering_node"),
      motor_ready_(false),
      previous_state_("UNKNOWN"),
      neutral_transition_pending_(false)
    {
        // ==========================================
        // cmd_velの購読
        // ==========================================

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/aiformula_control/twist_mux/cmd_vel",
            10,
            std::bind(
                &RearSteeringNode::cmdVelCallback,
                this,
                std::placeholders::_1));


        // ==========================================
        // モータ初期化
        // ==========================================

        motor_ = std::make_shared<SeedCommand>(
            "/dev/ttyACM0",
            115200);

        if (!motor_->COM_Open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "COM Open Failed");

            motor_ready_ = false;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Motor COM Open Success");

            const uint8_t id1 = 1;
            const uint8_t id2 = 2;

            // サーボON
            motor_->onServo(id1, 1);
            motor_->onServo(id2, 1);

            RCLCPP_INFO(
                this->get_logger(),
                "Motors servo ON");

            motor_ready_ = true;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Rear steering node started.");
    }


    // ==========================================
    // デストラクタ
    // ==========================================

    ~RearSteeringNode()
    {
        if (neutral_timer_)
        {
            neutral_timer_->cancel();
        }

        if (motor_ready_ && motor_)
        {
            const uint8_t id1 = 1;
            const uint8_t id2 = 2;

            // サーボOFF
            motor_->onServo(id1, 0);
            motor_->onServo(id2, 0);

            // COM通信終了
            motor_->COM_Close();

            RCLCPP_INFO(
                this->get_logger(),
                "Motors servo OFF and COM closed.");
        }
    }


private:

    // ==========================================
    // モータ設定
    // ==========================================

    //static constexpr int16_t speed = 2000;

    //static constexpr int32_t neutral_pulse_ = 10000;

    //static constexpr int32_t straight_pulse_ = 30000;


    // ==========================================
    // cmd_vel Callback
    // ==========================================

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double angular_z = msg->angular.z;

        std::string current_state;


        // ==========================================
        // angular.zによる旋回状態の判定
        // ==========================================

        if (angular_z == 0.0)
        {
            current_state = "STRAIGHT";
        }
        else if (angular_z > 0.0)
        {
            current_state = "RIGHT";
        }
        else
        {
            current_state = "LEFT";
        }


        // ==========================================
        // 状態が変化したときだけ処理
        // ==========================================

        if (current_state == previous_state_)
        {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "angular.z = %.3f -> %s",
            angular_z,
            current_state.c_str());


        // ==========================================
        // モータが使用可能な場合のみ動作
        // ==========================================

        if (!motor_ready_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Motor is not ready. Command was not sent.");

            previous_state_ = current_state;
            return;
        }


        // ==========================================
        // ニュートラル動作中に旋回指令が来た場合
        // ==========================================

        if (neutral_transition_pending_)
        {
            neutral_timer_->cancel();

            neutral_transition_pending_ = false;

            RCLCPP_WARN(
                this->get_logger(),
                "Neutral transition cancelled.");
        }


        // ==========================================
        // 旋回から直進に移行する場合
        // ==========================================

        if (current_state == "STRAIGHT" &&
            (previous_state_ == "RIGHT" ||
             previous_state_ == "LEFT"))
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Transition: TURN -> NEUTRAL -> STRAIGHT");

            // ニュートラル動作
            startNeutralTransition();
        }


        // ==========================================
        // RIGHT
        // ==========================================

        else if (current_state == "RIGHT")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Rear steering: RIGHT");

            // ID1
            motor_->actuateAbsolutePositionBySpeed(
                1,
                3000,
                75000);

            // ID2
            motor_->actuateAbsolutePositionBySpeed(
                2,
                3000,
                45000);
        }


        // ==========================================
        // LEFT
        // ==========================================

        else if (current_state == "LEFT")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Rear steering: LEFT");

            // ID1
            motor_->actuateAbsolutePositionBySpeed(
                1,
                3000,
                45000);

            // ID2
            motor_->actuateAbsolutePositionBySpeed(
                2,
                3000,
                75000);
        }


        // ==========================================
        // STRAIGHT（初回起動時など）
        // ==========================================

        else if (current_state == "STRAIGHT")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Rear steering: STRAIGHT");

            // ID1
            motor_->actuateAbsolutePositionBySpeed(
                1,
                3000,
                75000);

            // ID2
            motor_->actuateAbsolutePositionBySpeed(
                2,
                3000,
                75000);
        }


        // 現在の状態を保存
        previous_state_ = current_state;
    }


    // ==========================================
    // ニュートラル動作開始
    // ==========================================

    void startNeutralTransition()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Rear steering: NEUTRAL");

        // ID1
        motor_->actuateAbsolutePositionBySpeed(
            1,
            3000,
            45000);

        // ID2
        motor_->actuateAbsolutePositionBySpeed(
            2,
            3000,
            45000);

        neutral_transition_pending_ = true;


        // ==========================================
        // 1.2秒後に直進動作へ移行
        // ==========================================

        neutral_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1200),
            std::bind(
                &RearSteeringNode::finishNeutralTransition,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Neutral operation started. Waiting...");
    }


    // ==========================================
    // ニュートラル終了・直進動作
    // ==========================================

    void finishNeutralTransition()
    {
        if (!neutral_transition_pending_ || !motor_ready_)
        {
            return;
        }

        // タイマー停止
        neutral_timer_->cancel();

        neutral_transition_pending_ = false;


        RCLCPP_INFO(
            this->get_logger(),
            "Neutral operation finished.");

        // ==========================================
        // 直進位置へ移動
        // ==========================================

        RCLCPP_INFO(
            this->get_logger(),
            "Rear steering: STRAIGHT");

        // ID1
        motor_->actuateAbsolutePositionBySpeed(
            1,
            3000,
            75000);

        // ID2
        motor_->actuateAbsolutePositionBySpeed(
            2,
            3000,
            75000);

        RCLCPP_INFO(
            this->get_logger(),
            "Straight operation started.");
    }


    // ==========================================
    // ROS2 Subscription
    // ==========================================

    rclcpp::Subscription<
        geometry_msgs::msg::Twist
    >::SharedPtr subscription_;


    // ==========================================
    // モータ制御
    // ==========================================

    std::shared_ptr<SeedCommand> motor_;

    bool motor_ready_;


    // ==========================================
    // 状態管理
    // ==========================================

    std::string previous_state_;

    bool neutral_transition_pending_;


    // ==========================================
    // ニュートラル用タイマー
    // ==========================================

    rclcpp::TimerBase::SharedPtr neutral_timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RearSteeringNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}