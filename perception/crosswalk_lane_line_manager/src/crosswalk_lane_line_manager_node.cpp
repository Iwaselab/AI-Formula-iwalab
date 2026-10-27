#include "crosswalk_lane_line_manager/crosswalk_lane_line_manager.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::CrosswalkLaneLineManager>());
    rclcpp::shutdown();
    return 0;
}
