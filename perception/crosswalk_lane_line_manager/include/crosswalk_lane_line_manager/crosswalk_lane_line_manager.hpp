#ifndef CROSSWALK_LANE_LINE_MANAGER_HPP
#define CROSSWALK_LANE_LINE_MANAGER_HPP

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "aiformula_interfaces/msg/rect_multi_array.hpp"

namespace aiformula {

class CrosswalkLaneLineManager : public rclcpp::Node {
public:
    CrosswalkLaneLineManager();

private:
    enum LaneLineIndex : size_t { LEFT = 0, RIGHT = 1, CENTER = 2 };

    struct Line {
        tf2::Vector3 first;
        tf2::Vector3 last;
    };

    void initConnections();
    void laneLineCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t side);
    void crosswalkCallback(const aiformula_interfaces::msg::RectMultiArray::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishLines();
    void startLineCollection();
    void collectLineFrame(size_t side);
    void finishCompensation();

    std::optional<Line> captureLine(const sensor_msgs::msg::PointCloud2& msg);
    std::optional<Line> averageRecentLines(const std::deque<Line>& lines) const;
    std::optional<Line> transformLineToVehicle(const Line& line, const rclcpp::Time& stamp);
    sensor_msgs::msg::PointCloud2 createLineMessage(const Line& line, const rclcpp::Time& stamp);
    sensor_msgs::msg::PointCloud2 createCenterLineMessage(
        const Line& left, const Line& right, const rclcpp::Time& stamp);
    std::optional<float> crosswalkWidth(
        const aiformula_interfaces::msg::RectMultiArray& msg) const;
    static std::vector<tf2::Vector3> readPoints(const sensor_msgs::msg::PointCloud2& msg);

    std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr, 3> lane_line_subs_;
    rclcpp::Subscription<aiformula_interfaces::msg::RectMultiArray>::SharedPtr crosswalk_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, 3> lane_line_pubs_;

    std::array<std::optional<Line>, 2> saved_lines_;
    std::array<std::deque<Line>, 2> recent_lines_;
    std::array<sensor_msgs::msg::PointCloud2::SharedPtr, 3> latest_camera_lines_;
    std::array<bool, 2> collecting_line_received_{};
    bool crosswalk_active_{false};
    bool collecting_lines_{false};
    bool have_odometry_{false};
    rclcpp::Time latest_odometry_stamp_;
    std::string odom_frame_id_;
    std::string vehicle_frame_id_;
    double xmin_{0.0};
    double xmax_{20.0};
    double spacing_{1.0};
    double compensation_fit_xmax_{10.0};
    double crosswalk_bbox_width_threshold_{100.0};
    int crosswalk_exit_missing_frames_{3};
    int crosswalk_missing_frames_{0};
    int temporal_filter_frames_{5};
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace aiformula

#endif  // CROSSWALK_LANE_LINE_MANAGER_HPP
