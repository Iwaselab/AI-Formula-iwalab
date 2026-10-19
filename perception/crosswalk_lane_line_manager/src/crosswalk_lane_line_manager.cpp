#include "crosswalk_lane_line_manager/crosswalk_lane_line_manager.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "common_cpp/get_ros_parameter.hpp"

namespace aiformula {

CrosswalkLaneLineManager::CrosswalkLaneLineManager()
    : Node("crosswalk_lane_line_manager"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
    xmin_ = getRosParameter<double>(this, "lane_line_publisher.roi.xmin");
    xmax_ = getRosParameter<double>(this, "lane_line_publisher.roi.xmax");
    spacing_ = getRosParameter<double>(this, "lane_line_publisher.spacing");
    compensation_fit_xmax_ = getRosParameter<double>(this, "compensation_fit_xmax");
    crosswalk_bbox_width_threshold_ = getRosParameter<double>(this, "crosswalk_bbox_width_threshold");
    if (xmax_ <= xmin_ || spacing_ <= 0.0) {
        throw std::invalid_argument("lane line ROI and spacing must define a positive range");
    }
    if (crosswalk_bbox_width_threshold_ < 0.0) {
        throw std::invalid_argument("crosswalk_bbox_width_threshold must be non-negative");
    }
    if (compensation_fit_xmax_ <= 0.0) {
        throw std::invalid_argument("compensation_fit_xmax must be positive");
    }
    initConnections();
}

void CrosswalkLaneLineManager::initConnections() {
    const auto subscription_qos = rclcpp::SensorDataQoS();
    const auto publisher_qos = rclcpp::QoS(10);
    lane_line_subs_[0] = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_lane_lines/left", subscription_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { laneLineCallback(msg, 0); });
    lane_line_subs_[1] = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_lane_lines/right", subscription_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { laneLineCallback(msg, 1); });
    lane_line_subs_[2] = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_lane_lines/center", subscription_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { laneLineCallback(msg, 2); });
    crosswalk_sub_ = create_subscription<aiformula_interfaces::msg::RectMultiArray>(
        "crosswalk_detection", subscription_qos,
        std::bind(&CrosswalkLaneLineManager::crosswalkCallback, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", subscription_qos,
        std::bind(&CrosswalkLaneLineManager::odometryCallback, this, std::placeholders::_1));
    lane_line_pubs_[0] = create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/left", publisher_qos);
    lane_line_pubs_[1] = create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/right", publisher_qos);
    lane_line_pubs_[2] = create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/center", publisher_qos);
}

void CrosswalkLaneLineManager::laneLineCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t side) {
    latest_camera_lines_[side] = msg;
    if (crosswalk_active_ && !saved_lines_[side]) saved_lines_[side] = captureLine(*msg);
    if (!crosswalk_active_) {
        lane_line_pubs_[side]->publish(*msg);
        return;
    }
    publishLines();
}

void CrosswalkLaneLineManager::crosswalkCallback(
    const aiformula_interfaces::msg::RectMultiArray::SharedPtr msg) {
    const bool detected = hasCrosswalk(*msg);
    const bool start_compensation = shouldStartCompensation(*msg);
    if (start_compensation && !crosswalk_active_) {
        crosswalk_active_ = true;
        for (size_t side = 0; side < saved_lines_.size(); ++side) {
            if (latest_camera_lines_[side]) saved_lines_[side] = captureLine(*latest_camera_lines_[side]);
        }
        RCLCPP_INFO(get_logger(),
                    "Crosswalk bbox width exceeded %.1f px; switching to preserved lane lines",
                    crosswalk_bbox_width_threshold_);
    } else if (!detected && crosswalk_active_) {
        crosswalk_active_ = false;
        saved_lines_ = {};
        RCLCPP_INFO(get_logger(), "Crosswalk cleared; switching back to camera lane lines");
    }
    publishLines();
}

void CrosswalkLaneLineManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    have_odometry_ = true;
    latest_odometry_stamp_ = rclcpp::Time(msg->header.stamp);
    if (crosswalk_active_) publishLines();
}

std::optional<CrosswalkLaneLineManager::Line> CrosswalkLaneLineManager::captureLine(
    const sensor_msgs::msg::PointCloud2& msg) {
    const auto all_points = readPoints(msg);
    std::vector<tf2::Vector3> points;
    for (const auto& point : all_points) {
        if (point.x() >= 0.0 && point.x() <= compensation_fit_xmax_) {
            points.emplace_back(point);
        }
    }
    if (points.size() < 2) return std::nullopt;

    const auto stamp = rclcpp::Time(msg.header.stamp);
    try {
        const auto transform = tf_buffer_.lookupTransform(odom_frame_id_, msg.header.frame_id, stamp,
                                                           tf2::durationFromSec(0.05));
        tf2::Transform odom_T_camera_line;
        tf2::fromMsg(transform.transform, odom_T_camera_line);
        return Line{odom_T_camera_line * points.front(), odom_T_camera_line * points.back()};
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot preserve lane line: %s", e.what());
        return std::nullopt;
    }
}

std::optional<CrosswalkLaneLineManager::Line> CrosswalkLaneLineManager::transformLineToVehicle(
    const Line& line, const rclcpp::Time& stamp) {
    try {
        const auto transform = tf_buffer_.lookupTransform(vehicle_frame_id_, odom_frame_id_, stamp,
                                                           tf2::durationFromSec(0.05));
        tf2::Transform vehicle_T_odom;
        tf2::fromMsg(transform.transform, vehicle_T_odom);
        return Line{vehicle_T_odom * line.first, vehicle_T_odom * line.last};
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot transform preserved lane line: %s", e.what());
        return std::nullopt;
    }
}

sensor_msgs::msg::PointCloud2 CrosswalkLaneLineManager::createLineMessage(
    const Line& line, const rclcpp::Time& stamp) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const double dx = line.last.x() - line.first.x();
    const double dy = line.last.y() - line.first.y();
    const double length = std::hypot(dx, dy);
    const double ux = length > 1e-9 ? dx / length : 1.0;
    const double uy = length > 1e-9 ? dy / length : 0.0;
    const double vy = ux;
    const double center_x = 0.5 * (line.first.x() + line.last.x());
    const double center_y = 0.5 * (line.first.y() + line.last.y());
    const double direction_x = std::abs(ux) > 1e-6 ? ux : 1.0;
    const double direction_y = std::abs(ux) > 1e-6 ? uy : 0.0;

    for (double x = xmin_; x <= xmax_ + 1e-9; x += spacing_) {
        const double y = std::abs(direction_x) > 1e-6
                             ? center_y + direction_y * (x - center_x) / direction_x
                             : center_y + vy * (x - center_x);
        auto& point = cloud.points.emplace_back();
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = 0.0F;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = vehicle_frame_id_;
    return msg;
}

void CrosswalkLaneLineManager::publishLines() {
    if (!crosswalk_active_ || !have_odometry_) {
        for (size_t side = 0; side < latest_camera_lines_.size(); ++side)
            if (latest_camera_lines_[side]) lane_line_pubs_[side]->publish(*latest_camera_lines_[side]);
        return;
    }

    const auto stamp = latest_odometry_stamp_;
    for (size_t side = 0; side < saved_lines_.size(); ++side) {
        if (!saved_lines_[side]) {
            if (latest_camera_lines_[side]) lane_line_pubs_[side]->publish(*latest_camera_lines_[side]);
            continue;
        }
        const auto current_line = transformLineToVehicle(*saved_lines_[side], stamp);
        if (current_line) lane_line_pubs_[side]->publish(createLineMessage(*current_line, stamp));
    }
}

bool CrosswalkLaneLineManager::hasCrosswalk(
    const aiformula_interfaces::msg::RectMultiArray& msg) const {
    return std::any_of(msg.rects.begin(), msg.rects.end(),
                       [](const auto& rect) { return rect.class_id == 0; });
}

bool CrosswalkLaneLineManager::shouldStartCompensation(
    const aiformula_interfaces::msg::RectMultiArray& msg) const {
    return std::any_of(msg.rects.begin(), msg.rects.end(),
                       [this](const auto& rect) {
                           return rect.class_id == 0 &&
                                  rect.width > crosswalk_bbox_width_threshold_;
                       });
}

std::vector<tf2::Vector3> CrosswalkLaneLineManager::readPoints(
    const sensor_msgs::msg::PointCloud2& msg) {
    std::vector<tf2::Vector3> points;
    sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
    for (; x_it != x_it.end(); ++x_it, ++y_it)
        if (std::isfinite(*x_it) && std::isfinite(*y_it)) points.emplace_back(*x_it, *y_it, 0.0);
    return points;
}

}  // namespace aiformula
