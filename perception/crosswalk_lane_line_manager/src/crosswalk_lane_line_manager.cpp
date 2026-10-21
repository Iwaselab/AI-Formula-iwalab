#include "crosswalk_lane_line_manager/crosswalk_lane_line_manager.hpp"

#include <cmath>
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
    crosswalk_exit_missing_frames_ = getRosParameter<int>(this, "crosswalk_exit_missing_frames");
    temporal_filter_frames_ = getRosParameter<int>(this, "temporal_filter_frames");
    if (xmax_ <= xmin_ || spacing_ <= 0.0) {
        throw std::invalid_argument("lane line ROI and spacing must define a positive range");
    }
    if (crosswalk_bbox_width_threshold_ < 0.0) {
        throw std::invalid_argument("crosswalk_bbox_width_threshold must be non-negative");
    }
    if (compensation_fit_xmax_ <= 0.0) {
        throw std::invalid_argument("compensation_fit_xmax must be positive");
    }
    if (crosswalk_exit_missing_frames_ <= 0) {
        throw std::invalid_argument("crosswalk_exit_missing_frames must be positive");
    }
    if (temporal_filter_frames_ <= 0) {
        throw std::invalid_argument("temporal_filter_frames must be positive");
    }
    initConnections();
}

void CrosswalkLaneLineManager::initConnections() {
    const auto subscription_qos = rclcpp::SensorDataQoS();
    const auto publisher_qos = rclcpp::QoS(10);
    lane_line_subs_[LEFT] = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_lane_lines/left", subscription_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { laneLineCallback(msg, LEFT); });
    lane_line_subs_[RIGHT] = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_lane_lines/right", subscription_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { laneLineCallback(msg, RIGHT); });
    lane_line_subs_[CENTER] = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_lane_lines/center", subscription_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { laneLineCallback(msg, CENTER); });
    crosswalk_sub_ = create_subscription<aiformula_interfaces::msg::RectMultiArray>(
        "crosswalk_detection", subscription_qos,
        std::bind(&CrosswalkLaneLineManager::crosswalkCallback, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", subscription_qos,
        std::bind(&CrosswalkLaneLineManager::odometryCallback, this, std::placeholders::_1));
    lane_line_pubs_[LEFT] = create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/left", publisher_qos);
    lane_line_pubs_[RIGHT] = create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/right", publisher_qos);
    lane_line_pubs_[CENTER] = create_publisher<sensor_msgs::msg::PointCloud2>("lane_lines/center", publisher_qos);
}

void CrosswalkLaneLineManager::laneLineCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t side) {
    latest_camera_lines_[side] = msg;
    if (collecting_lines_ && side < recent_lines_.size()) {
        collectLineFrame(side);
    } else if (crosswalk_active_ && side < saved_lines_.size() && !saved_lines_[side]) {
        saved_lines_[side] = captureLine(*msg);
    }
    if (!crosswalk_active_) {
        lane_line_pubs_[side]->publish(*msg);
        return;
    }
    publishLines();
}

void CrosswalkLaneLineManager::crosswalkCallback(
    const aiformula_interfaces::msg::RectMultiArray::SharedPtr msg) {
    const auto width = crosswalkWidth(*msg);
    const bool detected = width.has_value();
    const bool start_compensation =
        detected && *width > crosswalk_bbox_width_threshold_;
    if (detected) {
        crosswalk_missing_frames_ = 0;
    } else if (crosswalk_active_) {
        ++crosswalk_missing_frames_;
    }
    if (start_compensation && !crosswalk_active_ && !collecting_lines_) {
        startLineCollection();
        RCLCPP_INFO(get_logger(),
                    "Crosswalk bbox width exceeded %.1f px; collecting %d lane-line frames",
                    crosswalk_bbox_width_threshold_);
    } else if (crosswalk_active_ &&
               crosswalk_missing_frames_ >= crosswalk_exit_missing_frames_) {
        finishCompensation();
        RCLCPP_INFO(get_logger(),
                    "Crosswalk missing for %d frames; switching back to camera lane lines",
                    crosswalk_exit_missing_frames_);
    }
    publishLines();
}

void CrosswalkLaneLineManager::startLineCollection() {
    collecting_lines_ = true;
    crosswalk_missing_frames_ = 0;
    recent_lines_ = {};
    saved_lines_ = {};
    collecting_line_received_ = {};
}

void CrosswalkLaneLineManager::collectLineFrame(size_t side) {
    collecting_line_received_[side] = true;
    if (!collecting_line_received_[LEFT] || !collecting_line_received_[RIGHT]) return;

    for (size_t line_side = LEFT; line_side <= RIGHT; ++line_side) {
        const auto line = captureLine(*latest_camera_lines_[line_side]);
        if (line) recent_lines_[line_side].push_back(*line);
    }
    collecting_line_received_ = {};

    if (recent_lines_[LEFT].size() < static_cast<size_t>(temporal_filter_frames_) ||
        recent_lines_[RIGHT].size() < static_cast<size_t>(temporal_filter_frames_)) {
        return;
    }
    for (size_t line_side = LEFT; line_side <= RIGHT; ++line_side)
        saved_lines_[line_side] = averageRecentLines(recent_lines_[line_side]);
    collecting_lines_ = false;
    crosswalk_active_ = true;
    RCLCPP_INFO(get_logger(), "Collected %d paired lane-line frames; switching to preserved lane lines",
                temporal_filter_frames_);
}

void CrosswalkLaneLineManager::finishCompensation() {
    crosswalk_active_ = false;
    collecting_lines_ = false;
    crosswalk_missing_frames_ = 0;
    saved_lines_ = {};
    recent_lines_ = {};
    collecting_line_received_ = {};
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
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_xx = 0.0;
        double sum_xy = 0.0;
        for (const auto& point : points) {
            sum_x += point.x();
            sum_y += point.y();
            sum_xx += point.x() * point.x();
            sum_xy += point.x() * point.y();
        }

        const double denominator = points.size() * sum_xx - sum_x * sum_x;
        const double mean_x = sum_x / static_cast<double>(points.size());
        const double mean_y = sum_y / static_cast<double>(points.size());
        const double slope = std::abs(denominator) > 1e-9
                                 ? (points.size() * sum_xy - sum_x * sum_y) / denominator
                                 : 0.0;
        const double intercept = mean_y - slope * mean_x;

        tf2::Transform odom_T_camera_line;
        tf2::fromMsg(transform.transform, odom_T_camera_line);
        return Line{
            odom_T_camera_line * tf2::Vector3(0.0, intercept, 0.0),
            odom_T_camera_line * tf2::Vector3(compensation_fit_xmax_,
                                               slope * compensation_fit_xmax_ + intercept, 0.0)};
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot preserve lane line: %s", e.what());
        return std::nullopt;
    }
}

std::optional<CrosswalkLaneLineManager::Line> CrosswalkLaneLineManager::averageRecentLines(
    const std::deque<Line>& lines) const {
    if (lines.empty()) return std::nullopt;

    tf2::Vector3 first(0.0, 0.0, 0.0);
    tf2::Vector3 last(0.0, 0.0, 0.0);
    for (const auto& line : lines) {
        first += line.first;
        last += line.last;
    }
    const double count = static_cast<double>(lines.size());
    return Line{first / count, last / count};
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
    const double center_x = 0.5 * (line.first.x() + line.last.x());
    const double center_y = 0.5 * (line.first.y() + line.last.y());

    for (double x = xmin_; x <= xmax_ + 1e-9; x += spacing_) {
        // A line with almost no x component cannot be represented as y=f(x).
        // Use its mean y rather than inventing a slope from a substituted direction.
        const double y = std::abs(dx) > 1e-6
                             ? center_y + dy * (x - center_x) / dx
                             : center_y;
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

sensor_msgs::msg::PointCloud2 CrosswalkLaneLineManager::createCenterLineMessage(
    const Line& left, const Line& right, const rclcpp::Time& stamp) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const auto lineYAtX = [](const Line& line, double x) {
        const double dx = line.last.x() - line.first.x();
        if (std::abs(dx) <= 1e-6) return 0.5 * (line.first.y() + line.last.y());
        return line.first.y() + (line.last.y() - line.first.y()) *
                                    (x - line.first.x()) / dx;
    };

    for (double x = xmin_; x <= xmax_ + 1e-9; x += spacing_) {
        auto& point = cloud.points.emplace_back();
        point.x = static_cast<float>(x);
        point.y = static_cast<float>((lineYAtX(left, x) + lineYAtX(right, x)) * 0.5);
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
    std::array<std::optional<Line>, 2> current_lines;
    for (size_t side = 0; side < current_lines.size(); ++side) {
        if (saved_lines_[side]) current_lines[side] = transformLineToVehicle(*saved_lines_[side], stamp);
        if (current_lines[side]) {
            lane_line_pubs_[side]->publish(createLineMessage(*current_lines[side], stamp));
        } else if (latest_camera_lines_[side]) {
            lane_line_pubs_[side]->publish(*latest_camera_lines_[side]);
        }
    }

    if (current_lines[LEFT] && current_lines[RIGHT]) {
        lane_line_pubs_[CENTER]->publish(
            createCenterLineMessage(*current_lines[LEFT], *current_lines[RIGHT], stamp));
    } else if (latest_camera_lines_[CENTER]) {
        lane_line_pubs_[CENTER]->publish(*latest_camera_lines_[CENTER]);
    }
}

std::optional<float> CrosswalkLaneLineManager::crosswalkWidth(
    const aiformula_interfaces::msg::RectMultiArray& msg) const {
    std::optional<float> maximum_width;
    for (const auto& rect : msg.rects) {
        if (rect.class_id != 0 ||
            (maximum_width && rect.width <= *maximum_width)) {
            continue;
        }
        maximum_width = rect.width;
    }
    return maximum_width;
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
