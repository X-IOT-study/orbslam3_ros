/**
 * @file publishers.cpp
 * @brief ROS publisher helpers for ORB-SLAM3 localization output.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include "orbslam3_ros/publishers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace orbslam3_ros {

    LocalizationPublishers::LocalizationPublishers(
        const rclcpp::Node::SharedPtr& node,
        const LocalizationPublisherConfig& config
    )
    : config_(config) {
        const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, config_.queue_size)));
        const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, config_.queue_size)));
        const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto tracking_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto map_points_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(config_.pose_topic, pose_qos);
        odometry_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(config_.odom_topic, odom_qos);
        path_publisher_ = node->create_publisher<nav_msgs::msg::Path>(config_.path_topic, path_qos);
        tracking_state_publisher_ = node->create_publisher<std_msgs::msg::UInt8>(
            config_.tracking_state_topic,
            tracking_qos
        );
        map_points_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.map_points_topic,
            map_points_qos
        );

        path_message_.header.frame_id = config_.map_frame;
        path_message_.poses.clear();
        map_points_message_.header.frame_id = config_.map_frame;
        map_points_message_.height = 1;
        map_points_message_.is_dense = true;
        map_points_message_.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(map_points_message_);
        modifier.setPointCloud2FieldsByString(1, "xyz");
    }

    void LocalizationPublishers::PublishTrackingState(TrackingState tracking_state) {
        if (!tracking_state_publisher_) {
            return;
        }

        std_msgs::msg::UInt8 message;
        message.data = static_cast<std::uint8_t>(tracking_state);
        tracking_state_publisher_->publish(message);
    }

    void LocalizationPublishers::PublishPoseBundle(
        const PoseSnapshot& pose_snapshot,
        const PoseSnapshot& odom_snapshot
    ) {
        if (!pose_snapshot.pose_valid || !odom_snapshot.pose_valid) {
            return;
        }

        const auto pose_msg = MakePoseStamped(pose_snapshot);
        const auto odom_msg = MakeOdometry(odom_snapshot);

        if (pose_publisher_) {
            pose_publisher_->publish(pose_msg);
        }

        if (odometry_publisher_) {
            odometry_publisher_->publish(odom_msg);
        }
    }

    void LocalizationPublishers::PublishPath(const PoseSnapshot& snapshot) {
        if (!snapshot.pose_valid) {
            return;
        }

        const auto pose_msg = MakePoseStamped(snapshot);
        std::lock_guard<std::mutex> lock(mutex_);
        AppendPathPose(pose_msg);
    }

    void LocalizationPublishers::PublishMapPoints(const PoseSnapshot& snapshot) {
        if (!map_points_publisher_ || !snapshot.pose_valid || snapshot.map_points.empty()) {
            return;
        }

        UpdateMapPointCloud(snapshot);
        map_points_publisher_->publish(map_points_message_);
    }

    void LocalizationPublishers::ResetPath() {
        std::lock_guard<std::mutex> lock(mutex_);
        path_poses_.clear();
        path_message_.poses.clear();
    }

    geometry_msgs::msg::PoseStamped LocalizationPublishers::MakePoseStamped(const PoseSnapshot& snapshot) const {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = snapshot.stamp;
        pose_msg.header.frame_id = config_.map_frame;
        pose_msg.pose = snapshot.pose;
        return pose_msg;
    }

    nav_msgs::msg::Odometry LocalizationPublishers::MakeOdometry(const PoseSnapshot& snapshot) const {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = snapshot.stamp;
        odom_msg.header.frame_id = config_.map_frame;
        odom_msg.child_frame_id = config_.base_frame;
        odom_msg.pose.pose = snapshot.pose;
        return odom_msg;
    }

    void LocalizationPublishers::UpdateMapPointCloud(const PoseSnapshot& snapshot) {
        map_points_message_.header.stamp = snapshot.stamp;
        map_points_message_.header.frame_id = config_.map_frame;
        map_points_message_.data.reserve(snapshot.map_points.size() * map_points_message_.point_step);

        sensor_msgs::PointCloud2Modifier modifier(map_points_message_);
        modifier.resize(snapshot.map_points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(map_points_message_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(map_points_message_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(map_points_message_, "z");

        for (const auto& point : snapshot.map_points) {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    bool LocalizationPublishers::ShouldAppendPathPose(const geometry_msgs::msg::PoseStamped& pose_msg) const {
        if (path_poses_.empty()) {
            return true;
        }

        const auto& last_pose = path_poses_.back();
        const double dx = pose_msg.pose.position.x - last_pose.pose.position.x;
        const double dy = pose_msg.pose.position.y - last_pose.pose.position.y;
        const double dz = pose_msg.pose.position.z - last_pose.pose.position.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double elapsed = static_cast<double>(
            static_cast<std::int64_t>(pose_msg.header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(pose_msg.header.stamp.nanosec) -
            static_cast<std::int64_t>(last_pose.header.stamp.sec) * 1000000000LL -
            static_cast<std::int64_t>(last_pose.header.stamp.nanosec)
        ) * 1e-9;

        return distance >= config_.path_update_distance || elapsed >= config_.path_update_interval_sec;
    }

    void LocalizationPublishers::AppendPathPose(const geometry_msgs::msg::PoseStamped& pose_msg) {
        if (!ShouldAppendPathPose(pose_msg)) {
            return;
        }

        path_poses_.push_back(pose_msg);
        while (path_poses_.size() > config_.path_history_size) {
            path_poses_.pop_front();
        }

        path_message_.header.stamp = pose_msg.header.stamp;
        path_message_.header.frame_id = config_.map_frame;
        path_message_.poses.assign(path_poses_.begin(), path_poses_.end());

        if (path_publisher_) {
            path_publisher_->publish(path_message_);
        }
    }

}  // namespace orbslam3_ros
