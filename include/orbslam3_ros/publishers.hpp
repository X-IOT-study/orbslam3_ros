/**
 * @file publishers.hpp
 * @brief Common ROS publisher helpers for ORB-SLAM3 localization output.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_PUBLISHERS_HPP
#define ORB_SLAM3_BRIDGE_PUBLISHERS_HPP

#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "orbslam3_ros/pose_snapshot.hpp"

namespace orbslam3_ros {

    struct LocalizationPublisherConfig {
        std::string pose_topic{"/orbslam3/pose"};
        std::string odom_topic{"/orbslam3/odom"};
        std::string path_topic{"/orbslam3/path"};
        std::string tracking_state_topic{"/orbslam3/tracking_state"};
        std::string map_points_topic{"/orbslam3/map_points"};
        std::string map_frame{"map"};
        std::string base_frame{"base_link"};
        std::size_t queue_size{10};
        std::size_t path_history_size{200};
        double path_update_distance{0.05};
        double path_update_interval_sec{0.1};
    };

    class LocalizationPublishers {
    public:
        // Construct the publisher bundle for one localization node.
        LocalizationPublishers(
            const rclcpp::Node::SharedPtr& node,
            const LocalizationPublisherConfig& config
        );

        // Publish the current tracking state.
        void PublishTrackingState(TrackingState tracking_state);
        // Publish pose, odometry, and related outputs for a single snapshot.
        void PublishPoseBundle(const PoseSnapshot& pose_snapshot, const PoseSnapshot& odom_snapshot);
        // Append the current pose to the published path if needed.
        void PublishPath(const PoseSnapshot& snapshot);
        // Publish the current map-point cloud snapshot.
        void PublishMapPoints(const PoseSnapshot& snapshot);
        // Clear the stored path history.
        void ResetPath();

    private:
        [[nodiscard]] geometry_msgs::msg::PoseStamped MakePoseStamped(const PoseSnapshot& snapshot) const;
        [[nodiscard]] nav_msgs::msg::Odometry MakeOdometry(const PoseSnapshot& snapshot) const;
        void UpdateMapPointCloud(const PoseSnapshot& snapshot);
        void AppendPathPose(const geometry_msgs::msg::PoseStamped& pose_msg);
        [[nodiscard]] bool ShouldAppendPathPose(const geometry_msgs::msg::PoseStamped& pose_msg) const;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr tracking_state_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;

        LocalizationPublisherConfig config_;
        nav_msgs::msg::Path path_message_;
        sensor_msgs::msg::PointCloud2 map_points_message_;
        std::deque<geometry_msgs::msg::PoseStamped> path_poses_;
        std::mutex mutex_;
    };

}  // namespace orbslam3_ros

#endif  // ORB_SLAM3_BRIDGE_PUBLISHERS_HPP
