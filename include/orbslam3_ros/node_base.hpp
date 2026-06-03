/**
 * @file node_base.hpp
 * @brief Common base interface for the ORB-SLAM3 ROS localization nodes.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_NODE_BASE_HPP
#define ORB_SLAM3_BRIDGE_NODE_BASE_HPP

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "orbslam3_ros/pose_snapshot.hpp"

namespace tf2_ros {
    class Buffer;
    class TransformListener;
    class StaticTransformBroadcaster;
}

namespace orbslam3_ros {

    class LocalizationNodeBase : public rclcpp::Node {
    public:
        // Construct a localization node with the given ROS node name.
        explicit LocalizationNodeBase(const std::string& node_name);

    protected:
        // Declare the parameters shared by both RGB-D node variants.
        void DeclareCommonParameters();
        // Set up TF listeners and broadcasters used by the localization output.
        void InitializeTfInfrastructure();

        // Return true when the given file path exists and points to a regular file.
        [[nodiscard]] bool ValidateFilePath(const std::string& path, const char* param_name) const;
        // Convert a ROS time stamp to seconds.
        [[nodiscard]] static double ToSeconds(const builtin_interfaces::msg::Time& stamp) noexcept;
        // Convert a ROS time stamp to nanoseconds.
        [[nodiscard]] static std::int64_t ToNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept;
        // Convert a nanosecond count to milliseconds.
        [[nodiscard]] static double ToMilliseconds(std::int64_t nanoseconds) noexcept;
        // Format a ROS time stamp as a decimal string.
        [[nodiscard]] static std::string FormatStamp(const builtin_interfaces::msg::Time& stamp);
        // Format a nanosecond count as a decimal time stamp string.
        [[nodiscard]] static std::string FormatStampNs(std::int64_t nanoseconds);
        // Format a floating-point value with fixed precision.
        [[nodiscard]] static std::string FormatDouble(double value);
        // Return the REP-103 optical-frame rotation.
        [[nodiscard]] static geometry_msgs::msg::Quaternion MakeOpticalQuaternion() noexcept;
        // Return the identity rotation.
        [[nodiscard]] static geometry_msgs::msg::Quaternion MakeIdentityQuaternion() noexcept;

        // Build a pose snapshot from a tracked SLAM pose and map points.
        [[nodiscard]] PoseSnapshot BuildPoseSnapshot(
            const Sophus::SE3f& tracked_pose,
            const builtin_interfaces::msg::Time& stamp,
            TrackingState tracking_state,
            std::vector<MapPointSnapshot> map_points
        ) const;

        // Convert a Sophus pose to a ROS pose message.
        [[nodiscard]] geometry_msgs::msg::Pose ToPoseMessage(const Sophus::SE3f& transform) const;
        // Convert an Eigen transform to a ROS pose message.
        [[nodiscard]] geometry_msgs::msg::Pose ToPoseMessage(const Eigen::Isometry3f& transform) const;
        // Convert a ROS pose message to an Eigen transform.
        [[nodiscard]] Eigen::Isometry3f ToIsometry(const geometry_msgs::msg::Pose& pose) const;
        // Convert a ROS transform message to an Eigen transform.
        [[nodiscard]] Eigen::Isometry3f ToIsometry(const geometry_msgs::msg::Transform& transform) const;
        // Look up the base-to-camera extrinsic from TF.
        [[nodiscard]] std::optional<Eigen::Isometry3f> LookupBaseToCameraTransform() const;

        // Publish the dynamic map-to-base transform.
        void PublishDynamicTransform(const PoseSnapshot& snapshot);
        // Publish any configured static TF fallback frames.
        void PublishStaticTransforms();

        std::string vocab_file_;
        std::string settings_file_;
        std::string pose_topic_{"/orbslam3/pose"};
        std::string odom_topic_{"/orbslam3/odom"};
        std::string path_topic_{"/orbslam3/path"};
        std::string tracking_state_topic_{"/orbslam3/tracking_state"};
        std::string map_points_topic_{"/orbslam3/map_points"};
        std::string map_frame_{"map"};
        std::string world_frame_{"map"};
        std::string base_frame_{"base_link"};
        std::string camera_frame_{"camera_link"};
        std::string camera_optical_frame_{"camera_optical_frame"};
        std::size_t path_history_size_{200};
        double path_update_distance_{0.05};
        double path_update_interval_sec_{0.1};
        bool publish_tf_{true};
        bool publish_static_camera_tf_{false};
        bool publish_static_optical_tf_{false};
        bool invert_pose_{false};
        bool use_viewer_{true};

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

        mutable std::atomic<bool> camera_transform_warning_emitted_{false};
    };

}  // namespace orbslam3_ros

#endif  // ORB_SLAM3_BRIDGE_NODE_BASE_HPP
