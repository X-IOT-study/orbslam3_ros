/**
 * @file node_base.cpp
 * @brief Common base implementation for the ORB-SLAM3 ROS localization nodes.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include "orbslam3_ros/node_base.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace orbslam3_ros {

    LocalizationNodeBase::LocalizationNodeBase(const std::string& node_name)
    : rclcpp::Node(node_name) {}

    void LocalizationNodeBase::DeclareCommonParameters() {
        auto make_integer_descriptor = [](const std::string& description, int from_value, int to_value) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = description;
            descriptor.integer_range.resize(1);
            descriptor.integer_range[0].from_value = from_value;
            descriptor.integer_range[0].to_value = to_value;
            descriptor.integer_range[0].step = 1;
            return descriptor;
        };

        auto make_double_descriptor = [](const std::string& description, double from_value, double to_value) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = description;
            descriptor.floating_point_range.resize(1);
            descriptor.floating_point_range[0].from_value = from_value;
            descriptor.floating_point_range[0].to_value = to_value;
            descriptor.floating_point_range[0].step = 0.0;
            return descriptor;
        };

        vocab_file_ = this->declare_parameter<std::string>("vocab_file", "");
        settings_file_ = this->declare_parameter<std::string>("settings_file", "");
        pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/orbslam3/pose");
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/orbslam3/odom");
        path_topic_ = this->declare_parameter<std::string>("path_topic", "/orbslam3/path");
        tracking_state_topic_ = this->declare_parameter<std::string>("tracking_state_topic", "/orbslam3/tracking_state");
        map_points_topic_ = this->declare_parameter<std::string>("map_points_topic", "/orbslam3/map_points");
        world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
        map_frame_ = this->declare_parameter<std::string>("map_frame", world_frame_);
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
        camera_optical_frame_ = this->declare_parameter<std::string>("camera_optical_frame", "camera_optical_frame");
        const int path_history_size = this->declare_parameter<int>(
            "path_history_size",
            200,
            make_integer_descriptor("Maximum number of poses stored in the output path.", 1, 10000)
        );
        path_history_size_ = static_cast<std::size_t>(std::max(1, path_history_size));
        path_update_distance_ = this->declare_parameter<double>(
            "path_update_distance",
            0.05,
            make_double_descriptor("Minimum translation required before adding a path sample.", 0.0, 10.0)
        );
        path_update_interval_sec_ = this->declare_parameter<double>(
            "path_update_interval",
            0.1,
            make_double_descriptor("Maximum time between path samples in seconds.", 0.0, 10.0)
        );
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        publish_static_camera_tf_ = this->declare_parameter<bool>("publish_static_camera_tf", false);
        publish_static_optical_tf_ = this->declare_parameter<bool>("publish_static_optical_tf", false);
        invert_pose_ = this->declare_parameter<bool>("invert_pose", false);
        use_viewer_ = this->declare_parameter<bool>("use_viewer", true);

        if (map_frame_.empty() || (map_frame_ == "map" && world_frame_ != "map")) {
            map_frame_ = world_frame_;
        }
    }

    void LocalizationNodeBase::InitializeTfInfrastructure() {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), false);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
        }

        if (publish_static_camera_tf_ || publish_static_optical_tf_) {
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
        }
    }

    bool LocalizationNodeBase::ValidateFilePath(const std::string& path, const char* param_name) const {
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is empty.", param_name);
            return false;
        }

        const std::filesystem::path file_path(path);
        if (!std::filesystem::exists(file_path) || !std::filesystem::is_regular_file(file_path)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter '%s' points to a missing or invalid file: %s",
                param_name,
                path.c_str()
            );
            return false;
        }

        return true;
    }

    const char* LocalizationNodeBase::TrackingStateToString(TrackingState tracking_state) noexcept {
        switch (tracking_state) {
            case TrackingState::Lost:
                return "Lost";
            case TrackingState::Initializing:
                return "Initializing";
            case TrackingState::Tracking:
                return "Tracking";
        }

        return "Unknown";
    }

    double LocalizationNodeBase::ToSeconds(const builtin_interfaces::msg::Time& stamp) noexcept {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    std::int64_t LocalizationNodeBase::ToNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
        return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
    }

    double LocalizationNodeBase::ToMilliseconds(std::int64_t nanoseconds) noexcept {
        return static_cast<double>(nanoseconds) * 1e-6;
    }

    std::string LocalizationNodeBase::FormatStamp(const builtin_interfaces::msg::Time& stamp) {
        std::ostringstream stream;
        stream << stamp.sec << '.' << std::setw(9) << std::setfill('0') << stamp.nanosec;
        return stream.str();
    }

    std::string LocalizationNodeBase::FormatStampNs(std::int64_t nanoseconds) {
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        return FormatStamp(stamp);
    }

    std::string LocalizationNodeBase::FormatDouble(double value) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(9) << value;
        return stream.str();
    }

    geometry_msgs::msg::Quaternion LocalizationNodeBase::MakeOpticalQuaternion() noexcept {
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = -0.5;
        quaternion.y = 0.5;
        quaternion.z = -0.5;
        quaternion.w = 0.5;
        return quaternion;
    }

    geometry_msgs::msg::Quaternion LocalizationNodeBase::MakeIdentityQuaternion() noexcept {
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.w = 1.0;
        return quaternion;
    }

    PoseSnapshot LocalizationNodeBase::BuildPoseSnapshot(
        const Sophus::SE3f& tracked_pose,
        const builtin_interfaces::msg::Time& stamp,
        TrackingState tracking_state,
        std::vector<MapPointSnapshot> map_points
    ) const {
        PoseSnapshot snapshot;
        snapshot.stamp = stamp;
        snapshot.tracking_state = tracking_state;
        snapshot.map_points = std::move(map_points);
        snapshot.pose_valid = true;

        const Sophus::SE3f camera_pose_world = tracked_pose.inverse();
        const auto base_to_camera = LookupBaseToCameraTransform();
        const Eigen::Isometry3f camera_to_base = base_to_camera
            ? base_to_camera->inverse()
            : Eigen::Isometry3f::Identity();

        const Eigen::Isometry3f world_camera = ToIsometry(ToPoseMessage(camera_pose_world));
        const Eigen::Isometry3f world_base = world_camera * camera_to_base;
        snapshot.pose = ToPoseMessage(world_base);
        return snapshot;
    }

    geometry_msgs::msg::Pose LocalizationNodeBase::ToPoseMessage(const Sophus::SE3f& transform) const {
        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        pose.linear() = transform.unit_quaternion().toRotationMatrix();
        pose.translation() = transform.translation();
        return ToPoseMessage(pose);
    }

    geometry_msgs::msg::Pose LocalizationNodeBase::ToPoseMessage(const Eigen::Isometry3f& transform) const {
        geometry_msgs::msg::Pose pose;
        pose.position.x = static_cast<double>(transform.translation().x());
        pose.position.y = static_cast<double>(transform.translation().y());
        pose.position.z = static_cast<double>(transform.translation().z());

        const Eigen::Quaternionf quaternion(transform.rotation());
        pose.orientation.x = static_cast<double>(quaternion.x());
        pose.orientation.y = static_cast<double>(quaternion.y());
        pose.orientation.z = static_cast<double>(quaternion.z());
        pose.orientation.w = static_cast<double>(quaternion.w());
        return pose;
    }

    Eigen::Isometry3f LocalizationNodeBase::ToIsometry(const geometry_msgs::msg::Pose& pose) const {
        Eigen::Quaternionf quaternion(
            static_cast<float>(pose.orientation.w),
            static_cast<float>(pose.orientation.x),
            static_cast<float>(pose.orientation.y),
            static_cast<float>(pose.orientation.z)
        );
        quaternion.normalize();

        Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
        transform.linear() = quaternion.toRotationMatrix();
        transform.translation() = Eigen::Vector3f(
            static_cast<float>(pose.position.x),
            static_cast<float>(pose.position.y),
            static_cast<float>(pose.position.z)
        );
        return transform;
    }

    Eigen::Isometry3f LocalizationNodeBase::ToIsometry(const geometry_msgs::msg::Transform& transform) const {
        Eigen::Quaternionf quaternion(
            static_cast<float>(transform.rotation.w),
            static_cast<float>(transform.rotation.x),
            static_cast<float>(transform.rotation.y),
            static_cast<float>(transform.rotation.z)
        );
        quaternion.normalize();

        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        pose.linear() = quaternion.toRotationMatrix();
        pose.translation() = Eigen::Vector3f(
            static_cast<float>(transform.translation.x),
            static_cast<float>(transform.translation.y),
            static_cast<float>(transform.translation.z)
        );
        return pose;
    }

    std::optional<Eigen::Isometry3f> LocalizationNodeBase::LookupBaseToCameraTransform() const {
        if (!tf_buffer_) {
            return std::nullopt;
        }

        try {
            const auto transform = tf_buffer_->lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
            return ToIsometry(transform.transform);
        } catch (const std::exception&) {
            if (!camera_transform_warning_emitted_.exchange(true)) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "No TF lookup from %s to %s; falling back to identity for visualization only. Publish a real extrinsic via URDF or robot_state_publisher.",
                    base_frame_.c_str(),
                    camera_frame_.c_str()
                );
            }
            return std::nullopt;
        }
    }

    void LocalizationNodeBase::PublishDynamicTransform(const PoseSnapshot& snapshot) {
        if (!publish_tf_ || !tf_broadcaster_ || !snapshot.pose_valid) {
            return;
        }

        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = snapshot.stamp;
        transform_msg.header.frame_id = map_frame_;
        transform_msg.child_frame_id = base_frame_;
        transform_msg.transform.translation.x = snapshot.pose.position.x;
        transform_msg.transform.translation.y = snapshot.pose.position.y;
        transform_msg.transform.translation.z = snapshot.pose.position.z;
        transform_msg.transform.rotation = snapshot.pose.orientation;
        tf_broadcaster_->sendTransform(transform_msg);
    }

    void LocalizationNodeBase::PublishStaticTransforms() {
        if (!static_tf_broadcaster_) {
            return;
        }

        if (publish_static_camera_tf_) {
            if (base_frame_ != camera_frame_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "publish_static_camera_tf publishes an identity fallback for %s -> %s; use URDF / robot_state_publisher for a real extrinsic.",
                    base_frame_.c_str(),
                    camera_frame_.c_str()
                );
            }
            geometry_msgs::msg::TransformStamped camera_transform;
            camera_transform.header.stamp = this->get_clock()->now();
            camera_transform.header.frame_id = base_frame_;
            camera_transform.child_frame_id = camera_frame_;
            camera_transform.transform.rotation = MakeIdentityQuaternion();
            static_tf_broadcaster_->sendTransform(camera_transform);
        }

        if (publish_static_optical_tf_) {
            geometry_msgs::msg::TransformStamped optical_transform;
            optical_transform.header.stamp = this->get_clock()->now();
            optical_transform.header.frame_id = camera_frame_;
            optical_transform.child_frame_id = camera_optical_frame_;
            optical_transform.transform.rotation = MakeOpticalQuaternion();
            static_tf_broadcaster_->sendTransform(optical_transform);
        }
    }

}  // namespace orbslam3_ros
