/**
 * @file pose_snapshot.hpp
 * @brief Shared pose and tracking snapshot data types.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_POSE_SNAPSHOT_HPP
#define ORB_SLAM3_BRIDGE_POSE_SNAPSHOT_HPP

#include <cstdint>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace orbslam3_ros {

    enum class TrackingState : std::uint8_t {
        Lost = 0,
        Initializing = 1,
        Tracking = 2,
    };

    struct MapPointSnapshot {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
    };

    struct PoseSnapshot {
        builtin_interfaces::msg::Time stamp;
        geometry_msgs::msg::Pose pose;
        TrackingState tracking_state{TrackingState::Lost};
        std::vector<MapPointSnapshot> map_points;
        bool pose_valid{false};
    };

}  // namespace orbslam3_ros

#endif  // ORB_SLAM3_BRIDGE_POSE_SNAPSHOT_HPP
