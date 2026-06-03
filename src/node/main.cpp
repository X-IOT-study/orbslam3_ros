/**
 * @file main.cpp
 * @brief Main entry point for the realtime RGB-D ROS node.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include <memory>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "orbslam3_ros/rgbd_node.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<orbslam3_ros::RGBDNode>();
        node->Start();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("orbslam3_ros"), "Failed to start realtime node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
