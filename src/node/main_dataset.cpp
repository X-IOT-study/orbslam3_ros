/**
 * @file main_dataset.cpp
 * @brief Main entry point for the dataset-driven RGB-D ROS node.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include <memory>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "orbslam3_ros/rgbd_dataset_node.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<orbslam3_ros::RGBDDatasetNode>();
        node->Start();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("orbslam3_ros"), "Failed to start dataset node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
