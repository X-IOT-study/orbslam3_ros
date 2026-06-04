/**
 * @file main_multimode.cpp
 * @brief Main entry point for the multi-mode ORB-SLAM3 ROS node.
 */

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "orbslam3_ros/multimode_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<orbslam3_ros::MultiModeNode>();
        node->Start();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("orbslam3_ros"), "Failed to start multi-mode node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
