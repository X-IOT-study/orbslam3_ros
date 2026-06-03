/**
 * @file tum_dataset_loader.hpp
 * @brief TUM dataset loader interface.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_TUM_DATASET_LOADER_HPP
#define ORB_SLAM3_BRIDGE_TUM_DATASET_LOADER_HPP

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace orbslam3_ros {
    class TUMDatasetLoader {
    public:
        struct Frame {
            std::filesystem::path rgb_path;
            std::filesystem::path depth_path;
            double timestamp{0.0};
        };

        TUMDatasetLoader() = default;

        // Load the TUM association file from the specified path.
        bool Load(const std::string& association_file);

        // Reset the iteration state back to the first frame.
        void Reset() noexcept;

        // Return the number of parsed frames.
        [[nodiscard]] std::size_t Size() const;
        // Return true when more frames are available.
        [[nodiscard]] bool HasNext() const;

        // Load the next frame metadata.
        [[nodiscard]] bool Next(Frame& frame);

        // Load the next RGB-D frame and return the RGB image, depth map, and timestamp.
        // Returns false if there are no more frames or the images cannot be read.
        [[nodiscard]] bool Next(cv::Mat& rgb, cv::Mat& depth, double& timestamp);

    private:
        std::vector<Frame> frames_;
        std::size_t current_index_{0};
    };
}

#endif  // ORB_SLAM3_BRIDGE_TUM_DATASET_LOADER_HPP
