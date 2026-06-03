/**
 * @file tum_dataset_loader.cpp
 * @brief Implementation of the TUM dataset loader for ORB-SLAM3.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include "orbslam3_ros/tum_dataset_loader.hpp"

#include <fstream>
#include <sstream>
#include <utility>

#include <opencv2/imgcodecs.hpp>

namespace {
    bool IsIgnorableLine(const std::string& line) {
        const auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return true;
        }

        return line[first] == '#';
    }

    std::filesystem::path ResolvePath(const std::filesystem::path& base_path, const std::string& file_path) {
        const std::filesystem::path path(file_path);
        return path.is_absolute() ? path : (base_path / path);
    }
}

namespace orbslam3_ros {
    bool TUMDatasetLoader::Load(const std::string& association_file) {
        const std::filesystem::path association_path(association_file);
        std::ifstream input(association_file);
        if (!input.is_open()) {
            return false;
        }

        std::vector<Frame> frames;
        const std::filesystem::path base_path = association_path.parent_path();

        std::string line;
        while (std::getline(input, line)) {
            if (IsIgnorableLine(line)) {
                continue;
            }

            std::istringstream stream(line);
            double rgb_timestamp = 0.0;
            double depth_timestamp = 0.0;
            std::string rgb_file;
            std::string depth_file;

            if (!(stream >> rgb_timestamp >> rgb_file >> depth_timestamp >> depth_file)) {
                continue;
            }

            frames.push_back(Frame{
                ResolvePath(base_path, rgb_file),
                ResolvePath(base_path, depth_file),
                rgb_timestamp
            });
        }

        if (frames.empty()) {
            return false;
        }

        frames_ = std::move(frames);
        current_index_ = 0;

        return true;
    }

    void TUMDatasetLoader::Reset() noexcept {
        current_index_ = 0;
    }

    std::size_t TUMDatasetLoader::Size() const {
        return frames_.size();
    }

    bool TUMDatasetLoader::HasNext() const {
        return current_index_ < frames_.size();
    }

    bool TUMDatasetLoader::Next(Frame& frame) {
        if (!HasNext()) {
            return false;
        }

        frame = frames_[current_index_++];
        return true;
    }

    bool TUMDatasetLoader::Next(cv::Mat& rgb, cv::Mat& depth, double& timestamp) {
        Frame frame;
        if (!Next(frame)) {
            return false;
        }

        rgb = cv::imread(frame.rgb_path.string(), cv::IMREAD_COLOR);
        depth = cv::imread(frame.depth_path.string(), cv::IMREAD_UNCHANGED);
        timestamp = frame.timestamp;

        return !rgb.empty() && !depth.empty();
    }
}
