/**
 * @file multimode_node.cpp
 * @brief Multi-mode ORB-SLAM3 ROS node implementation.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */
#include "orbslam3_ros/multimode_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {
    template<typename DescriptorFactory>
    auto MakeDescriptor(DescriptorFactory&& factory) {
        return factory();
    }

    cv::Mat ToMonocularImage(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg,
        const std::string& encoding
    ) {
        const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, encoding);
        const cv::Mat& source = cv_image->image;

        if (source.channels() == 1) {
            return source.clone();
        }

        cv::Mat grayscale;
        if (source.channels() == 3) {
            cv::cvtColor(source, grayscale, encoding.find("rgb") != std::string::npos &&
                encoding.find("bgr") == std::string::npos ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
        } else if (source.channels() == 4) {
            cv::cvtColor(source, grayscale, encoding.find("rgb") != std::string::npos &&
                encoding.find("bgr") == std::string::npos ? cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY);
        } else {
            grayscale = source.clone();
        }

        return grayscale;
    }
}

namespace orbslam3_ros {

    MultiModeNode::MultiModeNode() : LocalizationNodeBase("orbslam3_multimode_node") {
        DeclareParameters();
        RCLCPP_INFO(this->get_logger(), "Multi-mode ORB-SLAM3 node created");
    }

    MultiModeNode::~MultiModeNode() {
        StopWorkerThread();
        frame_queue_.reset();
        dataset_entries_.clear();
        dataset_imu_entries_.clear();
        slam_.reset();
        output_publishers_.reset();
        if (!runtime_settings_path_.empty()) {
            std::error_code ec;
            std::filesystem::remove(runtime_settings_path_, ec);
            runtime_settings_path_.clear();
        }
    }

    void MultiModeNode::Start() {
        if (started_) {
            return;
        }

        if (!ValidateFilePath(vocab_file_, "vocab_file") ||
            !ValidateFilePath(settings_file_, "settings_file")) {
            throw std::runtime_error("Invalid SLAM file parameters");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Starting ORB-SLAM3: mode=%s run_mode=%s settings_file=%s",
            mode_name_.c_str(),
            run_mode_name_.c_str(),
            settings_file_.c_str()
        );

        InitializeOutputs();
        frame_queue_ = std::make_unique<FrameQueue>(std::max<std::size_t>(1, frame_queue_size_));
        if (run_mode_ == RunMode::Realtime) {
            if (input_mode_ == SensorMode::Rgbd && runtime_calibration_from_camera_info_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Waiting for synchronized RGB-D frames and camera_info before initialization."
                );
            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for input frames before initialization.");
            }
        }

        if (run_mode_ == RunMode::Dataset) {
            if (!ValidateFilePath(association_file_, "association_file")) {
                throw std::runtime_error("Invalid dataset association file");
            }

            if (input_mode_ == SensorMode::StereoInertial &&
                !ValidateFilePath(imu_file_, "imu_file")) {
                throw std::runtime_error("Invalid IMU file");
            }

            if (!LoadDatasetFrames()) {
                throw std::runtime_error("Failed to load dataset frames");
            }

            InitializeRuntimeSlam();
            StartDatasetPlayback();
            StartWorkerThread();
        } else {
            StartRealtime();
            StartWorkerThread();
        }

        diagnostics_last_report_time_ = std::chrono::steady_clock::now();
        started_ = true;
        RCLCPP_INFO(this->get_logger(), "Multi-mode ORB-SLAM3 node started");
    }

    void MultiModeNode::DeclareParameters() {
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

        mode_name_ = this->declare_parameter<std::string>("mode", "rgbd");
        run_mode_name_ = this->declare_parameter<std::string>("run_mode", "realtime");
        dataset_root_ = this->declare_parameter<std::string>("dataset_root", "");
        input_mode_ = ParseSensorMode(mode_name_);
        run_mode_ = ParseRunMode(run_mode_name_);

        DeclareCommonParameters();

        image_topic_ = this->declare_parameter<std::string>("image_topic", "image_raw");
        left_topic_ = this->declare_parameter<std::string>("left_topic", "left/image_raw");
        right_topic_ = this->declare_parameter<std::string>("right_topic", "right/image_raw");
        rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "rgb/image_raw");
        depth_topic_ = this->declare_parameter<std::string>("depth_topic", "depth/image_raw");
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "imu");
        image_transport_ = this->declare_parameter<std::string>("image_transport", "raw");
        left_transport_ = this->declare_parameter<std::string>("left_transport", "raw");
        right_transport_ = this->declare_parameter<std::string>("right_transport", "raw");
        rgb_transport_ = this->declare_parameter<std::string>("rgb_transport", "raw");
        depth_transport_ = this->declare_parameter<std::string>("depth_transport", "raw");
        rgb_camera_info_topic_ = this->declare_parameter<std::string>("rgb_camera_info_topic", "camera_info");
        association_file_ = this->declare_parameter<std::string>("association_file", "");
        imu_file_ = this->declare_parameter<std::string>("imu_file", "");
        runtime_calibration_from_camera_info_ = this->declare_parameter<bool>(
            "runtime_calibration_from_camera_info",
            true
        );
        const int legacy_queue_size = this->declare_parameter<int>(
            "queue_size",
            10,
            make_integer_descriptor("Legacy compatibility queue size.", 1, 1000)
        );
        queue_size_ = static_cast<std::size_t>(std::max(1, legacy_queue_size));

        const int sync_queue_size = this->declare_parameter<int>(
            "sync_queue_size",
            legacy_queue_size,
            make_integer_descriptor("Approximate-time synchronizer queue size.", 1, 1000)
        );
        sync_queue_size_ = static_cast<std::size_t>(std::max(1, sync_queue_size));

        const int frame_queue_size = this->declare_parameter<int>(
            "frame_queue_size",
            legacy_queue_size,
            make_integer_descriptor("Bounded SPSC frame queue size.", 1, 1000)
        );
        frame_queue_size_ = static_cast<std::size_t>(std::max(1, frame_queue_size));

        stats_hz_ = this->declare_parameter<double>(
            "stats_hz",
            1.0,
            make_double_descriptor("Diagnostic logging frequency in Hz.", 0.1, 20.0)
        );
        map_points_rate_ = this->declare_parameter<double>(
            "map_points_rate",
            30.0,
            make_double_descriptor("Map points publishing frequency in Hz.", 0.1, 60.0)
        );
        sync_tolerance_sec_ = this->declare_parameter<double>(
            "sync_tolerance_sec",
            0.05,
            make_double_descriptor("Maximum stereo/RGB-D timestamp delta in seconds.", 0.0, 0.5)
        );
        playback_rate_ = this->declare_parameter<double>(
            "playback_rate",
            1.0,
            make_double_descriptor("Dataset playback speed multiplier.", 0.01, 100.0)
        );
        diagnostic_logging_ = this->declare_parameter<bool>("diagnostic_logging", false);
        loop_ = this->declare_parameter<bool>("loop", false);
    }

    void MultiModeNode::InitializeOutputs() {
        output_publishers_ = std::make_unique<LocalizationPublishers>(
            shared_from_this(),
            LocalizationPublisherConfig{
                pose_topic_,
                odom_topic_,
                path_topic_,
                tracking_state_topic_,
                map_points_topic_,
                map_frame_,
                base_frame_,
                std::max<std::size_t>(1, queue_size_),
                path_history_size_,
                path_update_distance_,
                path_update_interval_sec_
            }
        );

        InitializeTfInfrastructure();
        PublishStaticTransforms();

        if (stats_hz_ > 0.0) {
            const auto period = std::chrono::duration<double>(1.0 / stats_hz_);
            diagnostics_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                [this]() {
                    LogDiagnostics();
                }
            );
        }

        if (map_points_rate_ > 0.0) {
            const auto period = std::chrono::duration<double>(1.0 / map_points_rate_);
            map_points_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                [this]() {
                    PublishCachedMapPoints();
                }
            );
        }
    }

    void MultiModeNode::InitializeRuntimeSlam() {
        if (run_mode_ != RunMode::Dataset) {
            return;
        }

        slam_ = std::make_unique<SystemSlam>(
            vocab_file_,
            settings_file_,
            input_mode_,
            use_viewer_
        );
        slam_initialized_.store(true);
    }

    void MultiModeNode::StartRealtime() {
        switch (input_mode_) {
            case SensorMode::Monocular:
                SetupMonoSubscriptions();
                break;
            case SensorMode::Stereo:
                SetupStereoSubscriptions();
                break;
            case SensorMode::Rgbd:
                SetupRgbdSubscriptions();
                break;
            case SensorMode::StereoInertial:
                SetupStereoInertialSubscriptions();
                break;
        }

        SetupCommonTimers();
    }

    void MultiModeNode::StartDatasetPlayback() {
        playback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]() {
                OnPlaybackTick();
            }
        );
        SetupCommonTimers();
    }

    void MultiModeNode::SetupMonoSubscriptions() {
        image_transport::TransportHints hints(this, image_transport_, "image_transport");
        RCLCPP_INFO(
            this->get_logger(),
            "Subscribing mono image topic=%s transport=%s",
            image_topic_.c_str(),
            hints.getTransport().c_str()
        );
        mono_subscription_.subscribe(
            this,
            image_topic_,
            hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        mono_subscription_.registerCallback(&MultiModeNode::OnMonoImage, this);
    }

    void MultiModeNode::SetupStereoSubscriptions() {
        image_transport::TransportHints left_hints(this, left_transport_, "left_transport");
        image_transport::TransportHints right_hints(this, right_transport_, "right_transport");
        RCLCPP_INFO(
            this->get_logger(),
            "Subscribing stereo topics left=%s right=%s transports=%s/%s",
            left_topic_.c_str(),
            right_topic_.c_str(),
            left_hints.getTransport().c_str(),
            right_hints.getTransport().c_str()
        );

        left_subscription_.subscribe(
            this,
            left_topic_,
            left_hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        right_subscription_.subscribe(
            this,
            right_topic_,
            right_hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );

        stereo_sync_ = std::make_shared<ImagePairSynchronizer>(
            ImagePairSyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
            left_subscription_,
            right_subscription_
        );
        stereo_sync_->registerCallback(&MultiModeNode::OnStereoImages, this);
    }

    void MultiModeNode::SetupRgbdSubscriptions() {
        image_transport::TransportHints rgb_hints(this, rgb_transport_, "rgb_transport");
        image_transport::TransportHints depth_hints(this, depth_transport_, "depth_transport");
        RCLCPP_INFO(
            this->get_logger(),
            "Subscribing RGB-D topics rgb=%s depth=%s transports=%s/%s",
            rgb_topic_.c_str(),
            depth_topic_.c_str(),
            rgb_hints.getTransport().c_str(),
            depth_hints.getTransport().c_str()
        );

        rgb_subscription_.subscribe(
            this,
            rgb_topic_,
            rgb_hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        depth_subscription_.subscribe(
            this,
            depth_topic_,
            depth_hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );

        rgbd_sync_ = std::make_shared<ImagePairSynchronizer>(
            ImagePairSyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
            rgb_subscription_,
            depth_subscription_
        );
        rgbd_sync_->registerCallback(&MultiModeNode::OnRgbdImages, this);

        if (runtime_calibration_from_camera_info_) {
            camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                rgb_camera_info_topic_,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                    OnRgbdCameraInfo(msg);
                }
            );
        }
    }

    void MultiModeNode::SetupStereoInertialSubscriptions() {
        SetupStereoSubscriptions();
        RCLCPP_INFO(this->get_logger(), "Subscribing IMU topic=%s", imu_topic_.c_str());

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                OnImu(msg);
            }
        );
    }

    void MultiModeNode::SetupCommonTimers() {
        if (diagnostics_timer_) {
            diagnostics_timer_->reset();
        }
        if (map_points_timer_) {
            map_points_timer_->reset();
        }
    }

    void MultiModeNode::StartWorkerThread() {
        stop_requested_.store(false);
        worker_busy_.store(false);
        worker_thread_ = std::thread(&MultiModeNode::WorkerLoop, this);
    }

    void MultiModeNode::StopWorkerThread() {
        stop_requested_.store(true);
        playback_complete_.store(true);
        worker_cv_.notify_all();

        if (diagnostics_timer_) {
            diagnostics_timer_->cancel();
            diagnostics_timer_.reset();
        }
        if (playback_timer_) {
            playback_timer_->cancel();
            playback_timer_.reset();
        }
        if (map_points_timer_) {
            map_points_timer_->cancel();
            map_points_timer_.reset();
        }

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        if (frame_queue_) {
            frame_queue_->Clear();
        }

        slam_initialized_.store(false);
        initialization_retry_pending_.store(false);
        camera_info_wait_logged_.store(false);
        camera_info_ready_logged_.store(false);
        last_tracking_state_code_.store(255);
        pending_initial_frame_.reset();
    }

    void MultiModeNode::WorkerLoop() {
        while (!stop_requested_.load()) {
            std::shared_ptr<FramePacket> packet;
            if (frame_queue_ && frame_queue_->Pop(packet)) {
                if (packet) {
                    worker_busy_.store(true);
                    try {
                        HandleFrame(*packet);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Frame processing failed: %s", e.what());
                    }
                    worker_busy_.store(false);
                }
                continue;
            }

            if (run_mode_ == RunMode::Realtime &&
                input_mode_ == SensorMode::Rgbd &&
                runtime_calibration_from_camera_info_ &&
                !slam_initialized_.load() &&
                initialization_retry_pending_.exchange(false)) {
                if (pending_initial_frame_) {
                    HandleFrame(*pending_initial_frame_);
                    continue;
                }
            }

            std::unique_lock<std::mutex> lock(worker_mutex_);
            worker_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() {
                return stop_requested_.load() ||
                    (frame_queue_ && !frame_queue_->Empty()) ||
                    (run_mode_ == RunMode::Realtime &&
                     input_mode_ == SensorMode::Rgbd &&
                     runtime_calibration_from_camera_info_ &&
                     !slam_initialized_.load() &&
                     initialization_retry_pending_.load());
            });
        }
    }

    void MultiModeNode::HandleFrame(const FramePacket& packet) {
        if (run_mode_ == RunMode::Realtime && !slam_initialized_.load()) {
            pending_initial_frame_ = packet;

            if (input_mode_ == SensorMode::Rgbd && runtime_calibration_from_camera_info_) {
                std::optional<sensor_msgs::msg::CameraInfo> camera_info_snapshot;
                {
                    std::lock_guard<std::mutex> lock(camera_info_mutex_);
                    if (latest_camera_info_) {
                        camera_info_snapshot = latest_camera_info_;
                    }
                }

                if (!camera_info_snapshot) {
                    if (!camera_info_wait_logged_.exchange(true)) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Received synchronized RGB-D frame %llu but camera_info on %s has not arrived yet; initialization is waiting.",
                            static_cast<unsigned long long>(packet.sequence_id),
                            rgb_camera_info_topic_.c_str()
                        );
                    }
                    return;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Preparing RGB-D initialization from synchronized frame %llu | stamp=%s | camera_info=%ux%u",
                    static_cast<unsigned long long>(packet.sequence_id),
                    FormatStamp(packet.stamp).c_str(),
                    camera_info_snapshot->width,
                    camera_info_snapshot->height
                );

                if (!ValidateCameraInfo(*camera_info_snapshot, *pending_initial_frame_)) {
                    return;
                }

                if (!BuildRuntimeSettingsFile(*camera_info_snapshot, *pending_initial_frame_, runtime_settings_path_)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to build runtime RGB-D settings file");
                    return;
                }
            } else {
                const std::string mode_text =
                    input_mode_ == SensorMode::Monocular ? "mono" :
                    input_mode_ == SensorMode::Stereo ? "stereo" :
                    input_mode_ == SensorMode::Rgbd ? "RGB-D" : "stereo-inertial";
                RCLCPP_INFO(
                    this->get_logger(),
                    "Preparing %s initialization from synchronized frame %llu using static settings calibration.",
                    mode_text.c_str(),
                    static_cast<unsigned long long>(packet.sequence_id)
                );
                try {
                    slam_ = std::make_unique<SystemSlam>(
                        vocab_file_,
                        settings_file_,
                        input_mode_,
                        use_viewer_
                    );
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
                    return;
                }
            }

            if (!slam_) {
                try {
                    slam_ = std::make_unique<SystemSlam>(
                        vocab_file_,
                        runtime_settings_path_.string(),
                        input_mode_,
                        use_viewer_
                    );
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
                    return;
                }
            }

            slam_initialized_.store(true);
            initialization_retry_pending_.store(false);
            RCLCPP_INFO(
                this->get_logger(),
                "ORB-SLAM3 initialization completed; tracking output will become visible once ORB-SLAM3 enters Tracking."
            );

            if (slam_initialized_.load()) {
                const FramePacket seed_frame = *pending_initial_frame_;
                pending_initial_frame_.reset();
                ProcessFrame(seed_frame);
            }
            return;
        }

        ProcessFrame(packet);
    }

    void MultiModeNode::ProcessFrame(const FramePacket& packet) {
        if (!slam_ || !slam_initialized_.load()) {
            return;
        }

        const std::int64_t stamp_ns = ToNanoseconds(packet.stamp);
        if (!IsMonotonic(packet)) {
            dropped_duplicate_total_.fetch_add(1);
            return;
        }

        last_processed_sequence_ = packet.sequence_id;
        last_processed_stamp_ns_ = stamp_ns;
        processed_frames_total_.fetch_add(1);

        try {
            std::optional<Sophus::SE3f> pose;
            std::vector<ORB_SLAM3::IMU::Point> imu_measurements = packet.imu_measurements;
            if (run_mode_ == RunMode::Realtime && input_mode_ == SensorMode::StereoInertial) {
                const auto live_imu = CollectImuMeasurements(ToSeconds(packet.stamp));
                imu_measurements.insert(imu_measurements.end(), live_imu.begin(), live_imu.end());
            }

            switch (input_mode_) {
                case SensorMode::Monocular:
                    pose = slam_->TrackMonocular(
                        packet.primary_image,
                        ToSeconds(packet.stamp),
                        imu_measurements,
                        packet.filename
                    );
                    break;
                case SensorMode::Stereo:
                    pose = slam_->TrackStereo(
                        packet.primary_image,
                        packet.secondary_image,
                        ToSeconds(packet.stamp),
                        imu_measurements,
                        packet.filename
                    );
                    break;
                case SensorMode::Rgbd:
                    pose = slam_->TrackRgbd(
                        packet.primary_image,
                        packet.depth_image,
                        ToSeconds(packet.stamp),
                        imu_measurements,
                        packet.filename
                    );
                    break;
                case SensorMode::StereoInertial:
                    pose = slam_->TrackStereo(
                        packet.primary_image,
                        packet.secondary_image,
                        ToSeconds(packet.stamp),
                        imu_measurements,
                        packet.filename
                    );
                    break;
            }

            const auto tracking_state = slam_->GetTrackingState();
            const auto tracking_state_code = static_cast<std::uint8_t>(tracking_state);
            const auto previous_tracking_state_code = last_tracking_state_code_.exchange(tracking_state_code);
            if (previous_tracking_state_code != tracking_state_code) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Tracking state changed to %s.",
                    TrackingStateToString(tracking_state)
                );
                if (tracking_state == TrackingState::Initializing) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Viewer can stay black while ORB-SLAM3 is building its initial map; motion and textured scenes help it converge."
                    );
                } else if (tracking_state == TrackingState::Tracking) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Tracking is active; pose, path, and map outputs should now be visible."
                    );
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Tracking lost; outputs may pause until ORB-SLAM3 relocalizes."
                    );
                }
            }
            auto map_points = slam_->GetMapPointsSnapshot();

            PoseSnapshot snapshot;
            snapshot.stamp = packet.stamp;
            snapshot.tracking_state = tracking_state;
            snapshot.map_points = std::move(map_points);
            snapshot.pose_valid = static_cast<bool>(pose);
            if (pose) {
                snapshot = BuildPoseSnapshot(*pose, packet.stamp, tracking_state, std::move(snapshot.map_points));
            }

            {
                std::lock_guard<std::mutex> lock(latest_snapshot_mutex_);
                latest_output_snapshot_ = snapshot;
            }

            if (output_publishers_) {
                PoseSnapshot pose_snapshot = snapshot;
                if (pose && invert_pose_) {
                    pose_snapshot.pose = ToPoseMessage(*pose);
                }

                output_publishers_->PublishTrackingState(tracking_state);
                if (snapshot.pose_valid) {
                    output_publishers_->PublishPoseBundle(pose_snapshot, snapshot);
                    output_publishers_->PublishPath(snapshot);
                    PublishDynamicTransform(snapshot);
                }
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process image packet: %s", e.what());
        }
    }

    void MultiModeNode::PublishCachedMapPoints() {
        if (!output_publishers_) {
            return;
        }

        PoseSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(latest_snapshot_mutex_);
            snapshot = latest_output_snapshot_;
        }

        output_publishers_->PublishMapPoints(snapshot);
    }

    void MultiModeNode::LogDiagnostics() {
        if (!diagnostic_logging_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (diagnostics_last_report_time_.time_since_epoch().count() == 0) {
            diagnostics_last_report_time_ = now;
            diagnostics_last_processed_total_ = processed_frames_total_.load();
            return;
        }

        const auto elapsed = std::chrono::duration<double>(now - diagnostics_last_report_time_).count();
        const auto processed_total = processed_frames_total_.load();
        const auto processed_delta = processed_total - diagnostics_last_processed_total_;
        const double fps = elapsed > 0.0 ? static_cast<double>(processed_delta) / elapsed : 0.0;
        const std::size_t queue_depth = frame_queue_ ? frame_queue_->Size() : 0U;
        const auto sync_pair_total = sync_pair_total_.load();
        const auto camera_info_total = camera_info_total_.load();
        const std::string mode_text =
            input_mode_ == SensorMode::Monocular ? "mono" :
            input_mode_ == SensorMode::Stereo ? "stereo" :
            input_mode_ == SensorMode::Rgbd ? "rgbd" : "stereo_inertial";
        const std::string run_text = run_mode_ == RunMode::Dataset ? "dataset" : "realtime";
        const std::string init_state = slam_initialized_.load()
            ? "ready"
            : (run_mode_ == RunMode::Realtime && input_mode_ == SensorMode::Rgbd && runtime_calibration_from_camera_info_
                ? (camera_info_total == 0 ? "waiting_camera_info" : sync_pair_total == 0 ? "waiting_rgbd_frame" : "initializing")
                : (sync_pair_total == 0 ? "waiting_input" : "initializing"));

        RCLCPP_INFO(
            this->get_logger(),
            "mode=%s run=%s | fps=%.2f | queue=%zu/%zu | processed=%llu | dup_drop=%llu | sync_pairs=%llu | imu=%llu | init_state=%s",
            mode_text.c_str(),
            run_text.c_str(),
            fps,
            queue_depth,
            frame_queue_ ? frame_queue_->Capacity() : 0U,
            static_cast<unsigned long long>(processed_total),
            static_cast<unsigned long long>(dropped_duplicate_total_.load()),
            static_cast<unsigned long long>(sync_pair_total),
            static_cast<unsigned long long>(imu_messages_total_.load()),
            init_state.c_str()
        );

        diagnostics_last_report_time_ = now;
        diagnostics_last_processed_total_ = processed_total;
    }

    void MultiModeNode::OnMonoImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        if (!frame_queue_) {
            return;
        }

        try {
            FramePacket packet;
            packet.sequence_id = frame_sequence_.fetch_add(1) + 1;
            packet.stamp = msg->header.stamp;
            packet.primary_image = ToMonocularImage(msg, msg->encoding);
            packet.encoding = msg->encoding;
            packet.width = static_cast<int>(msg->width);
            packet.height = static_cast<int>(msg->height);

            const auto result = frame_queue_->Push(std::make_shared<FramePacket>(std::move(packet)));
            if (result.dropped_oldest) {
                dropped_queue_total_.fetch_add(1);
            }

            worker_cv_.notify_one();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert mono image: %s", e.what());
        }
    }

    void MultiModeNode::OnStereoImages(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg
    ) {
        if (!frame_queue_) {
            return;
        }

        const std::int64_t left_stamp_ns = ToNanoseconds(left_msg->header.stamp);
        const std::int64_t right_stamp_ns = ToNanoseconds(right_msg->header.stamp);
        const std::int64_t delta_ns = std::llabs(left_stamp_ns - right_stamp_ns);
        const double delta_sec = static_cast<double>(delta_ns) * 1e-9;
        sync_pair_total_.fetch_add(1);
        sync_delay_sum_ns_.fetch_add(delta_ns);
        latest_sync_delta_ns_.store(delta_ns);

        std::int64_t observed_max_delta_ns = max_sync_delta_ns_.load();
        while (delta_ns > observed_max_delta_ns &&
               !max_sync_delta_ns_.compare_exchange_weak(
                   observed_max_delta_ns,
                   delta_ns,
                   std::memory_order_relaxed,
                   std::memory_order_relaxed
               )) {
        }

        if (delta_sec > sync_tolerance_sec_) {
            return;
        }

        try {
            FramePacket packet;
            packet.sequence_id = frame_sequence_.fetch_add(1) + 1;
            packet.stamp = left_msg->header.stamp;
            packet.primary_image = cv_bridge::toCvShare(left_msg, left_msg->encoding)->image.clone();
            packet.secondary_image = cv_bridge::toCvShare(right_msg, right_msg->encoding)->image.clone();
            packet.encoding = left_msg->encoding;
            packet.width = static_cast<int>(left_msg->width);
            packet.height = static_cast<int>(left_msg->height);

            const auto result = frame_queue_->Push(std::make_shared<FramePacket>(std::move(packet)));
            if (result.dropped_oldest) {
                dropped_queue_total_.fetch_add(1);
            }

            worker_cv_.notify_one();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert stereo images: %s", e.what());
        }
    }

    void MultiModeNode::OnRgbdImages(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg
    ) {
        if (!frame_queue_) {
            return;
        }

        const std::int64_t rgb_stamp_ns = ToNanoseconds(rgb_msg->header.stamp);
        const std::int64_t depth_stamp_ns = ToNanoseconds(depth_msg->header.stamp);
        const std::int64_t delta_ns = std::llabs(rgb_stamp_ns - depth_stamp_ns);
        const double delta_sec = static_cast<double>(delta_ns) * 1e-9;
        sync_pair_total_.fetch_add(1);
        sync_delay_sum_ns_.fetch_add(delta_ns);
        latest_sync_delta_ns_.store(delta_ns);

        std::int64_t observed_max_delta_ns = max_sync_delta_ns_.load();
        while (delta_ns > observed_max_delta_ns &&
               !max_sync_delta_ns_.compare_exchange_weak(
                   observed_max_delta_ns,
                   delta_ns,
                   std::memory_order_relaxed,
                   std::memory_order_relaxed
               )) {
        }

        if (delta_sec > sync_tolerance_sec_) {
            return;
        }

        try {
            FramePacket packet;
            packet.sequence_id = frame_sequence_.fetch_add(1) + 1;
            packet.stamp = rgb_msg->header.stamp;
            packet.primary_image = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding)->image.clone();
            packet.depth_image = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image.clone();
            packet.encoding = rgb_msg->encoding;
            packet.width = static_cast<int>(rgb_msg->width);
            packet.height = static_cast<int>(rgb_msg->height);

            const auto result = frame_queue_->Push(std::make_shared<FramePacket>(std::move(packet)));
            if (result.dropped_oldest) {
                dropped_queue_total_.fetch_add(1);
            }

            if (!slam_initialized_.load() && runtime_calibration_from_camera_info_) {
                initialization_retry_pending_.store(true);
            }

            worker_cv_.notify_one();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert RGB-D images: %s", e.what());
        }
    }

    void MultiModeNode::OnRgbdCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (slam_initialized_.load()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            latest_camera_info_ = *msg;
        }

        latest_camera_info_stamp_ns_.store(ToNanoseconds(msg->header.stamp));
        camera_info_total_.fetch_add(1);
        if (!camera_info_ready_logged_.exchange(true)) {
            RCLCPP_INFO(
                this->get_logger(),
                "CameraInfo received on %s; waiting for the first synchronized RGB-D frame to initialize.",
                rgb_camera_info_topic_.c_str()
            );
        }
        initialization_retry_pending_.store(true);
        worker_cv_.notify_one();
    }

    void MultiModeNode::OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        const auto measurement = ORB_SLAM3::IMU::Point(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z,
            ToSeconds(msg->header.stamp)
        );

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_buffer_.push_back(measurement);
        }

        imu_messages_total_.fetch_add(1);
    }

    void MultiModeNode::OnPlaybackTick() {
        if (playback_complete_.load() &&
            (!frame_queue_ || frame_queue_->Empty()) &&
            !worker_busy_.load()) {
            RCLCPP_INFO(this->get_logger(), "Dataset playback finished");
            FinalizeDatasetPlayback();
            return;
        }

        if (!dataset_timing_initialized_) {
            if (!LoadNextDatasetFrame()) {
                playback_complete_.store(true);
            }
            return;
        }

        if (!IsDatasetFrameDue()) {
            return;
        }

        if (!LoadNextDatasetFrame()) {
            playback_complete_.store(true);
        }
    }

    bool MultiModeNode::LoadDatasetFrames() {
        dataset_entries_.clear();
        dataset_imu_entries_.clear();
        dataset_index_ = 0;
        dataset_imu_index_ = 0;

        const std::filesystem::path association_path(association_file_);
        const std::filesystem::path base_path = association_path.parent_path();
        std::ifstream input(association_file_);
        if (!input.is_open()) {
            return false;
        }

        std::string line;
        while (std::getline(input, line)) {
            if (IsIgnorableLine(line)) {
                continue;
            }

            std::istringstream stream(line);
            double timestamp = 0.0;
            std::string primary_file;
            std::string secondary_file;
            double ignored_timestamp = 0.0;
            std::string depth_file;

            DatasetEntry entry;
            if (input_mode_ == SensorMode::Monocular) {
                if (!(stream >> timestamp >> primary_file)) {
                    continue;
                }
                entry.timestamp = timestamp;
                entry.primary_path = ResolvePath(base_path, primary_file);
                entry.filename = primary_file;
            } else if (input_mode_ == SensorMode::Rgbd) {
                if (!(stream >> timestamp >> primary_file >> ignored_timestamp >> depth_file)) {
                    continue;
                }
                entry.timestamp = timestamp;
                entry.primary_path = ResolvePath(base_path, primary_file);
                entry.depth_path = ResolvePath(base_path, depth_file);
                entry.filename = primary_file;
            } else {
                if (!(stream >> timestamp >> primary_file >> ignored_timestamp >> secondary_file)) {
                    continue;
                }
                entry.timestamp = timestamp;
                entry.primary_path = ResolvePath(base_path, primary_file);
                entry.secondary_path = ResolvePath(base_path, secondary_file);
                entry.filename = primary_file;
            }

            dataset_entries_.push_back(std::move(entry));
        }

        if (dataset_entries_.empty()) {
            return false;
        }

        if (input_mode_ == SensorMode::StereoInertial) {
            std::ifstream imu_input(imu_file_);
            if (!imu_input.is_open()) {
                return false;
            }

            while (std::getline(imu_input, line)) {
                if (IsIgnorableLine(line)) {
                    continue;
                }

                std::istringstream stream(line);
                double timestamp = 0.0;
                double ax = 0.0;
                double ay = 0.0;
                double az = 0.0;
                double gx = 0.0;
                double gy = 0.0;
                double gz = 0.0;
                if (!(stream >> timestamp >> ax >> ay >> az >> gx >> gy >> gz)) {
                    continue;
                }

                dataset_imu_entries_.push_back(ImuEntry{
                    timestamp,
                    ORB_SLAM3::IMU::Point(ax, ay, az, gx, gy, gz, timestamp)
                });
            }

            if (dataset_imu_entries_.empty()) {
                return false;
            }
        }

        if (!dataset_root_.empty()) {
            const std::filesystem::path dataset_root_path(dataset_root_);
            if (std::filesystem::exists(dataset_root_path) && std::filesystem::is_directory(dataset_root_path)) {
                dataset_output_directory_ = dataset_root_path;
            }
        }

        if (dataset_output_directory_.empty()) {
            dataset_output_directory_ = association_path.parent_path();
        }
        if (dataset_output_directory_.empty()) {
            dataset_output_directory_ = std::filesystem::current_path();
        }

        return true;
    }

    bool MultiModeNode::LoadNextDatasetFrame() {
        if (dataset_entries_.empty()) {
            return false;
        }

        if (dataset_index_ >= dataset_entries_.size()) {
            if (!loop_) {
                return false;
            }

            dataset_index_ = 0;
            dataset_imu_index_ = 0;
            dataset_timing_initialized_ = false;
            frame_sequence_.store(0);
            last_processed_sequence_ = 0;
            last_processed_stamp_ns_ = -1;
        }

        const auto& entry = dataset_entries_[dataset_index_];
        FramePacket packet;
        packet.sequence_id = frame_sequence_.fetch_add(1) + 1;
        packet.stamp = ToStamp(entry.timestamp);
        packet.filename = entry.filename;

        if (input_mode_ == SensorMode::Monocular) {
            packet.primary_image = cv::imread(entry.primary_path.string(), cv::IMREAD_GRAYSCALE);
        } else if (input_mode_ == SensorMode::Rgbd) {
            packet.primary_image = cv::imread(entry.primary_path.string(), cv::IMREAD_COLOR);
            packet.depth_image = cv::imread(entry.depth_path.string(), cv::IMREAD_UNCHANGED);
        } else {
            packet.primary_image = cv::imread(entry.primary_path.string(), cv::IMREAD_GRAYSCALE);
            packet.secondary_image = cv::imread(entry.secondary_path.string(), cv::IMREAD_GRAYSCALE);
        }

        if (packet.primary_image.empty() ||
            ((input_mode_ == SensorMode::Rgbd) && packet.depth_image.empty()) ||
            ((input_mode_ == SensorMode::Stereo || input_mode_ == SensorMode::StereoInertial) &&
             packet.secondary_image.empty())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load dataset frame for timestamp %.6f", entry.timestamp);
            return false;
        }

        if (input_mode_ == SensorMode::StereoInertial) {
            while (dataset_imu_index_ < dataset_imu_entries_.size() &&
                   dataset_imu_entries_[dataset_imu_index_].timestamp <= entry.timestamp) {
                packet.imu_measurements.push_back(dataset_imu_entries_[dataset_imu_index_].measurement);
                ++dataset_imu_index_;
            }
        }

        packet.encoding = "dataset";
        packet.width = packet.primary_image.cols;
        packet.height = packet.primary_image.rows;

        if (!dataset_timing_initialized_) {
            dataset_sequence_start_stamp_ = entry.timestamp;
            dataset_sequence_start_wall_time_ = std::chrono::steady_clock::now();
            dataset_timing_initialized_ = true;
        }

        const auto result = frame_queue_->Push(std::make_shared<FramePacket>(std::move(packet)));
        if (result.dropped_oldest) {
            dropped_queue_total_.fetch_add(1);
        }

        worker_cv_.notify_one();
        ++dataset_index_;
        return true;
    }

    bool MultiModeNode::IsDatasetFrameDue() const noexcept {
        if (dataset_index_ >= dataset_entries_.size() || !dataset_timing_initialized_) {
            return false;
        }

        const auto& entry = dataset_entries_[dataset_index_];
        const double rate = playback_rate_ > 0.0 ? playback_rate_ : 1.0;
        const double elapsed_seconds = (entry.timestamp - dataset_sequence_start_stamp_) / rate;
        const auto target_time = dataset_sequence_start_wall_time_ +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(elapsed_seconds)
            );

        return std::chrono::steady_clock::now() >= target_time;
    }

    void MultiModeNode::FinalizeDatasetPlayback() {
        playback_complete_.store(true);
        stop_requested_.store(true);

        if (playback_timer_) {
            playback_timer_->cancel();
        }

        StopWorkerThread();

        if (slam_) {
            const auto camera_trajectory = BuildTrajectoryPath("CameraTrajectory.txt");
            const auto keyframe_trajectory = BuildTrajectoryPath("KeyFrameTrajectory.txt");

            if (input_mode_ == SensorMode::Rgbd) {
                slam_->SaveTUMTrajectories(camera_trajectory, keyframe_trajectory);
            } else {
                slam_->SaveEuRoCTrajectories(camera_trajectory, keyframe_trajectory);
            }
        }

        frame_queue_.reset();
        dataset_entries_.clear();
        dataset_imu_entries_.clear();
        output_publishers_.reset();
        slam_.reset();
        if (!runtime_settings_path_.empty()) {
            std::error_code ec;
            std::filesystem::remove(runtime_settings_path_, ec);
            runtime_settings_path_.clear();
        }

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    bool MultiModeNode::IsMonotonic(const FramePacket& packet) const noexcept {
        const std::int64_t stamp_ns = ToNanoseconds(packet.stamp);
        if (packet.sequence_id <= last_processed_sequence_) {
            return false;
        }
        if (stamp_ns <= last_processed_stamp_ns_) {
            return false;
        }
        return true;
    }

    bool MultiModeNode::IsRgbEncoding(const std::string& encoding) const noexcept {
        const std::string lower = ToLowerCopy(encoding);
        return lower.find("rgb") != std::string::npos && lower.find("bgr") == std::string::npos;
    }

    std::string MultiModeNode::ToLowerCopy(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return value;
    }

    std::string MultiModeNode::TrimCopy(const std::string& value) {
        const auto first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return "";
        }
        const auto last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    }

    bool MultiModeNode::IsIgnorableLine(const std::string& line) {
        const auto trimmed = TrimCopy(line);
        if (trimmed.empty()) {
            return true;
        }
        return trimmed.front() == '#';
    }

    std::filesystem::path MultiModeNode::ResolvePath(
        const std::filesystem::path& base_path,
        const std::string& file_path
    ) {
        const std::filesystem::path path(file_path);
        return path.is_absolute() ? path : (base_path / path);
    }

    builtin_interfaces::msg::Time MultiModeNode::ToStamp(double seconds) noexcept {
        const auto total_nanoseconds = static_cast<std::int64_t>(seconds * 1e9);
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<std::int32_t>(total_nanoseconds / 1000000000LL);
        stamp.nanosec = static_cast<std::uint32_t>(total_nanoseconds % 1000000000LL);
        return stamp;
    }

    double MultiModeNode::ToSeconds(const builtin_interfaces::msg::Time& stamp) noexcept {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    std::int64_t MultiModeNode::ToNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
        return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
    }

    double MultiModeNode::ToMilliseconds(std::int64_t nanoseconds) noexcept {
        return static_cast<double>(nanoseconds) * 1e-6;
    }

    std::string MultiModeNode::FormatStamp(const builtin_interfaces::msg::Time& stamp) {
        std::ostringstream stream;
        stream << stamp.sec << '.' << std::setw(9) << std::setfill('0') << stamp.nanosec;
        return stream.str();
    }

    std::string MultiModeNode::FormatStampNs(std::int64_t nanoseconds) {
        return FormatStamp(ToStamp(static_cast<double>(nanoseconds) * 1e-9));
    }

    std::string MultiModeNode::FormatDouble(double value) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(9) << value;
        return stream.str();
    }

    std::string MultiModeNode::NormalizeModeName(std::string value) {
        value = ToLowerCopy(std::move(value));
        for (char& ch : value) {
            if (ch == '-') {
                ch = '_';
            }
        }
        return value;
    }

    SensorMode MultiModeNode::ParseSensorMode(const std::string& mode) {
        const std::string normalized = NormalizeModeName(mode);
        if (normalized == "mono" || normalized == "monocular") {
            return SensorMode::Monocular;
        }
        if (normalized == "stereo") {
            return SensorMode::Stereo;
        }
        if (normalized == "rgbd" || normalized == "rgb_d") {
            return SensorMode::Rgbd;
        }
        if (normalized == "stereo_inertial" || normalized == "stereoimu") {
            return SensorMode::StereoInertial;
        }

        throw std::invalid_argument("Unknown ORB-SLAM3 mode: " + mode);
    }

    MultiModeNode::RunMode MultiModeNode::ParseRunMode(const std::string& mode) {
        const std::string normalized = NormalizeModeName(mode);
        if (normalized == "dataset" || normalized == "offline") {
            return RunMode::Dataset;
        }
        if (normalized == "realtime" || normalized == "live") {
            return RunMode::Realtime;
        }

        throw std::invalid_argument("Unknown run mode: " + mode);
    }

    bool MultiModeNode::RewriteSettingsFile(
        const std::filesystem::path& source_path,
        const std::filesystem::path& destination_path,
        const std::vector<std::pair<std::string, std::string>>& overrides
    ) {
        std::ifstream input(source_path);
        if (!input.is_open()) {
            return false;
        }

        std::ofstream output(destination_path);
        if (!output.is_open()) {
            return false;
        }

        std::string line;
        while (std::getline(input, line)) {
            bool replaced = false;
            const std::string trimmed = TrimCopy(line);
            for (const auto& [key, value] : overrides) {
                if (trimmed.rfind(key, 0) == 0) {
                    output << key << ": " << value << '\n';
                    replaced = true;
                    break;
                }
            }

            if (!replaced) {
                output << line << '\n';
            }
        }

        output << '\n' << "# Runtime overrides" << '\n';
        for (const auto& [key, value] : overrides) {
            output << key << ": " << value << '\n';
        }

        return output.good();
    }

    bool MultiModeNode::ValidateCameraInfo(
        const sensor_msgs::msg::CameraInfo& camera_info,
        const FramePacket& packet
    ) const {
        if (camera_info.width == 0 || camera_info.height == 0) {
            RCLCPP_ERROR(this->get_logger(), "CameraInfo has zero resolution");
            return false;
        }

        if (packet.width > 0 && packet.height > 0 &&
            (camera_info.width != static_cast<std::uint32_t>(packet.width) ||
             camera_info.height != static_cast<std::uint32_t>(packet.height))) {
            RCLCPP_ERROR(
                this->get_logger(),
                "CameraInfo resolution (%ux%u) does not match image (%dx%d)",
                camera_info.width,
                camera_info.height,
                packet.width,
                packet.height
            );
            return false;
        }

        if (camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 || !std::isfinite(camera_info.k[0]) ||
            !std::isfinite(camera_info.k[4]) || !std::isfinite(camera_info.k[2]) ||
            !std::isfinite(camera_info.k[5])) {
            RCLCPP_ERROR(this->get_logger(), "CameraInfo contains invalid intrinsic values");
            return false;
        }

        return true;
    }

    bool MultiModeNode::BuildRuntimeSettingsFile(
        const sensor_msgs::msg::CameraInfo& camera_info,
        const FramePacket& packet,
        std::filesystem::path& runtime_settings_file
    ) {
        std::error_code temp_dir_error;
        const std::filesystem::path temp_dir = std::filesystem::temp_directory_path(temp_dir_error);
        if (temp_dir_error) {
            return false;
        }

        const auto unique_name = std::string("orbslam3_ros_runtime_") +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml";
        runtime_settings_file = temp_dir / unique_name;

        const bool rgb_is_rgb = IsRgbEncoding(packet.encoding);
        const std::vector<std::pair<std::string, std::string>> overrides = {
            {"Camera1.fx", FormatDouble(camera_info.k[0])},
            {"Camera1.fy", FormatDouble(camera_info.k[4])},
            {"Camera1.cx", FormatDouble(camera_info.k[2])},
            {"Camera1.cy", FormatDouble(camera_info.k[5])},
            {"Camera.width", std::to_string(packet.width)},
            {"Camera.height", std::to_string(packet.height)},
            {"Camera.RGB", rgb_is_rgb ? "1" : "0"},
        };

        return RewriteSettingsFile(settings_file_, runtime_settings_file, overrides);
    }

    std::vector<ORB_SLAM3::IMU::Point> MultiModeNode::CollectImuMeasurements(double timestamp) {
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements;

        std::lock_guard<std::mutex> lock(imu_mutex_);
        while (!imu_buffer_.empty() && imu_buffer_.front().t <= timestamp) {
            imu_measurements.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }

        return imu_measurements;
    }

    std::filesystem::path MultiModeNode::BuildTrajectoryPath(const char* filename) const {
        return dataset_output_directory_ / filename;
    }

}  // namespace orbslam3_ros
