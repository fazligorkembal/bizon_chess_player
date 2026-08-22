#ifndef BIZON_BEHAVIOR_SERVERS_PLUGINS_BOARD_PLUGIN_HPP
#define BIZON_BEHAVIOR_SERVERS_PLUGINS_BOARD_PLUGIN_HPP

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"

#include "bizon_behavior_servers/timed_behavior.hpp"
#include "bizon_msgs/action/board.hpp"

#include <opencv2/opencv.hpp>

#include "bizon_behavior_servers/plugins/cell_detector.hpp"
#include "bizon_behavior_servers/plugins/cell_classifier.hpp"
using namespace nvinfer1;

namespace bizon_behaviors
{
    using BoardAction = bizon_msgs::action::Board;

    class BoardPlugin : public TimedBehavior<BoardAction>
    {
    public:
        using BoardActionGoal = BoardAction::Goal;

        /**
         * @brief A constructor for behavior_server::Board
         */
        BoardPlugin();
        ~BoardPlugin();

        /**
         * @brief Initialization to run behavior
         * @param command Goal to execute
         * @return Status of behavior
         */
        ResultStatus onRun(const std::shared_ptr<const BoardActionGoal> command) override;

        /**
         * @brief Loop function to run behavior
         * @return Status of behavior
         */
        ResultStatus onCycleUpdate() override;

        /**
         * @brief Called on action completion to fill result
         * @param result Result to fill
         */
        void onActionCompletion(std::shared_ptr<BoardAction::Result> result) override;

    protected:
        rclcpp::Time board_end_;
        BoardAction::Feedback::SharedPtr feedback_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        cv::Mat latest_image_;
        mutable std::mutex image_mutex_;
        void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        std::atomic<bool> is_camera_active_{false};
        CellDetector cell_detector_{1280, 720}; // TODO: Make these parameters configurable
        // Geometry must match the weights file. yolo26n is the nano variant:
        // gd 0.50, gw 0.25, max_channels 1024 (see parse_args in tensorrt/utils.h).
        // The medium values that used to sit here asked for a 64-channel first conv
        // against nano weights holding 16, so engine building aborted.
        CellClassifier cell_classifier_{"/home/user/Documents/bizon_chess_player/models/yolo26n-chessboard_v2.wts", 0.50f, 0.25f, 1024, "n"};
        std::string results_;
        int32_t empty_count_{0};
        std::atomic<bool> is_result_ready_{false};
        bool is_black_side_{false};

    };
}

#endif // BIZON_BEHAVIOR_SERVERS_PLUGINS_BOARD_PLUGIN_HPP