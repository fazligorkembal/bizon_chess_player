#include <chrono>
#include <memory>

#include "bizon_behavior_servers/plugins/board_plugin.hpp"
#include "bizon_behavior_servers/plugins/image_processing_utils.hpp"
#include "bizon_behavior_servers/debug_session_format.hpp"

/////////////////////////////
#include <fstream>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>

#include "bizon_behavior_servers/tensorrt/postprocess.h"
#include "bizon_behavior_servers/tensorrt/preprocess.h"
/////////////////////////////
namespace bizon_behaviors
{
    BoardPlugin::BoardPlugin() : TimedBehavior<BoardAction>(),
                                 feedback_(std::make_shared<BoardAction::Feedback>())
    {
    }

    void BoardPlugin::onConfigure()
    {
        auto node = node_.lock();

        // debug_session_dir is a top-level (not behavior_name_-namespaced)
        // parameter: it names one directory for the whole game, shared by
        // every plugin in this process, not a per-behavior setting -- see
        // bizon_player.launch.py, which resolves it once and hands the same
        // value to every plugin's namespace.
        if (!node->has_parameter("debug_session_dir")) {
            node->declare_parameter("debug_session_dir", std::string(""));
        }
        std::string debug_session_dir;
        node->get_parameter("debug_session_dir", debug_session_dir);
        DebugSession::instance().configure(debug_session_dir);

        // BoardPlugin is the only plugin that ever touches an image, so it
        // is the one that has to register the dump callback -- this is the
        // whole cross-plugin seam DecisionPlugin uses to get a bundle
        // written for a failure it detected but has no pixels for.
        DebugSession::instance().registerImageDumpCallback(
            [this](const std::string & bundle_dir) { writeImageArtifacts(bundle_dir); });
    }

    void BoardPlugin::pushFrame(const cv::Mat & frame)
    {
        // Caller already holds image_mutex_.
        frame_ring_.push_back(frame);
        while (frame_ring_.size() > kFrameRingSize) {
            frame_ring_.pop_front();
        }
    }

    void BoardPlugin::writeImageArtifacts(const std::string & bundle_dir)
    {
        // Snapshot everything under lock, then write outside the lock --
        // cv::imwrite for ~65 images is not something cameraCallback should
        // block on for the whole duration.
        std::deque<cv::Mat> frames;
        cv::Mat image_cropped;
        std::vector<cv::Rect> bboxes;
        std::vector<cv::Mat> cell_images;
        std::vector<std::string> class_names;
        std::vector<float> confidences;
        bool black_side = false;
        bool has_cells = false;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            frames = frame_ring_;
            image_cropped = last_image_cropped_.clone();
            bboxes = last_bboxes_;
            cell_images = last_cell_images_;
            class_names = last_class_names_pre_reversal_;
            confidences = last_confidences_;
            black_side = last_is_black_side_;
            has_cells = has_cell_data_;
        }

        auto logger = node_.lock()->get_logger();

        // Priority order per the spec refinement: cells_overlay.png first
        // (answers "is the grid even aligned", the first thing to rule
        // out), then the contact sheet, then the frames. context.txt is
        // written by the caller before this runs. Each step is wrapped
        // separately so one failure (e.g. an empty cell_images entry) never
        // stops the rest.
        if (has_cells && !image_cropped.empty()) {
            try {
                cv::Mat overlay = image_cropped.clone();
                for (const auto & bbox : bboxes) {
                    cv::rectangle(overlay, bbox, cv::Scalar(0, 255, 0), 2);
                }
                cv::imwrite(bundle_dir + "/cells_overlay.png", overlay);
                cv::imwrite(bundle_dir + "/board_cropped.png", image_cropped);
            } catch (const std::exception & ex) {
                RCLCPP_WARN(logger, "[debug bundle] failed to write cells_overlay/board_cropped: %s", ex.what());
            }
        }

        if (has_cells && cell_images.size() == 64) {
            try {
                constexpr int kThumb = 80;
                cv::Mat sheet(kThumb * 8, kThumb * 8, CV_8UC3, cv::Scalar(32, 32, 32));
                std::ofstream labels(bundle_dir + "/cell_labels.txt");
                for (int i = 0; i < 64; ++i) {
                    const std::string square = debug_format::squareForCellIndex(i, black_side);
                    const std::string & cls =
                        (static_cast<size_t>(i) < class_names.size()) ? class_names[static_cast<size_t>(i)] : "?";

                    if (labels.is_open()) {
                        labels << square << " " << cls;
                        if (static_cast<size_t>(i) < confidences.size()) {
                            labels << " " << confidences[static_cast<size_t>(i)];
                        }
                        labels << "\n";
                    }

                    if (cell_images[static_cast<size_t>(i)].empty()) {
                        continue;
                    }
                    cv::Mat thumb;
                    cv::resize(cell_images[static_cast<size_t>(i)], thumb, cv::Size(kThumb, kThumb));
                    if (thumb.channels() == 1) {
                        cv::cvtColor(thumb, thumb, cv::COLOR_GRAY2BGR);
                    }
                    const int row = i / 8;
                    const int col = i % 8;
                    thumb.copyTo(sheet(cv::Rect(col * kThumb, row * kThumb, kThumb, kThumb)));
                    cv::putText(
                        sheet, cls, cv::Point(col * kThumb + 2, row * kThumb + 14),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
                }
                cv::imwrite(bundle_dir + "/cells_contact_sheet.png", sheet);
            } catch (const std::exception & ex) {
                RCLCPP_WARN(logger, "[debug bundle] failed to write cells_contact_sheet/cell_labels: %s", ex.what());
            }
        }

        try {
            std::filesystem::create_directories(bundle_dir + "/frames");
            const int offset_of_oldest = -static_cast<int>(frames.size()) + 1;
            for (size_t i = 0; i < frames.size(); ++i) {
                if (frames[i].empty()) {
                    continue;
                }
                const int offset = offset_of_oldest + static_cast<int>(i);
                cv::imwrite(bundle_dir + "/frames/frame_" + std::to_string(offset) + ".png", frames[i]);
            }
        } catch (const std::exception & ex) {
            RCLCPP_WARN(logger, "[debug bundle] failed to write frames: %s", ex.what());
        }
    }

    void BoardPlugin::dumpBoardDetectFailureBundle()
    {
        // Board detection failed before a crop or any cell data existed, so
        // the only evidence available is the raw frame ring buffer -- still
        // useful (was the board even in frame? was it a lighting spike?).
        const std::string bundle_dir = DebugSession::instance().beginErrorBundle("board_detect_failed");
        if (bundle_dir.empty()) {
            return;  // not configured, or the bundle directory could not be created
        }
        DebugSession::instance().writeContext(
            bundle_dir,
            "failed: board detection (CellDetector::infer) could not find a board contour in "
            "the latest camera frame\n");
        writeImageArtifacts(bundle_dir);
    }

    void BoardPlugin::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!is_camera_active_.load())
        {
            return;
        }

        is_camera_active_.store(false);

        if (msg->width == 0 || msg->height == 0 || msg->data.empty())
        {
            RCLCPP_WARN(node_.lock()->get_logger(), "Invalid image received");
            return;
        }

        // Manually create cv::Mat from raw data
        int cv_type = CV_8UC3;
        if (msg->encoding == "mono8")
        {
            cv_type = CV_8UC1;
        }
        else if (msg->encoding == "rgba8" || msg->encoding == "bgra8")
        {
            cv_type = CV_8UC4;
        }

        cv::Mat raw_image(msg->height, msg->width, cv_type,
                          const_cast<uint8_t *>(msg->data.data()), msg->step);

        cv::Mat processed_image;
        if (msg->encoding == "rgb8")
        {
            cv::cvtColor(raw_image, processed_image, cv::COLOR_RGB2BGR);
        }
        else if (msg->encoding == "rgba8")
        {
            cv::cvtColor(raw_image, processed_image, cv::COLOR_RGBA2BGR);
        }
        else if (msg->encoding == "bgra8")
        {
            cv::cvtColor(raw_image, processed_image, cv::COLOR_BGRA2BGR);
        }
        else if (msg->encoding == "mono8")
        {
            cv::cvtColor(raw_image, processed_image, cv::COLOR_GRAY2BGR);
        }
        else
        {
            processed_image = raw_image.clone();
        }

        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = std::move(processed_image);
            // Ring buffer of raw camera frames for error bundles -- pushed
            // for every frame received, not just ones a board is found in,
            // so "was it already wrong a frame ago" can be answered even
            // when the very next frame is the one that fails detection.
            pushFrame(latest_image_.clone());
        }


        RCLCPP_INFO(node_.lock()->get_logger(), "Image received successfully, starting inference...");

        std::vector<cv::Rect> bboxes;
        cv::Mat image_cropped_;
        std::vector<cv::Mat> cell_images_;
        std::vector<cv::Point2f> points_crop;
        if (!cell_detector_.infer(latest_image_, image_cropped_, bboxes, points_crop))
        {
            RCLCPP_WARN(node_.lock()->get_logger(), "Board detection failed, retrying on next frame...");
            try
            {
                dumpBoardDetectFailureBundle();
            }
            catch (const std::exception &ex)
            {
                // Debug capture must never fail a goal -- log and carry on.
                RCLCPP_WARN(node_.lock()->get_logger(), "[debug bundle] board_detect_failed capture threw: %s", ex.what());
            }
            is_camera_active_.store(true);
            return;
        }
        for (const auto &bbox : bboxes)
        {
            cv::Mat cell = image_cropped_(bbox).clone();
            cell_images_.push_back(cell);
        }

        std::vector<std::string> class_names;
        std::vector<float> confidences;
        cell_classifier_.infer(cell_images_, class_names, &confidences);

        // Snapshot for error bundles *before* the black-side reversal below,
        // so debug_format::squareForCellIndex() can recover each crop's
        // real board square from bbox/scan order -- see the header comment
        // on last_class_names_pre_reversal_.
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            last_image_cropped_ = image_cropped_.clone();
            last_bboxes_ = bboxes;
            last_cell_images_ = cell_images_;
            last_class_names_pre_reversal_ = class_names;
            last_confidences_ = confidences;
            last_is_black_side_ = is_black_side_;
            has_cell_data_ = true;
        }

        if (is_black_side_)
        {
            std::reverse(class_names.begin(), class_names.end());
        }

        results_ = "";
        empty_count_ = 0;

        for (size_t i = 0; i < class_names.size(); i++)
        {
            std::string class_name = class_names[i];

            if (class_name == "emp")
            {
                empty_count_++;
            }
            else
            {
                if (empty_count_ > 0)
                {
                    results_ += std::to_string(empty_count_);
                    empty_count_ = 0;
                }
                results_ += class_name;
            }

            if ((i + 1) % 8 == 0)
            {
                if (empty_count_ > 0)
                {
                    results_ += std::to_string(empty_count_);
                    empty_count_ = 0;
                }

                if (i != 63)
                {
                    results_ += "/";
                }
            }
        }

        is_result_ready_.store(true);
        RCLCPP_INFO(node_.lock()->get_logger(), "Inference complete, FEN: '%s'", results_.c_str());
        
    }

    BoardPlugin::~BoardPlugin() = default;
    ResultStatus BoardPlugin::onRun(const std::shared_ptr<const BoardAction::Goal> command)
    {
        // Clear any previous image
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_.release();
        }

        if (!camera_subscription_)
        {
            std::string topic_name = std::string(node_.lock()->get_namespace()) + "/top_camera";
            camera_subscription_ = node_.lock()->create_subscription<sensor_msgs::msg::Image>(
                topic_name,
                1,
                std::bind(&BoardPlugin::cameraCallback, this, std::placeholders::_1));
        }

        is_black_side_ = (command->player_side == "black");
        is_camera_active_.store(true);
        is_result_ready_.store(false);
        board_end_ = node_.lock()->now() + rclcpp::Duration(command->time);
        return ResultStatus{Status::SUCCEEDED};
    }

    ResultStatus BoardPlugin::onCycleUpdate()
    {
        if (!is_result_ready_.load())
        {
            return ResultStatus{Status::RUNNING};
        }

        RCLCPP_INFO(node_.lock()->get_logger(), "Image captured and inference complete");
        return ResultStatus{Status::SUCCEEDED};
    }

    void BoardPlugin::onActionCompletion(std::shared_ptr<BoardAction::Result> result)
    {
        is_camera_active_.store(false);
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_.release();
        }
        RCLCPP_INFO(node_.lock()->get_logger(), "Filling result FEN");
        result->fen = results_;
        auto elapsed_time = node_.lock()->now() - (board_end_ - rclcpp::Duration(result->total_elapsed_time));
        result->total_elapsed_time = elapsed_time;
        result->error_code = 0; // TODO: Define error codes for different failure scenarios

        DebugSession::instance().logEvent(
            "board", "fen=" + (results_.empty() ? std::string("-") : results_) +
            " error_code=" + std::to_string(result->error_code));
    }

} // namespace bizon_behaviors
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bizon_behaviors::BoardPlugin, bizon_core::Behavior)