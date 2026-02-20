#include <chrono>
#include <memory>

#include "bizon_behavior_servers/plugins/board_plugin.hpp"
#include "bizon_behavior_servers/plugins/image_processing_utils.hpp"

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
        }
        

        RCLCPP_INFO(node_.lock()->get_logger(), "Image received successfully, starting inference...");

        std::vector<cv::Rect> bboxes;
        cv::Mat image_cropped_;
        std::vector<cv::Mat> cell_images_;
        std::vector<cv::Point2f> points_crop;
        if (!cell_detector_.infer(latest_image_, image_cropped_, bboxes, points_crop))
        {
            RCLCPP_WARN(node_.lock()->get_logger(), "Board detection failed, retrying on next frame...");
            is_camera_active_.store(true);
            return;
        }
        for (const auto &bbox : bboxes)
        {
            cv::Mat cell = image_cropped_(bbox).clone();
            cell_images_.push_back(cell);
        }

        std::vector<std::string> class_names;
        cell_classifier_.infer(cell_images_, class_names);

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
    }

} // namespace bizon_behaviors
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bizon_behaviors::BoardPlugin, bizon_core::Behavior)