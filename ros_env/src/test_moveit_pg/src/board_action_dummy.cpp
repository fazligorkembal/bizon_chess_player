#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "bizon_behavior_servers/plugins/cell_detector.hpp"
#include "bizon_behavior_servers/plugins/cell_classifier.hpp"

class CameraListener : public rclcpp::Node
{
public:
  CameraListener()
  : Node("camera_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/bizon2/top_camera",
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&CameraListener::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Camera listener node started");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Invalid image received");
      return;
    }

    // Manually create cv::Mat from raw data
    int cv_type = CV_8UC3;
    if (msg->encoding == "mono8") {
      cv_type = CV_8UC1;
    } else if (msg->encoding == "rgba8" || msg->encoding == "bgra8") {
      cv_type = CV_8UC4;
    }

    cv::Mat raw_image(msg->height, msg->width, cv_type,
      const_cast<uint8_t *>(msg->data.data()), msg->step);

    cv::Mat processed_image;
    if (msg->encoding == "rgb8") {
      cv::cvtColor(raw_image, processed_image, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "rgba8") {
      cv::cvtColor(raw_image, processed_image, cv::COLOR_RGBA2BGR);
    } else if (msg->encoding == "bgra8") {
      cv::cvtColor(raw_image, processed_image, cv::COLOR_BGRA2BGR);
    } else if (msg->encoding == "mono8") {
      cv::cvtColor(raw_image, processed_image, cv::COLOR_GRAY2BGR);
    } else {
      processed_image = raw_image.clone();
    }

    std::vector<cv::Rect> bboxes;
    cv::Mat image_cropped_;
    std::vector<cv::Mat> cell_images_;
    std::vector<cv::Point2f> points_crop_;
    if (!cell_detector_.infer(processed_image, image_cropped_, bboxes, points_crop_)) {
      RCLCPP_WARN(this->get_logger(), "Board detection failed, skipping frame...");
      cv::resize(processed_image, processed_image, cv::Size(540, 540), 0, 0, cv::INTER_AREA);
      cv::imshow("White Robot Board Detection", processed_image);
      cv::waitKey(10);
      return;
    }
    for (const auto & bbox : bboxes) {
      cv::Mat cell = image_cropped_(bbox).clone();
      cell_images_.push_back(cell);
    }

    std::vector<std::string> class_names;
    cell_classifier_.infer(cell_images_, class_names);
    std::cout << "Detected " << class_names.size() << " cells" << std::endl;

    for (size_t i = 0; i < class_names.size(); i++) {
      cv::Rect bbox = bboxes[i];
      std::string class_name = class_names[i];
      if (class_name == "emp") {
        cv::rectangle(image_cropped_, bbox, cv::Scalar(0, 0, 100), 2);
        cv::putText(
          image_cropped_, class_name, cv::Point(bbox.x + 5, bbox.y + 15),
          cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 100), 2);
      }

      if (class_name == "p" || class_name == "n" || class_name == "b" || class_name == "r" ||
        class_name == "q" || class_name == "k")
      {
        cv::rectangle(image_cropped_, bbox, cv::Scalar(0, 0, 0), 2);
        cv::putText(
          image_cropped_, class_name, cv::Point(bbox.x + 5, bbox.y + 15),
          cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 2);
      }

      if (class_name == "P" || class_name == "N" || class_name == "B" || class_name == "R" ||
        class_name == "Q" || class_name == "K")
      {
        cv::rectangle(image_cropped_, bbox, cv::Scalar(255, 255, 255), 2);
        cv::putText(
          image_cropped_, class_name, cv::Point(bbox.x + 5, bbox.y + 15),
          cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
      }
    }


    if (points_crop_.size() == 4) {
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Point> contour;
      for (const auto & pt : points_crop_) {
        contour.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
      }
      contours.push_back(contour);
      cv::polylines(processed_image, contours, true, cv::Scalar(0, 255, 0), 2);
    }
    cv::resize(processed_image, processed_image, cv::Size(540, 540), 0, 0, cv::INTER_AREA);
    cv::resize(image_cropped_, image_cropped_, cv::Size(540, 540), 0, 0, cv::INTER_AREA);

    cv::imshow("White Robot Board Detection", processed_image);
    cv::imshow("White Robot Cell Classification", image_cropped_);
    cv::waitKey(10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::Mat latest_image_;
  CellDetector cell_detector_{1280, 720};   // TODO: Make these parameters configurable
  bizon_behaviors::CellClassifier cell_classifier_{
    "/home/user/Documents/bizon_chess_player/models/yolo26n-chessboard_v2.wts", 0.50f, 1.00f, 512,
    "m"};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraListener>());
  rclcpp::shutdown();
  return 0;
}
