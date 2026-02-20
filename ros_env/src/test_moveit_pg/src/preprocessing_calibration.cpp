#include <iostream>
#include "bizon_behavior_servers/plugins/image_processing_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ChessboardIntersectionNode : public rclcpp::Node
{
public:
    ChessboardIntersectionNode() : Node("chessboard_intersection_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bizon2/top_camera", 10,
            std::bind(&ChessboardIntersectionNode::imageCallback, this, std::placeholders::_1));
    }

private:
    bool lineIntersection(cv::Vec4i line1, cv::Vec4i line2, cv::Point2f& intersection)
    {
        float x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
        float x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];

        float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (std::abs(denom) < 1e-10)
            return false;

        float px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
        float py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

        intersection = cv::Point2f(px, py);
        return true;
    }

    bool isHorizontal(cv::Vec4i line, float angle_threshold = 15.0)
    {
        float angle = std::abs(std::atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI);
        return angle < angle_threshold || angle > (180 - angle_threshold);
    }

    bool isVertical(cv::Vec4i line, float angle_threshold = 15.0)
    {
        float angle = std::abs(std::atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI);
        return angle > (90 - angle_threshold) && angle < (90 + angle_threshold);
    }

    std::vector<cv::Vec4i> mergeLines(std::vector<cv::Vec4i>& lines, bool horizontal, float distance_threshold = 20.0)
    {
        if (lines.empty()) return lines;

        std::vector<cv::Vec4i> merged;
        std::vector<bool> used(lines.size(), false);

        for (size_t i = 0; i < lines.size(); i++)
        {
            if (used[i]) continue;

            std::vector<cv::Vec4i> cluster;
            cluster.push_back(lines[i]);
            used[i] = true;

            for (size_t j = i + 1; j < lines.size(); j++)
            {
                if (used[j]) continue;

                float dist;
                if (horizontal)
                    dist = std::abs((lines[i][1] + lines[i][3]) / 2.0 - (lines[j][1] + lines[j][3]) / 2.0);
                else
                    dist = std::abs((lines[i][0] + lines[i][2]) / 2.0 - (lines[j][0] + lines[j][2]) / 2.0);

                if (dist < distance_threshold)
                {
                    cluster.push_back(lines[j]);
                    used[j] = true;
                }
            }

            float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
            for (const auto& line : cluster)
            {
                x1 += line[0]; y1 += line[1];
                x2 += line[2]; y2 += line[3];
            }
            x1 /= cluster.size(); y1 /= cluster.size();
            x2 /= cluster.size(); y2 /= cluster.size();

            merged.push_back(cv::Vec4i(x1, y1, x2, y2));
        }

        return merged;
    }

    std::vector<cv::Point2f> findChessboardIntersections(cv::Mat& image)
    {
        std::vector<cv::Point2f> intersections;

        cv::Mat gray, edges;
        if (image.channels() == 3)
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        else
            gray = image.clone();

        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
        cv::Canny(gray, edges, 50, 150, 3);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 80, 50, 10);

        if (lines.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No lines detected!");
            return intersections;
        }

        std::vector<cv::Vec4i> horizontal_lines, vertical_lines;
        for (const auto& line : lines)
        {
            if (isHorizontal(line))
                horizontal_lines.push_back(line);
            else if (isVertical(line))
                vertical_lines.push_back(line);
        }

        horizontal_lines = mergeLines(horizontal_lines, true, 15.0);
        vertical_lines = mergeLines(vertical_lines, false, 15.0);

        RCLCPP_INFO(this->get_logger(), "Horizontal lines: %zu, Vertical lines: %zu", 
                    horizontal_lines.size(), vertical_lines.size());

        for (const auto& h_line : horizontal_lines)
        {
            for (const auto& v_line : vertical_lines)
            {
                cv::Point2f intersection;
                if (lineIntersection(h_line, v_line, intersection))
                {
                    if (intersection.x >= 0 && intersection.x < image.cols &&
                        intersection.y >= 0 && intersection.y < image.rows)
                    {
                        intersections.push_back(intersection);
                    }
                }
            }
        }

        std::sort(intersections.begin(), intersections.end(), 
            [](const cv::Point2f& a, const cv::Point2f& b) {
                if (std::abs(a.y - b.y) < 10)
                    return a.x < b.x;
                return a.y < b.y;
            });

        return intersections;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
            cv::Mat input_image = cv_ptr->image;

            if (input_image.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty image");
                return;
            }

            if (input_image.channels() == 1)
                cv::cvtColor(input_image, input_image, cv::COLOR_GRAY2BGR);
            else if (msg->encoding == "rgb8")
                cv::cvtColor(input_image, input_image, cv::COLOR_RGB2BGR);

            
            std::vector<cv::Point2f> intersections = findChessboardIntersections(input_image);
            cv::Mat result = input_image.clone();
            for (size_t i = 0; i < intersections.size(); i++)
            {
                cv::circle(result, intersections[i], 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(result, std::to_string(i), intersections[i], 
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
            }

            RCLCPP_INFO(this->get_logger(), "Found %zu intersections", intersections.size());

            cv::imshow("Chessboard Intersections", result);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChessboardIntersectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}