#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CellDetector
{
public:
    CellDetector(int32_t input_width, int32_t input_height)
        : input_width_(input_width), input_height_(input_height)
    {
        clahe_processor_ = cv::createCLAHE(clahe_clip_limit_, clahe_tile_grid_size_);
        adaptive_thresh_block_size_ = std::max(15, (std::min(input_width_, input_height_) / 20) | 1);
        input_area_ = input_width_ * input_height_;
    }

    void infer(const cv::Mat &image, std::vector<cv::Rect> &bboxes)
    {
        auto start_time = std::chrono::system_clock::now();
        getRotatedRoiImage(image, image_cropped_);
        /*
        
        cv::imshow("Input Image", image);
        cv::imshow("Rotated ROI", image_cropped_);
        cv::waitKey(1);
        */

        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        RCLCPP_INFO(rclcpp::get_logger("CellDetector"), "Inference time: %.3f ms", elapsed_seconds.count() * 1000);
    }

private:
    void getRotatedRoiImage(const cv::Mat &image, cv::Mat &image_cropped)
    {
        CV_Assert(!image.empty());

        cv::cvtColor(image, gray_, cv::COLOR_RGB2GRAY);
        clahe_processor_->apply(gray_, clahe_);

        cv::GaussianBlur(
            clahe_,
            blur_,
            cv::Size(blur_size_, blur_size_),
            0);

        double best_score = -1.0;

        if(C_HOLDER_ != -1)
        {
            getScore(C_HOLDER_, best_score);
        }

        if(best_score < score_threshold_)
        {
            C_HOLDER_ = -1;
            best_score = -1.0;
            for (int C = 2; C < 12; ++C)
            {
                getScore(C, best_score);
            }
            RCLCPP_INFO(rclcpp::get_logger("CellDetector"), "Recalibrated C value: %d with score: %.3f", C_HOLDER_, best_score);
        }
        
        if(best_approx_.size() != 4)
        {
            RCLCPP_WARN(rclcpp::get_logger("CellDetector"), "Failed to detect a valid board contour. Best score: %.3f", best_score);
        }

        // === ORDER POINTS ===
        std::vector<cv::Point2f> src_pts = orderPoints(best_approx_);

        // === COMPUTE TARGET SIZE ===
        float widthA = cv::norm(src_pts[2] - src_pts[3]);
        float widthB = cv::norm(src_pts[1] - src_pts[0]);
        int maxWidth = static_cast<int>(std::max(widthA, widthB));

        float heightA = cv::norm(src_pts[1] - src_pts[2]);
        float heightB = cv::norm(src_pts[0] - src_pts[3]);
        int maxHeight = static_cast<int>(std::max(heightA, heightB));

        std::vector<cv::Point2f> dst_pts = {
            {0.f, 0.f},
            {static_cast<float>(maxWidth - 1), 0.f},
            {static_cast<float>(maxWidth - 1), static_cast<float>(maxHeight - 1)},
            {0.f, static_cast<float>(maxHeight - 1)}};

        // === WARP ===
        cv::Mat R = (cv::Mat_<double>(3, 3) << 0, -1, maxHeight,
                     1, 0, 0,
                     0, 0, 1);

        cv::Mat F = (cv::Mat_<double>(3, 3) << -1, 0, maxWidth,
                     0, 1, 0,
                     0, 0, 1);

        cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
        M = F * R * M;
        cv::warpPerspective(image, image_cropped, M, cv::Size(maxWidth, maxHeight));
    }

    std::vector<cv::Point2f> orderPoints(
        const std::vector<cv::Point> &pts)
    {
        CV_Assert(pts.size() == 4);

        std::vector<cv::Point2f> rect(4);
        std::vector<cv::Point2f> pts_f;

        for (const auto &p : pts)
            pts_f.emplace_back(p.x, p.y);

        // sum = x + y
        std::vector<float> sum(4);
        for (int i = 0; i < 4; ++i)
            sum[i] = pts_f[i].x + pts_f[i].y;

        rect[0] = pts_f[std::distance(sum.begin(),
                                      std::min_element(sum.begin(), sum.end()))]; // TL
        rect[2] = pts_f[std::distance(sum.begin(),
                                      std::max_element(sum.begin(), sum.end()))]; // BR

        // diff = x - y
        std::vector<float> diff(4);
        for (int i = 0; i < 4; ++i)
            diff[i] = pts_f[i].x - pts_f[i].y;

        rect[1] = pts_f[std::distance(diff.begin(),
                                      std::min_element(diff.begin(), diff.end()))]; // TR
        rect[3] = pts_f[std::distance(diff.begin(),
                                      std::max_element(diff.begin(), diff.end()))]; // BL

        return rect;
    }

    void getScore(int32_t C, double &best_score)
    {
        cv::adaptiveThreshold(
            blur_,
            thresh_,
            255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY,
            adaptive_thresh_block_size_,
            C);

        autoCanny(thresh_, edges_, auto_canny_sigma_);

        cv::dilate(edges_, edges_, kernel_, cv::Point(-1, -1), 2);
        cv::erode(edges_, edges_, kernel_, cv::Point(-1, -1), 1);

        contours_.clear();

        cv::findContours(
            edges_,
            contours_,
            cv::RETR_EXTERNAL,
            cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> approx;
        for (const auto &c : contours_)
        {
            double peri = cv::arcLength(c, true);
            cv::approxPolyDP(c, approx, 0.02 * peri, true);

            double score = scoreDetection(approx, c, input_area_, peri);

            if (score > best_score)
            {
                best_score = score;
                best_approx_ = approx;
                C_HOLDER_ = C;
            }
        }
    }

    void autoCanny(const cv::Mat &src, cv::Mat &dst, double sigma)
    {
        double v = computeMedian(src);

        int lower = static_cast<int>(std::max(0.0, (1.0 - sigma) * v));
        int upper = static_cast<int>(std::min(255.0, (1.0 + sigma) * v));

        cv::Canny(src, dst, lower, upper);
    }

    double computeMedian(const cv::Mat &gray)
    {
        CV_Assert(gray.type() == CV_8UC1);

        int hist[256] = {0};
        const uchar *data = gray.data;
        const size_t total = gray.total();

        for (size_t i = 0; i < total; ++i)
            hist[data[i]]++;

        const size_t mid = total / 2;
        size_t count = 0;
        for (int i = 0; i < 256; ++i)
        {
            count += hist[i];
            if (count >= mid)
                return static_cast<double>(i);
        }
        return 0.0;
    }

    double scoreDetection(
        const std::vector<cv::Point> &approx,
        const std::vector<cv::Point> &contour,
        int img_area,
        double contour_peri)
    {
        if (approx.size() != 4)
        {
            return 0.0;
        }

        double area = cv::contourArea(contour);
        double area_ratio = area / static_cast<double>(img_area);

        if (area_ratio <= 0.2 || area_ratio >= 0.9)
        {
            return 0.0;
        }

        if (contour_peri <= 1e-6)
        {
            return 0.0;
        }

        double approx_peri = cv::arcLength(approx, true);
        return approx_peri / contour_peri;
    }

    int32_t input_width_;
    int32_t input_height_;
    int32_t input_area_;

    double clahe_clip_limit_ = 2.0;
    cv::Size clahe_tile_grid_size_ = cv::Size(8, 8);

    int32_t blur_size_ = 3;

    int32_t adaptive_thresh_block_size_;

    int32_t C_HOLDER_ = -1;

    double auto_canny_sigma_ = 0.33;
    double score_threshold_ = 0.9;

    cv::Mat image_cropped_, gray_, clahe_, blur_, thresh_, edges_;
    cv::Mat kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Point> best_approx_;
    cv::Ptr<cv::CLAHE> clahe_processor_;
};

class CameraListener : public rclcpp::Node
{
public:
    CameraListener() : Node("camera_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bizon2/top_camera",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&CameraListener::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Camera listener node started");
    }

    CellDetector cell_detector{1280, 720};

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received image: %dx%d, encoding: %s",
        //            msg->width, msg->height, msg->encoding.c_str());

        // Convert ROS image to OpenCV (zero-copy)
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image.clone();
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        std::vector<cv::Rect> bboxes;
        cell_detector.infer(image, bboxes);

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraListener>());
    rclcpp::shutdown();
    return 0;
}
