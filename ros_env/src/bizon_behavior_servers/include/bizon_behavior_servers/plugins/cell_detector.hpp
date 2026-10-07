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

  bool infer(
    const cv::Mat & image, cv::Mat & image_cropped, std::vector<cv::Rect> & bboxes,
    std::vector<cv::Point2f> & points_crop_)
  {
    auto start_time = std::chrono::system_clock::now();
    if (!getRotatedRoiImage(image, image_cropped, points_crop_)) {
      return false;
    }
    getSquareBboxes(image_cropped, bboxes);

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    return true;
  }

private:
  bool getRotatedRoiImage(
    const cv::Mat & image, cv::Mat & image_cropped,
    std::vector<cv::Point2f> & points_crop_)
  {
    CV_Assert(!image.empty());
    best_approx_.clear();

    cv::cvtColor(image, gray_, cv::COLOR_RGB2GRAY);
    clahe_processor_->apply(gray_, clahe_);

    cv::GaussianBlur(
      clahe_,
      blur_,
      cv::Size(blur_size_, blur_size_),
      0);

    double best_score = -1.0;

    if (C_HOLDER_ != -1) {
      getScore(C_HOLDER_, best_score);
    }

    if (best_score < score_threshold_) {
      C_HOLDER_ = -1;
      best_score = -1.0;
      for (int C = 2; C < 12; ++C) {
        getScore(C, best_score);
      }
      RCLCPP_INFO(
        rclcpp::get_logger(
          "CellDetector"), "Recalibrated C value: %d with score: %.3f", C_HOLDER_, best_score);
      if (best_score < score_threshold_) {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "CellDetector"), "Score %.3f below threshold %.3f after recalibration, retrying...", best_score,
          score_threshold_);
        return false;
      }
    }

    if (best_approx_.size() != 4) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "CellDetector"), "Failed to detect a valid board contour. Best score: %.3f", best_score);
      return false;
    }

    if (cv::contourArea(best_approx_) < threshold_area_px_) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "CellDetector"), "Detected contour area too small: %.3f px^2",
        cv::contourArea(best_approx_));
      return false;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "CellDetector"), "Best contour area: %.3f px^2 with score: %.3f", cv::contourArea(
        best_approx_), best_score);

    // === ORDER POINTS ===
    points_crop_ = orderPoints(best_approx_);

    // === COMPUTE TARGET SIZE ===
    float widthA = cv::norm(points_crop_[2] - points_crop_[3]);
    float widthB = cv::norm(points_crop_[1] - points_crop_[0]);
    int maxWidth = static_cast<int>(std::max(widthA, widthB));

    float heightA = cv::norm(points_crop_[1] - points_crop_[2]);
    float heightB = cv::norm(points_crop_[0] - points_crop_[3]);
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

    cv::Mat M = cv::getPerspectiveTransform(points_crop_, dst_pts);
    M = F * R * M;
    cv::warpPerspective(image, image_cropped, M, cv::Size(maxWidth, maxHeight));
    if (image_cropped.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("CellDetector"), "Warp produced empty image");
      return false;
    }
    return true;
  }

  std::vector<cv::Point2f> orderPoints(
    const std::vector<cv::Point> & pts)
  {
    CV_Assert(pts.size() == 4);

    std::vector<cv::Point2f> rect(4);
    std::vector<cv::Point2f> pts_f;

    for (const auto & p : pts) {
      pts_f.emplace_back(p.x, p.y);
    }

    // sum = x + y
    std::vector<float> sum(4);
    for (int i = 0; i < 4; ++i) {
      sum[i] = pts_f[i].x + pts_f[i].y;
    }

    rect[0] = pts_f[std::distance(
          sum.begin(),
          std::min_element(sum.begin(), sum.end()))];                             // TL
    rect[2] = pts_f[std::distance(
          sum.begin(),
          std::max_element(sum.begin(), sum.end()))];                             // BR

    // diff = x - y
    std::vector<float> diff(4);
    for (int i = 0; i < 4; ++i) {
      diff[i] = pts_f[i].x - pts_f[i].y;
    }

    rect[1] = pts_f[std::distance(
          diff.begin(),
          std::min_element(diff.begin(), diff.end()))];                             // TR
    rect[3] = pts_f[std::distance(
          diff.begin(),
          std::max_element(diff.begin(), diff.end()))];                             // BL

    return rect;
  }

  void getScore(int32_t C, double & best_score)
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
    for (const auto & c : contours_) {
      double peri = cv::arcLength(c, true);
      cv::approxPolyDP(c, approx, 0.02 * peri, true);

      double score = scoreDetection(approx, c, input_area_, peri);

      if (score > 0.0 && score > best_score) {
        best_score = score;
        best_approx_ = approx;
        C_HOLDER_ = C;
      }
    }
  }

  void autoCanny(const cv::Mat & src, cv::Mat & dst, double sigma)
  {
    double v = computeMedian(src);

    int lower = static_cast<int>(std::max(0.0, (1.0 - sigma) * v));
    int upper = static_cast<int>(std::min(255.0, (1.0 + sigma) * v));

    cv::Canny(src, dst, lower, upper);
  }

  double computeMedian(const cv::Mat & gray)
  {
    CV_Assert(gray.type() == CV_8UC1);

    int hist[256] = {0};
    const uchar * data = gray.data;
    const size_t total = gray.total();

    for (size_t i = 0; i < total; ++i) {
      hist[data[i]]++;
    }

    const size_t mid = total / 2;
    size_t count = 0;
    for (int i = 0; i < 256; ++i) {
      count += hist[i];
      if (count >= mid) {
        return static_cast<double>(i);
      }
    }
    return 0.0;
  }

  double scoreDetection(
    const std::vector<cv::Point> & approx,
    const std::vector<cv::Point> & contour,
    int img_area,
    double contour_peri)
  {
    if (approx.size() != 4) {
      return 0.0;
    }

    double area = cv::contourArea(contour);
    double area_ratio = area / static_cast<double>(img_area);

    if (area_ratio <= 0.2 || area_ratio >= 0.9) {
      return 0.0;
    }

    if (contour_peri <= 1e-6) {
      return 0.0;
    }

    double approx_peri = cv::arcLength(approx, true);
    return approx_peri / contour_peri;
  }

  void getSquareBboxes(
    const cv::Mat & src, std::vector<cv::Rect> & bboxes,
    float border_ratio = 0.08f, float inner_crop_ratio = 0.01f)
  {
    int height = src.rows;
    int width = src.cols;

    int bx = static_cast<int>(width * border_ratio);
    int by = static_cast<int>(height * border_ratio);

    int inner_width = width - 2 * bx;
    int inner_height = height - 2 * by;

    float square_width = inner_width / 8.0f;
    float square_height = inner_height / 8.0f;

    for (int row = 0; row < 8; ++row) {
      for (int col = 0; col < 8; ++col) {
        int x1 = static_cast<int>(bx + col * square_width);
        int y1 = static_cast<int>(by + row * square_height);
        int x2 = static_cast<int>(bx + (col + 1) * square_width);
        int y2 = static_cast<int>(by + (row + 1) * square_height);

        float dx = (x2 - x1) * inner_crop_ratio;
        float dy = (y2 - y1) * inner_crop_ratio;

        x1 += static_cast<int>(dx);
        y1 += static_cast<int>(dy);
        x2 -= static_cast<int>(dx);
        y2 -= static_cast<int>(dy);

        bboxes.emplace_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
      }
    }
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
  double threshold_area_px_ = 380000.0;   // Minimum area in pixels for a valid contour
};
