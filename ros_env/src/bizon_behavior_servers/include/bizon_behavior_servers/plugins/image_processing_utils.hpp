#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <algorithm>


namespace bizon_behaviors
{

void getImageGray(const cv::Mat & src, cv::Mat & dst)
{
  cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
}

void getImageBlur(cv::Mat & src, cv::Mat & dst, int kernel_size, int sigma)
{
  cv::GaussianBlur(src, dst, cv::Size(kernel_size, kernel_size), sigma, cv::BORDER_DEFAULT);
}

void getImageOtsu(cv::Mat & src, cv::Mat & dst)
{
  cv::threshold(src, dst, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
}

void getImageCanny(cv::Mat & src, cv::Mat & dst, int low_threshold, int high_threshold, bool dilate)
{
  cv::Canny(src, dst, low_threshold, high_threshold);
  if (dilate) {
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(dst, dst, element, cv::Point(-1, -1), 1);
  }
}


bool compareXAxis(cv::Point2f a, cv::Point2f b)
{
  return a.x < b.x;
}

bool compareYAxis(cv::Point2f a, cv::Point2f b)
{
  return a.y < b.y;
}


void getBiggerContour(
  cv::Mat & src, cv::Point2f (& src_points)[4], int & max_area, int mode,
  int method)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(src.clone(), contours, mode, method);
  max_area = 0;
  std::vector<cv::Point> selected_approx;

  for (auto contour : contours) {
    int area = cv::contourArea(contour);
    if (area > 100) {
      double peri = cv::arcLength(contour, true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contour, approx, (double)0.02 * peri, true);
      if (area > max_area && approx.size() == 4) {
        max_area = area;
        selected_approx = approx;
      }

    }

  }

  std::sort(selected_approx.begin(), selected_approx.end(), compareXAxis);
  auto left_sides = std::vector<cv::Point>(selected_approx.begin(), selected_approx.begin() + 2);
  auto right_sides = std::vector<cv::Point>(selected_approx.end() - 2, selected_approx.end());

  std::sort(left_sides.begin(), left_sides.end(), compareYAxis);
  std::sort(right_sides.begin(), right_sides.end(), compareYAxis);

  src_points[0] = left_sides[0];
  src_points[1] = left_sides[1];
  src_points[2] = right_sides[1];
  src_points[3] = right_sides[0];

  //std::cout << "Left sides: " << left_sides[0] << " " << left_sides[1] << std::endl;
  //std::cout << "Right sides: " << right_sides[0] << " " << right_sides[1] << std::endl;


}

cv::Mat cropAndWarp(
  const cv::Mat & src, cv::Mat & dst, const cv::Point2f (& src_points)[4],
  const cv::Size size = cv::Size(480, 480), const int margin = 30,
  const int border = 420, const bool rotate = true)
{
  cv::Point2f dst_points[4];
  dst_points[0] = cv::Point2f(0, 0);
  dst_points[1] = cv::Point2f(0, size.height);
  dst_points[2] = cv::Point2f(size.width, size.height);
  dst_points[3] = cv::Point2f(size.width, 0);

  cv::Mat matrix = cv::getPerspectiveTransform(src_points, dst_points);
  cv::warpPerspective(src, dst, matrix, size);

  double angle =
    atan2(
    (src_points[1].y - src_points[0].y),
    (src_points[1].x - src_points[0].x)) * 180 / 3.14159265358979323846;

  //std::cout << "Angle: " << angle << std::endl;
  if (angle > 135 && rotate && abs(angle - 180) > 5) {
    //cv::rotate(dst, dst, cv::ROTATE_90_CLOCKWISE);
  }

  dst = dst(cv::Rect(margin, margin, border, border));
  return matrix;
}

void getImageClahe(
  const cv::Mat & src, cv::Mat & dst, const double clip_limit,
  const int tile_grid_size)
{
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clip_limit, cv::Size(tile_grid_size, tile_grid_size));
  clahe->apply(src, dst);
}


bool isNewLine(const cv::Vec2f & line, const std::vector<cv::Vec2f> & lines)
{
  for (auto l : lines) {
    if (sqrt(pow(l[0] - line[0], 2) + pow(l[1] - line[1], 2)) < 10) {
      return false;
    }
  }
  return true;
}

std::vector<cv::Point2f> getIntersections(
  const std::vector<cv::Vec2f> & vertical_lines,
  const std::vector<cv::Vec2f> & horizontal_lines)
{

  std::vector<cv::Point2f> intersections;
  for (int i = 0; i < vertical_lines.size(); i++) {
    for (int j = 0; j < horizontal_lines.size(); j++) {
      float rho1 = vertical_lines[i][0];
      float theta1 = vertical_lines[i][1];
      float rho2 = horizontal_lines[j][0];
      float theta2 = horizontal_lines[j][1];
      double a1 = cos(theta1), b1 = sin(theta1);
      double a2 = cos(theta2), b2 = sin(theta2);
      double x0 = a1 * rho1, y0 = b1 * rho1;
      double x1 = a2 * rho2, y1 = b2 * rho2;
      double det = a1 * b2 - a2 * b1;
      if (det != 0) {
        double x = (b2 * x0 - b1 * x1) / det;
        double y = (a1 * y1 - a2 * y0) / det;
        intersections.push_back(cv::Point2f(x, y));
      }
    }
  }
  return intersections;
}

void getImageHoughlines(
  const cv::Mat & src, cv::Mat & dst,
  std::vector<cv::Point2f> & intersections, const int rho, const double theta,
  const int threshold, const double minLineLength, const double maxLineGap)
{
  dst = src.clone();
  cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(
    src,
    lines,
    0.1,
    CV_PI / 180.0,
    threshold
  );

  std::vector<cv::Vec2f> vertical_lines;
  std::vector<cv::Vec2f> horizontal_lines;

  for (auto line : lines) {
    float rho = line[0];
    float theta = line[1];
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
    cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));

    if (theta == 0.0 && isNewLine(line, vertical_lines)) {
      vertical_lines.push_back(line);
      cv::line(dst, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    if (theta != 0.0 && isNewLine(line, horizontal_lines)) {
      horizontal_lines.push_back(line);
      cv::line(dst, pt1, pt2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
  }

  //sort vertical lines
  std::sort(
    vertical_lines.begin(), vertical_lines.end(), [](cv::Vec2f a, cv::Vec2f b) {
      return a[0] < b[0];
    });

  //sort horizontal lines
  std::sort(
    horizontal_lines.begin(), horizontal_lines.end(), [](cv::Vec2f a, cv::Vec2f b) {
      return a[0] < b[0];
    });

  intersections = getIntersections(vertical_lines, horizontal_lines);

}

}  // namespace bizon_behaviors
