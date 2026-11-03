// Publishes the joint_bench firmware's USB stream as sensor_msgs/Imu on imu/data_raw.

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "bizon_imu/stream_line.hpp"

namespace
{
using std::chrono::milliseconds;
using std::chrono::steady_clock;

constexpr double kGyroLsbPerDps = 65.5;  // the firmware sets +-500 dps, see GYRO_CONFIG in mpu.cpp
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kGravity = 9.80665;
constexpr int kCalibrationSamples = 2000;  // 2 s at the firmware's 1 kHz
// 2 dps. A module at rest peaks near 0.3 dps, so a wider spread means it is being moved, and a
// bias averaged over that motion would make the orientation drift from the first second.
constexpr int kStillRangeLsb = 131;

// A raw serial port that hands out whole lines. Blocking I/O with a poll timeout, so the caller
// can notice shutdown; a USB unplug surfaces as an exception and ends the node.
class SerialLines
{
public:
  explicit SerialLines(const std::string & path)
  : fd_(::open(path.c_str(), O_RDWR | O_NOCTTY))
  {
    if (fd_ < 0) {
      throw std::runtime_error(path + ": " + std::strerror(errno));
    }
    // TIOCEXCL makes a second reader, such as `joint_bench record`, fail at open. Sharing the
    // port would split the lines between them and both would see gaps.
    ::ioctl(fd_, TIOCEXCL);
    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      ::close(fd_);
      throw std::runtime_error(path + ": not a serial port");
    }
    ::cfmakeraw(&tty);
    ::tcsetattr(fd_, TCSANOW, &tty);
    ::tcflush(fd_, TCIOFLUSH);
  }

  ~SerialLines() {::close(fd_);}
  SerialLines(const SerialLines &) = delete;
  SerialLines & operator=(const SerialLines &) = delete;

  void send(const std::string & command)
  {
    const std::string line = command + "\n";
    if (::write(fd_, line.data(), line.size()) != static_cast<ssize_t>(line.size())) {
      throw std::runtime_error(std::string("serial write: ") + std::strerror(errno));
    }
  }

  // The next complete line, or nullopt if none arrived within the timeout.
  std::optional<std::string> next_line(milliseconds timeout)
  {
    const auto deadline = steady_clock::now() + timeout;
    for (;;) {
      const auto newline = buffer_.find('\n');
      if (newline != std::string::npos) {
        std::string line = buffer_.substr(0, newline);
        buffer_.erase(0, newline + 1);
        return line;
      }
      const auto left = std::chrono::duration_cast<milliseconds>(deadline - steady_clock::now());
      if (left.count() <= 0) {
        return std::nullopt;
      }
      pollfd pfd{fd_, POLLIN, 0};
      const int ready = ::poll(&pfd, 1, static_cast<int>(left.count()));
      if (ready < 0 && errno != EINTR) {
        throw std::runtime_error(std::string("serial poll: ") + std::strerror(errno));
      }
      if (ready <= 0) {
        continue;
      }
      char chunk[4096];
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n < 0 && errno != EINTR) {
        throw std::runtime_error(std::string("serial read: ") + std::strerror(errno));
      }
      if (n == 0) {
        throw std::runtime_error("serial read: port closed");
      }
      if (n > 0) {
        buffer_.append(chunk, static_cast<std::size_t>(n));
      }
    }
  }

  void drain(milliseconds quiet)
  {
    while (next_line(quiet)) {
    }
    buffer_.clear();
  }

private:
  int fd_;
  std::string buffer_;
};

// The gyro's zero-rate offset, measured while the module is still. Without a magnetometer no
// filter can observe yaw error, so an uncorrected offset becomes a steady yaw drift: the GY-521
// on this bench reads about 0.26 dps on Z at rest, 15 degrees a minute.
class GyroBias
{
public:
  // True once the bias is known. Motion restarts the measurement.
  bool add(const std::array<int, 3> & gyro)
  {
    if (count_ >= kCalibrationSamples) {
      return true;
    }
    if (count_ == 0) {
      min_ = max_ = gyro;
      sum_ = {0.0, 0.0, 0.0};
    }
    for (std::size_t i = 0; i < 3; ++i) {
      min_[i] = std::min(min_[i], gyro[i]);
      max_[i] = std::max(max_[i], gyro[i]);
      sum_[i] += gyro[i];
      if (max_[i] - min_[i] > kStillRangeLsb) {
        count_ = 0;
        ++restarts_;
        return false;
      }
    }
    ++count_;
    return count_ >= kCalibrationSamples;
  }

  double operator[](std::size_t axis) const {return sum_[axis] / count_;}
  int restarts() const {return restarts_;}

private:
  int count_{0};
  int restarts_{0};
  std::array<int, 3> min_{};
  std::array<int, 3> max_{};
  std::array<double, 3> sum_{};
};

class SerialImuNode : public rclcpp::Node
{
public:
  SerialImuNode()
  : Node("serial_imu_node"),
    publisher_(create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10)))
  {
    declare_parameter("port", "/dev/ttyACM0");
  }

  void run()
  {
    SerialLines port(get_parameter("port").as_string());

    // A previous run that died without saying goodbye leaves the stream on, and its lines would
    // bury the replies to the commands below.
    port.send("stream off");
    port.drain(milliseconds(200));

    port.send("imu info");
    const int lsb_per_g = read_lsb_per_g(port);

    port.send("stream on");
    RCLCPP_INFO(get_logger(), "Measuring gyro bias, keep the IMU still");
    while (rclcpp::ok()) {
      const auto line = port.next_line(milliseconds(1000));
      if (!line) {
        throw std::runtime_error("no samples for 1 s");
      }
      if (const auto sample = bizon_imu::parse_sample(*line)) {
        handle(*sample, lsb_per_g);
      }
    }
    port.send("stream off");
  }

private:
  int read_lsb_per_g(SerialLines & port)
  {
    for (int i = 0; i < 10; ++i) {
      const auto reply = port.next_line(milliseconds(1000));
      if (!reply) {
        break;
      }
      if (reply->rfind("ERR", 0) == 0) {
        throw std::runtime_error("imu info: " + *reply);
      }
      if (const auto lsb = bizon_imu::parse_lsb_per_g(*reply)) {
        RCLCPP_INFO(get_logger(), "%s", reply->c_str());
        return *lsb;
      }
    }
    throw std::runtime_error("imu info: no usable reply");
  }

  void handle(const bizon_imu::Sample & s, int lsb_per_g)
  {
    if (last_seq_ && s.seq != *last_seq_ + 1) {
      ++missed_runs_;
      RCLCPP_WARN(
        get_logger(), "Sample gap: %u after %u (%d so far)", s.seq, *last_seq_, missed_runs_);
    }
    last_seq_ = s.seq;

    const int restarts_before = bias_.restarts();
    const bool was_ready = bias_ready_;
    bias_ready_ = bias_.add(s.gyro);
    if (bias_.restarts() != restarts_before) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "IMU is moving, restarting the gyro bias measurement");
    }
    if (!bias_ready_) {
      return;
    }
    if (!was_ready) {
      RCLCPP_INFO(
        get_logger(), "Gyro bias %.1f %.1f %.1f counts, publishing", bias_[0], bias_[1], bias_[2]);
    }

    // now(), not the firmware's clock: the two run at slightly different rates, and TF rejects a
    // stamp that has drifted away from the ROS clock. The filter takes its dt from constant_dt,
    // so USB's batching of samples into one read does not reach the estimate.
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = "imu_link";
    msg.orientation_covariance[0] = -1.0;  // REP 145: this message carries no orientation
    const double accel_scale = kGravity / lsb_per_g;
    msg.linear_acceleration.x = s.accel[0] * accel_scale;
    msg.linear_acceleration.y = s.accel[1] * accel_scale;
    msg.linear_acceleration.z = s.accel[2] * accel_scale;
    const double gyro_scale = kDegToRad / kGyroLsbPerDps;
    msg.angular_velocity.x = (s.gyro[0] - bias_[0]) * gyro_scale;
    msg.angular_velocity.y = (s.gyro[1] - bias_[1]) * gyro_scale;
    msg.angular_velocity.z = (s.gyro[2] - bias_[2]) * gyro_scale;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  GyroBias bias_;
  bool bias_ready_{false};
  std::optional<uint32_t> last_seq_;
  int missed_runs_{0};
};
}  // namespace

// No executor: the node only publishes, and a sample's arrival is the only thing it waits for.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  auto node = std::make_shared<SerialImuNode>();
  try {
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
