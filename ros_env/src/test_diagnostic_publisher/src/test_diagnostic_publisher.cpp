#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

class TestDiagnosticPublisher : public rclcpp::Node
{
public:
  TestDiagnosticPublisher()
  : Node("test_diagnostic_publisher")
  {
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("test_cam1");
    diagnostic_updater_->add(
      "Test Camera Health", this,
      &TestDiagnosticPublisher::check_camera_status);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TestDiagnosticPublisher::publish_diagnostics, this));
  }

private:
  void check_camera_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (camera_ok) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera operating normally");
      stat.add("fps", 30);
      stat.add("temperature", "42C");
      stat.add("error_code", "NONE");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera failure detected");
      stat.add("fps", 10);
      stat.add("temperature", "85C");
      stat.add("error_code", "CAM_ERR_01");
    }
  }

  void publish_diagnostics()
  {
    diagnostic_updater_->force_update();
    camera_ok = !camera_ok;
  }

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool camera_ok = true;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestDiagnosticPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
