#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace
{
constexpr double kEps = 1e-6;

bool parseDouble(const std::string & text, double & out)
{
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  out = value;
  return true;
}

bool parseInt64(const std::string & text, int64_t & out)
{
  char * end = nullptr;
  const long long value = std::strtoll(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  out = static_cast<int64_t>(value);
  return true;
}

speed_t baudToConstant(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

std::string trimLine(std::string line)
{
  while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }
  return line;
}
}  // namespace

class MotorSerialBridgeNode : public rclcpp::Node
{
public:
  MotorSerialBridgeNode()
  : Node("motor_serial_bridge_node"), running_(true)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyTHS1");
    baud_ = declare_parameter<int>("baud", 115200);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 75.0);
    cmd_timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", 200);

    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

    wheel_base_m_ = declare_parameter<double>("wheel_base_m", 0.33);
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.05);
    ticks_per_rev_ = declare_parameter<int64_t>("ticks_per_rev", 2048);

    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 5.0);
    max_steer_rad_ = declare_parameter<double>("max_steer_rad", 0.45);

    (void)declare_parameter<std::string>("drive_model", "ackermann");
    command_mode_ = declare_parameter<std::string>("command_mode", "mps_rad");

    pwm_min_ = declare_parameter<int>("pwm_min", 1000);
    pwm_max_ = declare_parameter<int>("pwm_max", 2000);
    steer_center_us_ = declare_parameter<int>("steer_center_us", 1500);
    steer_left_us_ = declare_parameter<int>("steer_left_us", 1900);
    steer_right_us_ = declare_parameter<int>("steer_right_us", 1100);

    invert_drive_ = declare_parameter<bool>("invert_drive", false);
    use_measured_steer_ = declare_parameter<bool>("use_measured_steer", false);
    debug_print_ = declare_parameter<bool>("debug_print", false);

    ackermann_topic_ = declare_parameter<std::string>("ackermann_topic", "/ackermann_cmd");
    enable_cmd_vel_ = declare_parameter<bool>("enable_cmd_vel", false);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    emergency_stop_topic_ =
      declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
    allow_estop_release_ = declare_parameter<bool>("allow_estop_release", true);

    publish_odom_ = declare_parameter<bool>("publish_odom", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);
    publish_wheel_ticks_ = declare_parameter<bool>("publish_wheel_ticks", true);
    publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);

    drive_wheel_joint_name_ =
      declare_parameter<std::string>("drive_wheel_joint_name", "drive_wheel_joint");
    steering_joint_name_ = declare_parameter<std::string>("steering_joint_name", "steering_joint");

    reconnect_period_ms_ = declare_parameter<int>("reconnect_period_ms", 1000);

    if (command_mode_ != "mps_rad" && command_mode_ != "pwm_servo_us") {
      RCLCPP_WARN(get_logger(), "Unknown command_mode='%s'. Falling back to mps_rad.",
        command_mode_.c_str());
      command_mode_ = "mps_rad";
    }

    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    }
    if (publish_joint_states_) {
      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);
    }
    if (publish_wheel_ticks_) {
      wheel_ticks_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>("/wheel_ticks", 20);
    }
    if (publish_diagnostics_) {
      diagnostics_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    }

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    ackermann_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      ackermann_topic_, 20,
      std::bind(&MotorSerialBridgeNode::onAckermannCmd, this, std::placeholders::_1));

    if (enable_cmd_vel_) {
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 20,
        std::bind(&MotorSerialBridgeNode::onCmdVel, this, std::placeholders::_1));
    }

    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, 10,
      std::bind(&MotorSerialBridgeNode::onEmergencyStop, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MotorSerialBridgeNode::onControlTimer, this));

    read_thread_ = std::thread(&MotorSerialBridgeNode::readLoop, this);

    RCLCPP_INFO(get_logger(), "motor_serial_bridge_node started. port=%s baud=%d mode=%s",
      port_.c_str(), baud_, command_mode_.c_str());
  }

  ~MotorSerialBridgeNode() override
  {
    running_.store(false);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    closeSerial();
  }

private:
  struct Feedback
  {
    rclcpp::Time stamp;
    bool has_enc = false;
    bool has_enc_l = false;
    bool has_enc_r = false;
    bool has_vel_mps = false;
    bool has_vel_radps = false;
    bool has_steer_rad = false;
    bool has_batt = false;
    bool has_fault = false;

    int64_t enc = 0;
    int64_t enc_l = 0;
    int64_t enc_r = 0;
    double vel_mps = 0.0;
    double vel_radps = 0.0;
    double steer_rad = 0.0;
    double batt = 0.0;
    int64_t fault = 0;
  };

  void onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    setCommand(msg->drive.speed, msg->drive.steering_angle);
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double v = msg->linear.x;
    double steer = 0.0;
    if (std::abs(v) > 1e-3) {
      steer = std::atan(wheel_base_m_ * msg->angular.z / std::max(std::abs(v), 1e-3));
    }
    setCommand(v, steer);
  }

  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (msg->data) {
      estop_latched_ = true;
    } else if (allow_estop_release_) {
      estop_latched_ = false;
    }
  }

  void setCommand(double speed_mps, double steer_rad)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_speed_mps_ = std::clamp(speed_mps, -max_speed_mps_, max_speed_mps_);
    cmd_steer_rad_ = std::clamp(steer_rad, -max_steer_rad_, max_steer_rad_);
    last_cmd_time_ = now();
    has_cmd_ = true;
  }

  void onControlTimer()
  {
    ensureSerialConnected();

    double speed_cmd = 0.0;
    double steer_cmd = 0.0;
    bool estop = false;

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      const bool timed_out =
        (!has_cmd_) || ((now() - last_cmd_time_).nanoseconds() / 1000000LL > cmd_timeout_ms_);
      estop = estop_latched_;

      if (!timed_out && !estop) {
        speed_cmd = cmd_speed_mps_;
        steer_cmd = cmd_steer_rad_;
      }

      last_steer_cmd_rad_ = steer_cmd;
    }

    const std::string cmd_line = buildCommandLine(speed_cmd, steer_cmd, estop);
    if (!cmd_line.empty()) {
      (void)sendCommandLine(cmd_line);
    }

    updateAndPublishState();
    publishDiagnostics();
  }

  std::string buildCommandLine(double speed_mps, double steer_rad, bool estop)
  {
    if (invert_drive_) {
      speed_mps = -speed_mps;
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);

    if (command_mode_ == "mps_rad") {
      ss << "CMD spd=" << speed_mps
         << " steer=" << std::clamp(steer_rad, -max_steer_rad_, max_steer_rad_)
         << " estop=" << (estop ? 1 : 0) << "\n";
      return ss.str();
    }

    const int pwm = speedToPwm(speed_mps);
    const int steer_us = steerToServoUs(steer_rad);
    std::ostringstream ss2;
    ss2 << "CMD pwm=" << pwm << " steer_us=" << steer_us << " estop=" << (estop ? 1 : 0)
        << "\n";
    return ss2.str();
  }

  int speedToPwm(double speed_mps) const
  {
    const double limited = std::clamp(speed_mps, -max_speed_mps_, max_speed_mps_);
    const double span = static_cast<double>(pwm_max_ - pwm_min_);
    const double norm = (limited / std::max(max_speed_mps_, kEps) + 1.0) * 0.5;
    return static_cast<int>(std::round(pwm_min_ + span * std::clamp(norm, 0.0, 1.0)));
  }

  int steerToServoUs(double steer_rad) const
  {
    const double limited = std::clamp(steer_rad, -max_steer_rad_, max_steer_rad_);
    if (limited >= 0.0) {
      const double t = limited / std::max(max_steer_rad_, kEps);
      return static_cast<int>(std::round(steer_center_us_ + t * (steer_left_us_ - steer_center_us_)));
    }
    const double t = (-limited) / std::max(max_steer_rad_, kEps);
    return static_cast<int>(std::round(steer_center_us_ + t * (steer_right_us_ - steer_center_us_)));
  }

  bool ensureSerialConnected()
  {
    if (serial_connected_.load()) {
      return true;
    }

    const auto t_now = now();
    if ((t_now - last_connect_attempt_).nanoseconds() / 1000000LL < reconnect_period_ms_) {
      return false;
    }
    last_connect_attempt_ = t_now;

    const int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to open %s: %s",
        port_.c_str(), std::strerror(errno));
      return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    cfmakeraw(&tty);
    const speed_t baud_const = baudToConstant(baud_);
    if (cfsetispeed(&tty, baud_const) != 0 || cfsetospeed(&tty, baud_const) != 0) {
      RCLCPP_WARN(get_logger(), "cfsetispeed/cfsetospeed failed: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_fd_ = fd;
      rx_buffer_.clear();
    }
    serial_connected_.store(true);
    RCLCPP_INFO(get_logger(), "Serial connected: %s @ %d", port_.c_str(), baud_);
    return true;
  }

  void closeSerial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    serial_connected_.store(false);
  }

  bool sendCommandLine(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ < 0) {
      return false;
    }

    const ssize_t sent = write(serial_fd_, line.data(), line.size());
    if (sent < 0 || static_cast<size_t>(sent) != line.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial write failed: %s",
        std::strerror(errno));
      if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
      }
      serial_connected_.store(false);
      return false;
    }

    if (debug_print_) {
      RCLCPP_DEBUG(get_logger(), "TX: %s", trimLine(line).c_str());
    }
    return true;
  }

  void readLoop()
  {
    std::vector<char> buffer(256);
    while (running_.load()) {
      if (!ensureSerialConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      int fd = -1;
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        fd = serial_fd_;
      }

      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      const ssize_t n = read(fd, buffer.data(), buffer.size());
      if (n > 0) {
        appendRxData(std::string(buffer.data(), static_cast<size_t>(n)));
      } else if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          continue;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read failed: %s",
          std::strerror(errno));
        closeSerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void appendRxData(const std::string & chunk)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    rx_buffer_.append(chunk);

    size_t newline_pos = std::string::npos;
    while ((newline_pos = rx_buffer_.find('\n')) != std::string::npos) {
      std::string line = rx_buffer_.substr(0, newline_pos);
      rx_buffer_.erase(0, newline_pos + 1);
      line = trimLine(line);
      if (!line.empty()) {
        parseFeedbackLine(line);
      }
    }

    if (rx_buffer_.size() > 4096) {
      rx_buffer_.clear();
    }
  }

  void parseFeedbackLine(const std::string & line)
  {
    if (debug_print_) {
      RCLCPP_DEBUG(get_logger(), "RX: %s", line.c_str());
    }

    if (line.rfind("FB", 0) != 0) {
      return;
    }

    Feedback feedback;
    feedback.stamp = now();

    std::istringstream ss(line);
    std::string token;
    ss >> token;  // FB

    while (ss >> token) {
      const size_t eq = token.find('=');
      if (eq == std::string::npos || eq == 0 || eq + 1 >= token.size()) {
        continue;
      }

      const std::string key = token.substr(0, eq);
      const std::string value = token.substr(eq + 1);

      int64_t int_val = 0;
      double double_val = 0.0;

      if ((key == "enc") && parseInt64(value, int_val)) {
        feedback.enc = int_val;
        feedback.has_enc = true;
      } else if ((key == "encL") && parseInt64(value, int_val)) {
        feedback.enc_l = int_val;
        feedback.has_enc_l = true;
      } else if ((key == "encR") && parseInt64(value, int_val)) {
        feedback.enc_r = int_val;
        feedback.has_enc_r = true;
      } else if ((key == "vel_mps") && parseDouble(value, double_val)) {
        feedback.vel_mps = double_val;
        feedback.has_vel_mps = true;
      } else if ((key == "vel") && parseDouble(value, double_val)) {
        feedback.vel_radps = double_val;
        feedback.has_vel_radps = true;
      } else if ((key == "steer") && parseDouble(value, double_val)) {
        feedback.steer_rad = std::clamp(double_val, -max_steer_rad_, max_steer_rad_);
        feedback.has_steer_rad = true;
      } else if ((key == "batt") && parseDouble(value, double_val)) {
        feedback.batt = double_val;
        feedback.has_batt = true;
      } else if ((key == "fault") && parseInt64(value, int_val)) {
        feedback.fault = int_val;
        feedback.has_fault = true;
      }
    }

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    latest_feedback_ = feedback;
  }

  std::optional<double> computeVelocityMps(const Feedback & fb, double dt)
  {
    if (fb.has_vel_mps) {
      return fb.vel_mps;
    }

    if (fb.has_vel_radps) {
      return fb.vel_radps * wheel_radius_m_;
    }

    if (dt <= kEps || ticks_per_rev_ <= 0) {
      return std::nullopt;
    }

    if (fb.has_enc_l && fb.has_enc_r) {
      if (!have_prev_ticks_lr_) {
        prev_enc_l_ = fb.enc_l;
        prev_enc_r_ = fb.enc_r;
        have_prev_ticks_lr_ = true;
        return std::nullopt;
      }

      const int64_t d_l = fb.enc_l - prev_enc_l_;
      const int64_t d_r = fb.enc_r - prev_enc_r_;
      prev_enc_l_ = fb.enc_l;
      prev_enc_r_ = fb.enc_r;

      const double d_avg_ticks = 0.5 * static_cast<double>(d_l + d_r);
      const double d_rev = d_avg_ticks / static_cast<double>(ticks_per_rev_);
      const double ds = d_rev * (2.0 * M_PI * wheel_radius_m_);
      return ds / dt;
    }

    if (fb.has_enc) {
      if (!have_prev_ticks_single_) {
        prev_enc_ = fb.enc;
        have_prev_ticks_single_ = true;
        return std::nullopt;
      }

      const int64_t d = fb.enc - prev_enc_;
      prev_enc_ = fb.enc;
      const double d_rev = static_cast<double>(d) / static_cast<double>(ticks_per_rev_);
      const double ds = d_rev * (2.0 * M_PI * wheel_radius_m_);
      return ds / dt;
    }

    return std::nullopt;
  }

  void updateAndPublishState()
  {
    const rclcpp::Time t_now = now();

    if (last_odom_time_.nanoseconds() == 0) {
      last_odom_time_ = t_now;
      return;
    }

    const double dt = (t_now - last_odom_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_odom_time_ = t_now;

    Feedback fb;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      fb = latest_feedback_;
    }

    const std::optional<double> v_opt = computeVelocityMps(fb, dt);
    const double v_mps = v_opt.has_value() ? *v_opt : 0.0;

    double steer_rad = 0.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      steer_rad = last_steer_cmd_rad_;
    }
    if (use_measured_steer_ && fb.has_steer_rad) {
      steer_rad = fb.steer_rad;
    }

    const double omega =
      (std::abs(wheel_base_m_) > kEps) ? (v_mps / wheel_base_m_) * std::tan(steer_rad) : 0.0;

    x_ += v_mps * std::cos(yaw_) * dt;
    y_ += v_mps * std::sin(yaw_) * dt;
    yaw_ += omega * dt;

    if (publish_odom_ && odom_pub_) {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = t_now;
      odom_msg.header.frame_id = odom_frame_;
      odom_msg.child_frame_id = base_frame_;
      odom_msg.pose.pose.position.x = x_;
      odom_msg.pose.pose.position.y = y_;
      odom_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw_);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      odom_msg.twist.twist.linear.x = v_mps;
      odom_msg.twist.twist.angular.z = omega;
      odom_pub_->publish(odom_msg);

      if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = odom_msg.header;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
      }
    }

    if (publish_joint_states_ && joint_state_pub_) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = t_now;
      js.name = {drive_wheel_joint_name_, steering_joint_name_};
      js.position.resize(2, 0.0);
      js.velocity.resize(2, 0.0);

      if (std::abs(wheel_radius_m_) > kEps) {
        drive_wheel_pos_rad_ += (v_mps / wheel_radius_m_) * dt;
        js.velocity[0] = v_mps / wheel_radius_m_;
      }
      js.position[0] = drive_wheel_pos_rad_;
      js.position[1] = steer_rad;
      js.velocity[1] = 0.0;

      joint_state_pub_->publish(js);
    }

    if (publish_wheel_ticks_ && wheel_ticks_pub_) {
      std_msgs::msg::Int64MultiArray ticks_msg;
      if (fb.has_enc_l && fb.has_enc_r) {
        ticks_msg.data = {fb.enc_l, fb.enc_r};
      } else if (fb.has_enc) {
        ticks_msg.data = {fb.enc};
      }
      if (!ticks_msg.data.empty()) {
        wheel_ticks_pub_->publish(ticks_msg);
      }
    }
  }

  void publishDiagnostics()
  {
    if (!publish_diagnostics_ || !diagnostics_pub_) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "motor_serial_bridge";
    status.hardware_id = port_;

    Feedback fb;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      fb = latest_feedback_;
    }

    if (!serial_connected_.load()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "serial_disconnected";
    } else if (fb.has_fault && fb.fault != 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "controller_fault";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
    }

    auto add_kv = [&status](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      status.values.push_back(kv);
    };

    add_kv("connected", serial_connected_.load() ? "true" : "false");
    add_kv("command_mode", command_mode_);
    if (fb.has_batt) {
      add_kv("batt", std::to_string(fb.batt));
    }
    if (fb.has_fault) {
      add_kv("fault", std::to_string(fb.fault));
    }

    diag.status.push_back(status);
    diagnostics_pub_->publish(diag);
  }

private:
  std::string port_;
  int baud_ = 115200;
  double publish_rate_hz_ = 75.0;
  int cmd_timeout_ms_ = 200;
  std::string base_frame_ = "base_link";
  std::string odom_frame_ = "odom";
  double wheel_base_m_ = 0.33;
  double wheel_radius_m_ = 0.05;
  int64_t ticks_per_rev_ = 2048;
  double max_speed_mps_ = 5.0;
  double max_steer_rad_ = 0.45;
  std::string command_mode_ = "mps_rad";
  int pwm_min_ = 1000;
  int pwm_max_ = 2000;
  int steer_center_us_ = 1500;
  int steer_left_us_ = 1900;
  int steer_right_us_ = 1100;
  bool invert_drive_ = false;
  bool use_measured_steer_ = false;
  bool debug_print_ = false;

  std::string ackermann_topic_ = "/ackermann_cmd";
  bool enable_cmd_vel_ = false;
  std::string cmd_vel_topic_ = "/cmd_vel";
  std::string emergency_stop_topic_ = "/emergency_stop";
  bool allow_estop_release_ = true;

  bool publish_odom_ = true;
  bool publish_tf_ = true;
  bool publish_joint_states_ = true;
  bool publish_wheel_ticks_ = true;
  bool publish_diagnostics_ = true;
  std::string drive_wheel_joint_name_ = "drive_wheel_joint";
  std::string steering_joint_name_ = "steering_joint";

  int reconnect_period_ms_ = 1000;

  std::atomic<bool> running_{false};
  std::thread read_thread_;

  std::mutex serial_mutex_;
  int serial_fd_ = -1;
  std::string rx_buffer_;
  std::atomic<bool> serial_connected_{false};
  rclcpp::Time last_connect_attempt_;

  std::mutex cmd_mutex_;
  double cmd_speed_mps_ = 0.0;
  double cmd_steer_rad_ = 0.0;
  double last_steer_cmd_rad_ = 0.0;
  rclcpp::Time last_cmd_time_;
  bool has_cmd_ = false;
  bool estop_latched_ = false;

  std::mutex feedback_mutex_;
  Feedback latest_feedback_;

  bool have_prev_ticks_single_ = false;
  bool have_prev_ticks_lr_ = false;
  int64_t prev_enc_ = 0;
  int64_t prev_enc_l_ = 0;
  int64_t prev_enc_r_ = 0;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double drive_wheel_pos_rad_ = 0.0;
  rclcpp::Time last_odom_time_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr wheel_ticks_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
