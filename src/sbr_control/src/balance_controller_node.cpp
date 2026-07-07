#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sbr_msgs/msg/balance_state.hpp"

#include "sbr_control/balance_controller.hpp"

namespace sbr_control
{

class BalanceControllerNode : public rclcpp::Node
{
public:
  BalanceControllerNode()
  : rclcpp::Node("balance_controller_node")
  {
    loop_rate_ = declare_parameter<double>("loop_rate", 200.0);
    if (loop_rate_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "loop_rate must be > 0 (got %.3f)", loop_rate_);
      throw std::invalid_argument("loop_rate must be > 0");
    }

    params_.pitch_gains.kp = declare_parameter<double>("pitch_kp", 6.0);
    params_.pitch_gains.ki = declare_parameter<double>("pitch_ki", 0.0);
    params_.pitch_gains.kd = declare_parameter<double>("pitch_kd", 0.4);
    params_.pitch_gains.integral_limit = declare_parameter<double>("integral_limit", 0.5);
    params_.pitch_gains.output_limit = declare_parameter<double>("output_limit", 1.0);
    params_.pitch_offset = declare_parameter<double>("pitch_offset", 0.0);
    params_.output_scale = declare_parameter<double>("output_scale", 1.0);
    params_.fall_threshold = declare_parameter<double>("fall_threshold", 0.78);
    params_.lean_per_velocity = declare_parameter<double>("lean_per_velocity", 0.08);
    params_.steer_gain = declare_parameter<double>("steer_gain", 0.4);
    controller_.set_params(params_);

    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    imu_timeout_ = declare_parameter<double>("imu_timeout", 0.2);
    invert_pitch_ = declare_parameter<bool>("invert_pitch", false);
    log_state_ = declare_parameter<bool>("log_state", true);

    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto wheel_cmd_topic = declare_parameter<std::string>("wheel_cmd_topic", "/wheel_cmd");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&BalanceControllerNode::on_imu, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&BalanceControllerNode::on_cmd_vel, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&BalanceControllerNode::on_odom, this, std::placeholders::_1));

    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_cmd_topic, 10);
    state_pub_ = create_publisher<sbr_msgs::msg::BalanceState>("balance_state", 10);

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&BalanceControllerNode::on_set_params, this, std::placeholders::_1));

    last_cmd_time_ = now();
    last_imu_time_ = now();
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / loop_rate_),
      std::bind(&BalanceControllerNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "balance_controller_node started at %.1f Hz", loop_rate_);
  }

private:
  static double pitch_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.0) {
      return std::copysign(M_PI / 2.0, sinp);
    }
    return std::asin(sinp);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Degraded-IMU convention: a negative orientation covariance marks the
    // sample as unusable for control (e.g. imu_node fell back to mock after
    // a hardware failure). Skip it so the freshness watchdog can trip.
    if (msg->orientation_covariance[0] < 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Ignoring IMU sample flagged invalid (orientation_covariance[0] < 0)");
      return;
    }

    double pitch = pitch_from_quaternion(msg->orientation);
    double pitch_rate = msg->angular_velocity.y;
    if (invert_pitch_) {
      pitch = -pitch;
      pitch_rate = -pitch_rate;
    }
    pitch_ = pitch;
    pitch_rate_ = pitch_rate;
    have_imu_ = true;
    last_imu_time_ = now();
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_cmd_ = msg->linear.x;
    angular_cmd_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;
  }

  void on_timer()
  {
    if (!have_imu_) {
      return;
    }

    // IMU-freshness watchdog: a frozen IMU must not leave the last wheel
    // command standing. Publish an explicit zero (motor_node's watchdog only
    // trips on command ABSENCE, and the sim effort controller holds values).
    if ((now() - last_imu_time_).seconds() > imu_timeout_) {
      controller_.reset();
      std_msgs::msg::Float64MultiArray stop_msg;
      stop_msg.data = {0.0, 0.0};
      wheel_pub_->publish(stop_msg);
      sbr_msgs::msg::BalanceState state;
      state.header.stamp = now();
      state.header.frame_id = "base_link";
      state.pitch = pitch_;
      state.balancing = false;
      state_pub_->publish(state);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "IMU data stale (> %.2f s) - motors commanded to zero", imu_timeout_);
      return;
    }

    if ((now() - last_cmd_time_).seconds() > cmd_timeout_) {
      linear_cmd_ = 0.0;
      angular_cmd_ = 0.0;
    }

    // Measured dt (clamped): timer jitter or a slow cycle would otherwise
    // corrupt the PID integral, which assumes the nominal period.
    const double nominal_dt = 1.0 / loop_rate_;
    const rclcpp::Time step_now = now();
    double dt = nominal_dt;
    if (last_step_time_.nanoseconds() > 0) {
      dt = (step_now - last_step_time_).seconds();
      dt = std::max(0.25 * nominal_dt, std::min(dt, 4.0 * nominal_dt));
    }
    last_step_time_ = step_now;

    const auto cmd = controller_.update(pitch_, pitch_rate_, linear_cmd_, angular_cmd_, dt);

    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = {cmd.left_effort, cmd.right_effort};
    wheel_pub_->publish(wheel_msg);

    sbr_msgs::msg::BalanceState state;
    state.header.stamp = now();
    state.header.frame_id = "base_link";
    state.pitch = pitch_;
    state.pitch_rate = pitch_rate_;
    state.pitch_setpoint = cmd.pitch_setpoint;
    state.linear_command = linear_cmd_;
    state.angular_command = angular_cmd_;
    state.left_effort = cmd.left_effort;
    state.right_effort = cmd.right_effort;
    state.balancing = cmd.balancing;
    state.pos_x = pos_x_;
    state.pos_y = pos_y_;
    state.pos_z = pos_z_;
    state_pub_->publish(state);

    if (log_state_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "BAL  pitch=%+.2f deg  rate=%+.2f rad/s  set=%+.2f deg  "
        "L=%+.2f R=%+.2f  pos=[%+.2f %+.2f %+.2f] m",
        pitch_ * 180.0 / M_PI,
        pitch_rate_,
        cmd.pitch_setpoint * 180.0 / M_PI,
        cmd.left_effort,
        cmd.right_effort,
        pos_x_, pos_y_, pos_z_);
    }
  }

  rcl_interfaces::msg::SetParametersResult on_set_params(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "pitch_kp") {
        params_.pitch_gains.kp = p.as_double();
      } else if (name == "pitch_ki") {
        params_.pitch_gains.ki = p.as_double();
      } else if (name == "pitch_kd") {
        params_.pitch_gains.kd = p.as_double();
      } else if (name == "integral_limit") {
        params_.pitch_gains.integral_limit = p.as_double();
      } else if (name == "output_limit") {
        params_.pitch_gains.output_limit = p.as_double();
      } else if (name == "pitch_offset") {
        params_.pitch_offset = p.as_double();
      } else if (name == "output_scale") {
        params_.output_scale = p.as_double();
      } else if (name == "fall_threshold") {
        params_.fall_threshold = p.as_double();
      } else if (name == "lean_per_velocity") {
        params_.lean_per_velocity = p.as_double();
      } else if (name == "steer_gain") {
        params_.steer_gain = p.as_double();
      } else if (name == "imu_timeout") {
        imu_timeout_ = p.as_double();
      } else if (name == "log_state") {
        log_state_ = p.as_bool();
      }
    }
    controller_.set_params(params_);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  BalanceController controller_;
  BalanceController::Params params_;
  double loop_rate_{200.0};
  double cmd_timeout_{0.5};
  double imu_timeout_{0.2};
  bool invert_pitch_{false};
  bool log_state_{true};

  double pitch_{0.0};
  double pitch_rate_{0.0};
  double linear_cmd_{0.0};
  double angular_cmd_{0.0};
  double pos_x_{0.0};
  double pos_y_{0.0};
  double pos_z_{0.0};
  bool have_imu_{false};
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_step_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Publisher<sbr_msgs::msg::BalanceState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace sbr_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sbr_control::BalanceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
