#include <rclcpp/logging.hpp>

#include "path_follower_example.hpp"

#include "path_follower_base.hpp"

#include "rosplane_msgs/msg/attacked_state.hpp"

#include <random>
#include <chrono>

namespace rosplane
{

PathFollowerBase::PathFollowerBase()
    : Node("path_follower_base")
    , params_(this)
    , params_initialized_(false)
{
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&PathFollowerBase::vehicle_state_callback, this, _1));

  current_path_sub_ = this->create_subscription<rosplane_msgs::msg::CurrentPath>(
    "current_path", 100, std::bind(&PathFollowerBase::current_path_callback, this, _1));

  controller_commands_pub_ =
    this->create_publisher<rosplane_msgs::msg::ControllerCommands>("controller_command", 1);

  // Publish attacked state logs
  attacked_state_pub_ =
    this->create_publisher<rosplane_msgs::msg::AttackedState>("attacked_state", 10);

  // Define the callback to handle on_set_parameter_callback events
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PathFollowerBase::parametersCallback, this, std::placeholders::_1));

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  params_initialized_ = true;

  // Now that the parameters have been set and loaded from the launch file, create the timer.
  set_timer();

  state_init_ = false;
  current_path_init_ = false;
  attack_active_ = false;  // Initialize attack state
}

void PathFollowerBase::set_timer()
{
  // Convert the frequency to a period in microseconds
  double frequency = params_.get_double("controller_commands_pub_frequency");
  timer_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));

  update_timer_ =
    this->create_wall_timer(timer_period_, std::bind(&PathFollowerBase::update, this));
}

void PathFollowerBase::update()
{

  Output output;

  if (state_init_ == true && current_path_init_ == true) {
    follow(input_, output);
    rosplane_msgs::msg::ControllerCommands msg;

    rclcpp::Time now = this->get_clock()->now();

    // Populate the message with the required information
    msg.header.stamp = now;
    msg.chi_c = output.chi_c;
    msg.va_c = output.va_c;
    msg.h_c = output.h_c;
    msg.phi_ff = output.phi_ff;

    controller_commands_pub_->publish(msg);
  }
}

std::array<double, 3> PathFollowerBase::apply_attack(double pn, double pe, double h,
                                                     int attack_type)
{
  std::array<double, 3> result = {pn, pe, h};

  rclcpp::Time now = this->get_clock()->now();
  if (!attack_timer_initialized_) {
    attack_start_time_ = now;
    attack_timer_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Attack timing started at: %.2f", now.seconds());
  }

  double elapsed_time = (now - attack_start_time_).seconds();
  double attack_duration = params_.get_double("attack_duration");

  bool was_active = attack_active_;
  attack_active_ = (elapsed_time >= 40.0 && elapsed_time <= (40.0 + attack_duration));

  if (!was_active && attack_active_) {
    RCLCPP_INFO(this->get_logger(), "Attack became ACTIVE at elapsed time %.2f", elapsed_time);
  } else if (was_active && !attack_active_) {
    RCLCPP_INFO(this->get_logger(), "Attack became INACTIVE at elapsed time %.2f", elapsed_time);
  }

  if (!attack_active_ || attack_type == 0) {
    return result;
  }

  double attack_magnitude = params_.get_double("attack_magnitude") / 100.0;
  double l2_norm = std::sqrt(pn * pn + pe * pe + h * h);
  double mag = l2_norm * attack_magnitude - l2_norm;

  switch (attack_type) {
    case 1: { // Point Attack
      if ((now - last_attack_time_).seconds() >= 5.0) {
        last_attack_time_ = now;
        result = {pn + mag, pe + mag, h + mag};
        RCLCPP_WARN(this->get_logger(),
                    "POINT ATTACK TRIGGERED at elapsed time %.2f - Magnitude: %.2f", elapsed_time,
                    mag);
      }
      break;
    }
    case 2: { // Random Value Attack
      if ((now - last_random_time_).seconds() >= 5.0) {
        last_random_time_ = now;
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(1.0, 2.0);
        double rand_mag = mag * dis(gen);
        result = {pn + rand_mag, pe + rand_mag, h + rand_mag};
        RCLCPP_INFO(this->get_logger(),
                    "Random Value Attack - Modified: [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f], "
                    "Rand Mag: %.2f",
                    pn, pe, h, result[0], result[1], result[2], rand_mag);
      }
      break;
    }
    case 3: { // Sequence Attack
      double elapsed = (now - sequence_start_time_).seconds();
      if (elapsed >= attack_duration || sequence_start_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
        sequence_start_time_ = now;
        elapsed = 0;
        RCLCPP_INFO(this->get_logger(), "Sequence Attack - Starting new sequence at elapsed %.2f",
                    elapsed_time);
      }
      if (elapsed < attack_duration) {
        result = {pn + mag, pe + mag, h + mag};
        RCLCPP_INFO(
          this->get_logger(),
          "Sequence Attack - Modified: [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f], Elapsed: %.2f", pn,
          pe, h, result[0], result[1], result[2], elapsed);
      }
      break;
    }
    case 4: { // Ramp Attack
      double elapsed = (now - ramp_start_time_).seconds();
      if (elapsed >= attack_duration || ramp_start_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
        ramp_start_time_ = now;
        elapsed = 0;
        RCLCPP_INFO(this->get_logger(), "Ramp Attack - Starting ramp at elapsed %.2f",
                    elapsed_time);
      }
      if (elapsed < attack_duration) {
        double ramp_factor = elapsed / attack_duration;
        double current_mag = mag * ramp_factor;
        result = {pn + current_mag, pe + current_mag, h + current_mag};
        RCLCPP_INFO(
          this->get_logger(),
          "Ramp Attack - Modified: [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f], Ramp factor: %.2f", pn,
          pe, h, result[0], result[1], result[2], ramp_factor);
      }
      break;
    }
    case 5: { // DoS Attack
      double elapsed = (now - sequence_start_time_).seconds();
      if (elapsed >= attack_duration || sequence_start_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
        sequence_start_time_ = now;
        frozen_position_ = {pn, pe, h};
        is_position_frozen_ = true;
        elapsed = 0;
        RCLCPP_INFO(this->get_logger(), "DoS Attack - Freezing at: [%.2f, %.2f, %.2f]", pn, pe, h);
      }
      if (elapsed < attack_duration && is_position_frozen_) {
        result = frozen_position_;
        RCLCPP_INFO(this->get_logger(),
                    "DoS Attack - Maintaining frozen: [%.2f, %.2f, %.2f], Elapsed: %.2f", result[0],
                    result[1], result[2], elapsed);
      } else {
        is_position_frozen_ = false;
      }
      break;
    }
    case 6: { // Random Pattern Attack
      if (random_attack_times_.empty()
          || (now - sequence_start_time_).seconds() >= attack_duration) {
        sequence_start_time_ = now;
        random_attack_times_.clear();
        std::random_device rd;
        std::mt19937 gen(rd());
        for (int i = 0; i < 10; ++i) {
          double base_time = elapsed_time + (i * attack_duration / 10.0);
          std::uniform_real_distribution<> dis(-1.0, 1.0);
          random_attack_times_.push_back(base_time + dis(gen));
        }
        RCLCPP_INFO(this->get_logger(), "Random Pattern Attack - New times at %.2f", elapsed_time);
      }

      static rclcpp::Time last_pattern_time(0, 0, now.get_clock_type());
      for (const auto & attack_time : random_attack_times_) {
        if (std::abs(elapsed_time - attack_time) < 0.05
            && (now - last_pattern_time).seconds() >= 1.0) {
          last_pattern_time = now;
          result = {pn + mag, pe + mag, h + mag};
          RCLCPP_INFO(this->get_logger(), "Random Pattern Attack - TRIGGERED at %.2f",
                      elapsed_time);
          break;
        }
      }
      break;
    }
    default:
      break;
  }

  return result;
}

void PathFollowerBase::vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg)
{
  rosplane_msgs::msg::AttackedState last_estimated_state_;
  last_estimated_state_.header = msg->header;
  last_estimated_state_.original_state = *msg;

  double pn = msg->position[0];
  double pe = msg->position[1];
  double h = -msg->position[2];

  int attack_type = params_.get_int("attack_type");
  auto attacked_values = apply_attack(pn, pe, h, attack_type);

  input_.pn = attacked_values[0];
  input_.pe = attacked_values[1];
  input_.h = attacked_values[2];

  last_estimated_state_.is_attacked = attack_active_ && attack_type != 0;
  last_estimated_state_.delta_position = {
    static_cast<float>(attacked_values[0] - pn),
    static_cast<float>(attacked_values[1] - pe),
    static_cast<float>(attacked_values[2] - h)
  };

  input_.chi = msg->chi;
  input_.psi = msg->psi;
  input_.va = msg->va;
  state_init_ = true;

  // Convert numeric attack type to string name
  std::string attack_name = "NO_ATTACK";
  if (attack_active_) {
    switch (attack_type) {
      case 1:
        attack_name = "POINT_ATTACK";
        break;
      case 2:
        attack_name = "RANDOM_VALUE_ATTACK"; 
        break;
      case 3:
        attack_name = "SEQUENCE_ATTACK";
        break;
      case 4:
        attack_name = "RAMP_ATTACK";
        break;
      case 5:
        attack_name = "DOS_ATTACK";
        break;
      case 6:
        attack_name = "RANDOM_PATTERN_ATTACK";
        break;
    }
  }
  last_estimated_state_.attack_type = attack_name;  // Now storing string name
  attacked_state_pub_->publish(last_estimated_state_);
}

void PathFollowerBase::current_path_callback(const rosplane_msgs::msg::CurrentPath::SharedPtr msg)
{
  if (msg->path_type == msg->LINE_PATH) {
    input_.p_type = PathType::LINE;
  } else if (msg->path_type == msg->ORBIT_PATH) {
    input_.p_type = PathType::ORBIT;
  }

  // Populate the input message with the correct information
  input_.va_d = msg->va_d;
  for (int i = 0; i < 3; i++) {
    input_.r_path[i] = msg->r[i];
    input_.q_path[i] = msg->q[i];
    input_.c_orbit[i] = msg->c[i];
  }
  input_.rho_orbit = msg->rho;
  input_.lam_orbit = msg->lamda;
  current_path_init_ = true;
}

rcl_interfaces::msg::SetParametersResult
PathFollowerBase::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the path_follower node";

  // Update the param_manager object with the new parameters
  bool success = params_.set_parameters_callback(parameters);
  if (success) {
    result.successful = true;
    result.reason = "success";
  }

  // Check to see if the timer frequency parameter has changed
  if (params_initialized_ && success) {
    double frequency = params_.get_double("controller_commands_pub_frequency");

    std::chrono::microseconds curr_period =
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));
    if (timer_period_ != curr_period) {
      update_timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void PathFollowerBase::declare_parameters()
{
  params_.declare_double("controller_commands_pub_frequency", 10.0);
  params_.declare_double("chi_infty", .5);
  params_.declare_double("k_path", 0.05);
  params_.declare_double("k_orbit", 4.0);
  params_.declare_int("update_rate", 100);
  params_.declare_double("gravity", 9.81);
  params_.declare_int("attack_type", 0);
  params_.declare_double("attack_duration", 50.0);
  params_.declare_double("attack_magnitude", 10.0);
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<rosplane::PathFollowerExample>());
  return 0;
}
