#include <rclcpp/logging.hpp>

#include "path_follower_example.hpp"

#include "path_follower_base.hpp"

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

  // Start scheduling attacks here
  schedule_attacks();
}

void PathFollowerBase::schedule_attacks()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(40, 90); // Random attack window between 10-50 seconds

  if (attack_timers_.empty()) { // Ensure we schedule attacks only once
    for (int i = 0; i < 5; ++i) {
      int attack_delay = dist(gen); // Random delay for attack

      // Capture the attack_delay in the lambda to avoid the error
      attack_timers_.emplace_back(
        this->create_wall_timer(std::chrono::seconds(attack_delay), [this, attack_delay, i]() {
          RCLCPP_DEBUG(this->get_logger(), "Attack %d triggered after %d seconds", i + 1,
                       attack_delay);
          trigger_attack();
          attack_timers_[i]->cancel(); // Cancel the timer after the attack is triggered
        }));
    }
  }
}

void PathFollowerBase::trigger_attack()
{
  attack_active_ = true;
  RCLCPP_WARN(this->get_logger(), "Attack triggered!");

  attack_reset_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    attack_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Attack ended");
    attack_reset_timer_->cancel();
  });
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

void PathFollowerBase::vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg)
{
  // input_.pn = msg->position[0]; /** position north */
  // input_.pe = msg->position[1]; /** position east */
  // input_.h = -msg->position[2]; /** altitude */
  // input_.chi = msg->chi;
  // input_.psi = msg->psi;
  // input_.va = msg->va;

  // RCLCPP_DEBUG_STREAM(this->get_logger(), "FROM STATE -- input.chi: " << input_.chi);

  // state_init_ = true;
  // Extract raw coordinates
  double pn = msg->position[0]; // Position North
  double pe = msg->position[1]; // Position East
  double h = -msg->position[2]; // Altitude (Down is negative)

  // Compute L2 norm
  double norm = std::sqrt(pn * pn + pe * pe + h * h);

  // Normalize NED coordinates (avoid division by zero)
  double norm_pn = (norm > 1e-6) ? pn / norm : 0.0;
  double norm_pe = (norm > 1e-6) ? pe / norm : 0.0;
  double norm_h = (norm > 1e-6) ? h / norm : 0.0;

  // Add the new normalized values to the rolling buffer
  ned_history_.push_back({norm_pn, norm_pe, norm_h});

  // If buffer exceeds size 5, remove the oldest entry
  if (ned_history_.size() > BUFFER_SIZE) {
    std::array<double, 3> removed = ned_history_.front();
    accumulated_ned_[0] -= static_cast<double>(removed[0]);
    accumulated_ned_[1] -= static_cast<double>(removed[1]);
    accumulated_ned_[2] -= static_cast<double>(removed[2]);
    ned_history_.pop_front();
  }

  // Update accumulated sum with the new normalized values
  accumulated_ned_[0] += static_cast<double>(norm_pn);
  accumulated_ned_[1] += static_cast<double>(norm_pe);
  accumulated_ned_[2] += static_cast<double>(norm_h);

  if (attack_active_) {
    input_.pn = pn + (accumulated_ned_[0] * 0.1);
    input_.pe = pe + (accumulated_ned_[1] * 0.1);
    input_.h = h + (accumulated_ned_[2] * 0.1);
  } else {
    input_.pn = pn;
    input_.pe = pe;
    input_.h = h;
  }

  input_.chi = msg->chi;
  input_.psi = msg->psi;
  input_.va = msg->va;

  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Modified input.pn: " << input_.pn << ", input.pe: " << input_.pe
                                            << ", input.h: " << input_.h);

  state_init_ = true;
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
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<rosplane::PathFollowerExample>());
  return 0;
}
