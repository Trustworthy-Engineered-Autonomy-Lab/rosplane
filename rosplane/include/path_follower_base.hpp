#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <rclcpp/rclcpp.hpp>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/controller_commands.hpp"
#include "rosplane_msgs/msg/current_path.hpp"
#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/msg/attacked_state.hpp"
#include <deque>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace rosplane
{

enum class PathType
{
  ORBIT,
  LINE
};

enum class AttackType
{
  NO_ATTACK = 0,        // Default - no attack
  L2_NORM = 1,          // L2 Norm Attack
  POINT_ATTACK = 2,     // Point Attack
  RANDOM_VALUE = 3,     // Random Value Attack
  SEQUENCE = 4,         // Sequence Attack
  RAMP = 5,             // Ramp Attack
  DOS = 6,              // Denial-of-Service Attack
  RANDOM_PATTERN = 7    // Random Pattern Attack
};

class PathFollowerBase : public rclcpp::Node
{
public:
  PathFollowerBase();
  float spin();

protected:
  struct Input
  {
    PathType p_type;
    float va_d;
    float r_path[3];
    float q_path[3];
    float c_orbit[3];
    float rho_orbit;
    int lam_orbit;
    float pn;  /** position north */
    float pe;  /** position east */
    float h;   /** altitude */
    float va;  /** airspeed */
    float chi; /** course angle */
    float psi; /** heading angle */
  };

  struct Output
  {
    double va_c;   /** commanded airspeed (m/s) */
    double h_c;    /** commanded altitude (m) */
    double chi_c;  /** commanded course (rad) */
    double phi_ff; /** feed forward term for orbits (rad) */
  };

  virtual void follow(const Input & input, Output & output) = 0;

  ParamManager params_;

private:
  /**
   * Subscribes to state from the estimator
   */
  rclcpp::Subscription<rosplane_msgs::msg::AttackedState>::SharedPtr vehicle_state_sub_;

  /** 
   * Subscribes to the current_path topic from the path manager
   */
  rclcpp::Subscription<rosplane_msgs::msg::CurrentPath>::SharedPtr current_path_sub_;

  /**
   * Publishes commands to the controller
   */
  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_pub_;


  std::chrono::microseconds timer_period_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  bool params_initialized_;
  bool state_init_;
  bool current_path_init_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rosplane_msgs::msg::ControllerCommands controller_commands_;
  Input input_;

  rosplane_msgs::msg::AttackedState last_estimated_state_;
  
  // Attack timing variables
  bool attack_timer_initialized_{false};
  rclcpp::Time attack_start_time_{0, 0, RCL_ROS_TIME};
  
  // Existing timing variables
  rclcpp::Time sequence_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ramp_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attack_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_random_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pattern_time_{0, 0, RCL_ROS_TIME};

  /**
   * @brief Sets the timer with the timer period as specified by the ROS2 parameters
   */
  void set_timer();

  /**
   * @brief Callback for the subscribed state messages from the estimator
   */
  void vehicle_state_callback(const rosplane_msgs::msg::AttackedState::SharedPtr msg);

  /**
   * @brief Callback for the subscribed current_path messages from the path_manager
   */
  void current_path_callback(const rosplane_msgs::msg::CurrentPath::SharedPtr msg);

  /**
   * @brief Calculates and publishes the commands messages
   */
  void update();

  /**
   * @brief Callback for when ROS2 parameters change.
   * 
   * @param Vector of rclcpp::Parameter objects that have changed
   * @return SetParametersResult object that describes the success or failure of the request
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which can be overridden by a parameter file
   */
  void declare_parameters();

  std::deque<std::array<double, 3>> ned_history_;
  std::array<double, 3> accumulated_ned_ = {0.0, 0.0, 0.0};
  const size_t BUFFER_SIZE = 5;

  bool attack_active_ = false;
  AttackType current_attack_type_ = AttackType::NO_ATTACK;

  // Helper function to apply attacks to position values
  std::array<double, 3> apply_attack(double pn, double pe, double h, int attack_type);


  // Attack state variables
  static constexpr double EPSILON = 0.01;
  static constexpr double ATTACK_PERIOD = 40.0;
  static constexpr double ATTACK_DURATION = 10.0;
  
  // Attack state tracking
  std::array<double, 3> frozen_position_ = {0.0, 0.0, 0.0};
  std::vector<double> random_attack_times_;
  bool is_position_frozen_ = false;
};

} // namespace rosplane

#endif // PATH_FOLLOWER_BASE_H
