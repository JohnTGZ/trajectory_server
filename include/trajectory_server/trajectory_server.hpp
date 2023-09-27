#ifndef TRAJECTORY_SERVER__TRAJECTORY_SERVER_HPP_
#define TRAJECTORY_SERVER__TRAJECTORY_SERVER_HPP_

#include "trajectory_server/visibility_control.h"

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int8.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"

#include "mavros_msgs/msg/command_bool.hpp"
#include "mavros_msgs/msg/set_mode.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/* State machine  */
enum ServerState
{
  INIT,
  IDLE,
  TAKEOFF,
  LAND,
  HOVER,
  MISSION,
  E_STOP,
};

/* State machine events */
enum ServerEvent
{
  TAKEOFF_E,        // 0
  LAND_E,           // 1
  MISSION_E,        // 2
  HOVER_E,          // 3
  E_STOP_E,         // 4
  EMPTY_E,          // 5
};

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class TrajectoryServer : public rclcpp::Node
{

public:
  TrajectoryServer();

  virtual ~TrajectoryServer();

// ROS 
private: 
  // Callback groups
  rclcpp::CallbackGroup::SharedPtr fsm_cb_group_;
  rclcpp::CallbackGroup::SharedPtr mavros_sub_group_;
  rclcpp::CallbackGroup::SharedPtr service_client_group_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_raw_local_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_history_pub_;

  // Subscribers
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr fsm_event_sub_;
  rclcpp::Subscription<traj_server_msgs::msg::Trajectory>::SharedPtr traj_sub_;

  rclcpp::Subscription<traj_server_msgs::msg::PlannerState>::SharedPtr planner_state_sub_;

  // Service clients
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr exec_traj_timer_; // Execute trajectory timer
  rclcpp::TimerBase::SharedPtr tick_server_state_timer_; // Manage FSM states timer
  rclcpp::TimerBase::SharedPtr benchmark_timer_; // Timer for obtaining benchmark values such as trajectory tracking error

// Parameters
private:
  std::string node_name_{"traj_server"};

  int drone_id_; // ID of drone being commanded by trajectory server instance
  std::string origin_frame_; // frame that the drone originated from i.e. it's local pose is (0,0,0) w.r.t to this frame.

  double planner_heartbeat_timeout_{0.5}; // Planner heartbeat timeout

  double takeoff_height_{0.0}; // Default height to take off to
  double landed_height_{0.05}; // We assume that the ground is even (z = 0)
  double take_off_landing_tol_{0.05};

// Logging methodss
private:
  void logInfo(const std::string& str){
    ROS_INFO_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logError(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

};

#endif  // TRAJECTORY_SERVER__TRAJECTORY_SERVER_HPP_
