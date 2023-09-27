#include "trajectory_server/trajectory_server.hpp"

TrajectoryServer::TrajectoryServer()
{
  // ROS Parameters
  this->declare_parameter("drone_id", 0);
  this->declare_parameter("origin_frame", "world");
  this->declare_parameter("planner_heartbeat_timeout", 0.25);
  this->declare_parameter("takeoff_height", 1.0);

  this->declare_parameter("pub_cmd_freq", 25.0);
  this->declare_parameter("fsm_tick_freq", 50.0);
  this->declare_parameter("benchmark_freq", 10.0);

  drone_id_ = this->get_parameter("drone_id").as_string(); 
  origin_frame_ = this->get_parameter("origin_frame").as_string(); 
  planner_heartbeat_timeout_ = this->get_parameter("planner_heartbeat_timeout").as_string(); 
  takeoff_height_ = this->get_parameter("takeoff_height").as_string(); 

  double pub_cmd_freq = this->get_parameter("pub_cmd_freq").as_string(); // Frequency to publish PVA commands
  double fsm_tick_freq = this->get_parameter("fsm_tick_freq").as_string(); // Frequency to tick state machine
  double benchmark_freq = this->get_parameter("benchmark_freq").as_string(); // Frequency to tick state machine

  // Callback groups
  fsm_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  mavros_sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Publishers
  setpoint_raw_local_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/mavros/setpoint_raw/local", 10);  

  traj_history_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "uav/traj_history", 10);  

  // Subscribers
  pose_sub_ = this->create_subscription<nav_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", 10, std::bind(&TrajectoryServer::UAVPoseCb, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/mavros/local_position/odom", 10, std::bind(&TrajectoryServer::UAVOdomCb, this, _1));

  mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10, std::bind(&TrajectoryServer::trajCb, this, _1));

  fsm_event_sub_ = this->create_subscription<nav_msgs::msg::Trajectory>(
    "/fsm/event", 10, std::bind(&TrajectoryServer::FSMEventCb, this, _1));

  planner_state_sub_ = this->create_subscription<traj_server_msgs::msg::PlannerState>(
    "/planner/state", 10, std::bind(&TrajectoryServer::FSMEventCb, this, _1));

  traj_sub_ = this->create_subscription<traj_server_msgs::msg::Trajectory>(
    "/planner/trajectory", 10, std::bind(&TrajectoryServer::trajCb, this, _1));

  // Service clients
  arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", 
                                                                    rmw_qos_profile_services_default,
                                                                    service_client_group_);
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/cmd/set_mode", 
                                                                    rmw_qos_profile_services_default,
                                                                    service_client_group_);

  // Timers
  exec_traj_timer_ = this->create_wall_timer((1/pub_cmd_freq)s, std::bind(&TrajectoryServer::execTrajTimerCb, this),
                                      fsm_cb_group_);

  tick_server_state_timer_ = this->create_wall_timer((1/fsm_tick_freq)s, std::bind(&TrajectoryServer::tickServerStateTimerCb, this),
                                      fsm_cb_group_);

  benchmark_timer_ = this->create_wall_timer((1/benchmark_freq)s, std::bind(&TrajectoryServer::benchmarkTimerCb, this),
                                      fsm_cb_group_);

}

TrajectoryServer::~TrajectoryServer()
{
}

/* Subscriber Callbacks */

// TODO
void TrajServer::trajCb(const traj_server_msgs::Trajectory::ConstPtr msg)
{

}

void TrajServer::MavrosStateCb(const mavros_msgs::State::ConstPtr &msg)
{
  // logInfoThrottled(string_format("State: Mode[%s], Connected[%d], Armed[%d]", msg->mode.c_str(), msg->connected, msg->armed), 1.0);
  uav_current_state_ = *msg;
}

void TrajServer::UAVPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  uav_pose_ = *msg; 
  uav_poses_.push_back(uav_pose_);

  if (uav_poses_.size() > max_poses_to_track_) {
    uav_poses_.pop_front(); // Remove the oldest pose
  }

  // Publish mesh visualization
	model_mesh_.pose = msg->pose;
  model_mesh_.header.stamp = msg->header.stamp;

	uav_mesh_pub_.publish(model_mesh_);
}

void TrajServer::UAVOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
  uav_odom_ = *msg;
}

void TrajServer::FSMEventCb(const std_msgs::Int8::ConstPtr & msg)
{
  if (msg->data < 0 || msg->data > ServerEvent::EMPTY_E){
    logError("Invalid server event, ignoring...");
  } 
  setServerEvent(ServerEvent(msg->data));
}

/* Timer Callbacks */
void TrajectoryServer::execTrajTimerCb(const ros::TimerEvent &e)
{
  switch (getServerState()){
    case ServerState::INIT:
      // Do nothing, drone is not initialized
      break;
    
    case ServerState::IDLE:
      // Do nothing, drone has not taken off
      break;
    
    case ServerState::TAKEOFF:
      execTakeOff();
      break;
    
    case ServerState::LAND:
      execLand();
      break;
    
    case ServerState::HOVER:
      execHover();
      break;
    
    case ServerState::MISSION:
      if (isMissionComplete()){
        // logInfoThrottled("Waiting for mission", 5.0);
        execHover();
      }
      else {
        if (isPlannerHeartbeatTimeout()){
          logErrorThrottled("[traj_server] Lost heartbeat from the planner.", 1.0);
          execHover();
        }

        execMission();
      }
      break;

    case ServerState::E_STOP:
      // Do nothing, drone should stop all motors immediately
      break;
  }
}

void TrajectoryServer::tickServerStateTimerCb(const ros::TimerEvent &e)
{
  // logInfoThrottled(string_format("Current Server State: [%s]", StateToString(getServerState()).c_str()), 1.0);

  switch (getServerState())
  {
    case ServerState::INIT:
      {
        // Wait for FCU Connection
        if (uav_current_state_.connected){
          logInfo("[INIT] Connected to flight stack!");
          setServerState(ServerState::IDLE);
        }
        else {
          logInfoThrottled("[INIT] Initializing Server, waiting for connection to FCU...", 2.0 );
        }

        break;
      }
    case ServerState::IDLE:
      // logInfoThrottled("[IDLE] Ready to take off", 5.0 );

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logInfo("[IDLE] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[IDLE] IGNORED EVENT. UAV has not taken off, unable to LAND");
          break;
        case MISSION_E:
          logWarn("[IDLE] IGNORED EVENT. Please TAKEOFF first before setting MISSION mode");
          break;
        case HOVER_E:
          logWarn("[IDLE] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[IDLE] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }
      break;
    
    case ServerState::TAKEOFF:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          // logWarn("[TAKEOFF] IGNORED EVENT. UAV already attempting taking off");
          break;
        case LAND_E:
          logInfo("[TAKEOFF] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[TAKEOFF] IGNORED EVENT. Wait until UAV needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[TAKEOFF] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[TAKEOFF] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!is_uav_ready()){
        logInfo("[TAKEOFF] Calling toggle offboard mode");
        toggle_offboard_mode(true);
      }

      if (isTakenOff()){
        logInfo("[TAKEOFF] Take off complete");
        setServerState(ServerState::HOVER);
      }
      else {
        logInfoThrottled("[TAKEOFF] Taking off...", 1.0 );
      }

      break;
    
    case ServerState::LAND:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logInfo("[LAND] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[LAND] IGNORED EVENT. UAV already attempting landing");
          break;
        case MISSION_E:
          logWarn("[LAND] IGNORED EVENT. UAV is landing, it needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[LAND] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[LAND] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (isLanded()){
        logInfo("[LAND] Landing complete");
        setServerState(ServerState::IDLE);
      }
      else {
        logInfoThrottled("[LAND] landing...", 1.0);
      }

      setServerState(ServerState::IDLE);
      break;
    
    case ServerState::HOVER:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logWarn("[HOVER] IGNORED EVENT. UAV already took off. Currently in [HOVER] mode");
          break;
        case LAND_E:
          logInfo("[HOVER] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logInfo("[HOVER] UAV entering [MISSION] mode.");
          setServerState(ServerState::MISSION);
          break;
        case HOVER_E:
          logWarn("[HOVER] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[HOVER] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      break;
    
    case ServerState::MISSION:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logWarn("[MISSION] IGNORED EVENT. UAV already took off. Currently in [MISSION] mode");
          break;
        case LAND_E:
          logWarn("[MISSION] Mission cancelled! Landing...");
          endMission();
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[MISSION] IGNORED EVENT. UAV already in [MISSION] mode");
          break;
        case HOVER_E:
          logWarn("[MISSION] Mission cancelled! Hovering...");
          endMission();
          setServerState(ServerState::HOVER);
          break;
        case E_STOP_E:
          logFatal("[MISSION] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!isMissionComplete()){
        // logInfoThrottled("[MISSION] executing mission...", 5.0);
      }
      break;

    case ServerState::E_STOP:
      logFatalThrottled("[E_STOP] Currently in E STOP State, please reset the vehicle and trajectory server!", 1.0);
      break;

  }
}

/* Trajectory execution methods */

bool TrajectoryServer::toggleOffboard(const bool& toggle)
{
    bool arm_val = false;
    std::string set_mode_val = "AUTO.LOITER"; 
    if (toggle){
        arm_val = true;
        set_mode_val = "OFFBOARD"; 
    }

    auto conditions_fulfilled = [&] () {
        return (toggle ? is_uav_ready() : is_uav_idle());
    };

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm_val;

    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = set_mode_val;

    // Make sure takeoff is not immediately sent, 
    // this will help to stream the correct data to the program first.
    // Will give a 1sec buffer
    // ros::Duration(1.0).sleep();

    ros::Rate rate(pub_cmd_freq_);

    // send a few setpoints before starting
    for (int i = 0; ros::ok() && i < 10; i++)
    {
        execTakeOff();
        ros::spinOnce();
        rate.sleep();
    }
    ros::Time last_request_t = ros::Time::now();

    while (!conditions_fulfilled()){

        bool request_timeout = ((ros::Time::now() - last_request_t) > ros::Duration(2.0));

        if (uav_current_state_.mode != set_mode_val && request_timeout)
        {
        if (set_mode_client.call(set_mode_srv))
        {
            if (set_mode_srv.response.mode_sent){
            logInfo(string_format("Setting %s mode successful", set_mode_val.c_str()));
            }
            else {
            logInfo(string_format("Setting %s mode failed", set_mode_val.c_str()));
            }
        }
        else {
            logInfo("Service call to PX4 set_mode_client failed");
        }

        last_request_t = ros::Time::now();
        }
        else if (uav_current_state_.armed != arm_val && request_timeout) 
        {
        if (arming_client.call(arm_cmd)){
            if (arm_cmd.response.success){
            logInfo(string_format("Setting arm to %d successful", arm_val));
            }
            else {
            logInfo(string_format("Setting arm to %d failed", arm_val));
            }
        }
        else {
            logInfo("Service call to PX4 arming_client failed");
        }

        last_request_t = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();        
    }

    return true;
}

void TrajectoryServer::execLand()
{
  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, landed_height_;
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publishCmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);
}

void TrajectoryServer::execTakeOff()
{ 
  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, takeoff_height_;
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publishCmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);

}

void TrajectoryServer::execHover()
{
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration
  Eigen::Vector3d pos;
  pos << last_mission_pos_(0), last_mission_pos_(1), last_mission_pos_(2);

  if (last_mission_pos_(0) < 0.1){
    pos(2) = takeoff_height_;
  }

  publishCmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);
}

// TODO
void TrajectoryServer::execMission()
{

}

void TrajectoryServer::startMission(){
  mission_completed_ = false;
}

void TrajectoryServer::endMission()
{
  mission_completed_ = true;
}

void TrajServer::publishCmd(
  Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate, uint16_t type_mask)
{
  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = origin_frame_;
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  pos_cmd.type_mask = type_mask;
  // pos_cmd.type_mask = 1024; // Ignore Yaw
  // pos_cmd.type_mask = 2048; // ignore yaw_rate
  // pos_cmd.type_mask = 2496; // Ignore Acceleration
  // pos_cmd.type_mask = 3520; // Ignore Acceleration and Yaw
  // pos_cmd.type_mask = 2552; // Ignore Acceleration, Velocity, 
  // pos_cmd.type_mask = 3576; // Ignore Acceleration, Velocity and Yaw

  pos_cmd.position.x = p(0);
  pos_cmd.position.y = p(1);
  pos_cmd.position.z = p(2);
  pos_cmd.velocity.x = v(0);
  pos_cmd.velocity.y = v(1);
  pos_cmd.velocity.z = v(2);
  pos_cmd.acceleration_or_force.x = a(0);
  pos_cmd.acceleration_or_force.y = a(1);
  pos_cmd.acceleration_or_force.z = a(2);
  pos_cmd.yaw = yaw;
  pos_cmd.yaw_rate = yaw_rate;
  pos_cmd_raw_pub_.publish(pos_cmd);
}

/* Conditional checking methods */

bool TrajectoryServer::isLanded()
{
  // Check that difference between desired landing height and current UAV position
  // is within tolerance 
  return abs(uav_pose_.pose.position.z - landed_height_) < take_off_landing_tol_;
}

bool TrajectoryServer::isTakenOff()
{
  // Check that difference between desired landing height and current UAV position
  // is within tolerance 
  return abs(uav_pose_.pose.position.z - takeoff_height_) < take_off_landing_tol_;
}

bool TrajectoryServer::isMissionComplete()
{
  return mission_completed_;
}

bool TrajectoryServer::isPlannerHeartbeatTimeout(){
  return (ros::Time::now() - heartbeat_time_).toSec() > planner_heartbeat_timeout_;
}

/* FSM Methods */

void TrajectoryServer::setServerState(ServerState des_state)
{
  logInfo(string_format("Transitioning server state: %s -> %s", 
    StateToString(getServerState()).c_str(), StateToString(des_state).c_str()));

  server_state_ = des_state;
}

ServerState TrajectoryServer::getServerState()
{
  return server_state_;
}

void TrajectoryServer::setServerEvent(ServerEvent event)
{
  // logInfo(string_format("Set server event: %s", EventToString(event).c_str()));

  server_event_ = event;
}

ServerEvent TrajectoryServer::getServerEvent()
{
  // logInfo(string_format("Retrieved server event: %s", EventToString(server_event_).c_str()));
  ServerEvent event = server_event_;
  server_event_ = ServerEvent::EMPTY_E;  // Reset to empty

  return event;
}
