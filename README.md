# trajectory_server
ROS2 Based trajectory server. The function is to abstract away managing mavros states and executing trajectories provided by an external planner.

# TODO
1. Create a planner state message 
    - heartbeat
    - planner state
2. Create a planner Package
    - This planner package will take in waypoints and plan a trajectory to be taken in by the trajectory_server node.
    - Create message for sending waypoints
        - Waypoints
3. 