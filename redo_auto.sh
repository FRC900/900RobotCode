# Verify path gen accel and velocity
rosservice call /frcrobot_jetson/intake_controller/intake_arm_command "intake_arm_extend: true" 
rosservice call /frcrobot_jetson/intake_controller/intake_roller_command "percent_out: 0.9" 
rosservice call /frcrobot_jetson/indexer_controller/indexer_command "indexer_velocity: 1.5"
ROS_NAMESPACE=auto rosrun behaviors auto_node 

rostopic pub -1 /auto/auto_mode behavior_actions/AutoMode "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
auto_mode: 6
distance_from_wall: 0.0" 

rosservice call /path_follower/galactic_path_gen "{}" 

