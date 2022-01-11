# Verify path gen accel and velocity
rosservice call /frcrobot_jetson/intake_controller/intake_arm_command "intake_arm_extend: false" 
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

rosservice call /path_follower/game_piece_path_gen "object_id: 'power_cell'
max_objects: 255
endpoint:
  position:
    x: 8.87
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 
rosservice call /frcrobot_jetson/swerve_drive_controller/reset_odom "data: true"
rosservice call /frcrobot_jetson/swerve_drive_controller/reset_odom "data: false"

rosservice call /zed_ar/reset_tracking "{}" 

PID - rotation P=-2.0
      strafe   P=4.5 D=0.0

	  P=4.2 next experiment
	  2/1.5/1.5 speed/vel

# Eject
rosservice call /frcrobot_jetson/intake_controller/intake_roller_command "percent_out: -0.9" 
rosservice call /frcrobot_jetson/indexer_controller/indexer_command "indexer_velocity: -3.0"


# Stop the squeaking
rosservice call /frcrobot_jetson/intake_controller/intake_roller_command "percent_out: 0" 
rosservice call /frcrobot_jetson/indexer_controller/indexer_command "indexer_velocity: 0"

rosservice call /imu/set_imu_zero "angle: 0.0"




/path_follower/x_position_pid/parameter_descriptions
/path_follower/x_position_pid/parameter_updates
/path_follower/x_position_pid/pid_debug
/path_follower/x_position_pid/pid_enable
/path_follower/x_position_pid/x_cmd_pub
/path_follower/x_position_pid/x_command
/path_follower/x_position_pid/x_state_pub


state = odom reading
command = setpoint
cmd_pub =  PID output

rosservice call /imu/set_imu_zero "angle: 0.0"

####################################################
rosservice call /frcrobot_jetson/intake_controller/intake_arm_command "intake_arm_extend: true" 

rosservice call /zed_ar/reset_tracking "{}" 
sleep 3
ROS_NAMESPACE=auto rosrun behaviors auto_node 

rostopic pub -1 /auto/auto_mode behavior_actions/AutoMode "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
auto_mode: 4
distance_from_wall: 0.0" 

rosservice call /frcrobot_jetson/swerve_drive_controller/reset_odom "data: true"
rosservice call /frcrobot_jetson/swerve_drive_controller/reset_odom "data: false"

rosservice call /zed_ar/reset_tracking "{}" 


