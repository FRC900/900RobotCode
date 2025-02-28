# Requested rotation position and velocity.
# Setting velocity to non-zero will apply velocity
# feed forward in addition to PID when calculating rotation effort.
float64 position
float64 velocity
# Drive the robot from the teleop node
# If set to true, the teleop node will drive the robot
# to the requested position, even if the x&y strafe 
# commanded from the joystics are 0.  This is the setting
# to use if the node requesting rotation won't be publishing
# a cmd_vel message to drive the robot.
# If set to false, the teleop node will only sent 
# drive commands if the joystick strafe values are
# non-zero.  If the x&y stick values are 0, the
# teleop node will stop publishing, even if the robot
# hasn't reached the requested rotation position. This
# is the setting to use if another node will be sending
# a cmd_vel message.
bool    drive_from_teleop