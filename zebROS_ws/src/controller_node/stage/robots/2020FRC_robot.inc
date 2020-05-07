
# A camera used for tensorflow object detection
# TODO - these values are all guesses
define normal_camera fiducial
(
	range_min 0.0
	range_max 10.0
	range_max_id 10.0
	fov 359
	ignore_zloc 1
	size [ 0.2 0.2 0.2 ]
	color "purple"
)

define omnidir_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [ 0.03 0.03 999999 999999 999999 0.02 ]

  size [ 0.75 0.75 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "omni"

  # spawn sensors
  normal_camera(pose [ 0.1 -0.1 1.65 0.0 ])
)
