
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
  update_interval 5
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "odom"
  odom_error [ 0.02 0.02 0.0 0.002 ]

  size [ 0.889 0.889 0.30 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "omni"

  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-5.5 5.5 -5.5 5.5 0 0 -90 90]
  acceleration_bounds [-4.5 4.5 -4.5 4.5 0 0 -90 90]

  # spawn sensors
  normal_camera(pose [ 0.1 -0.1 0.65 0.0 ])
)
