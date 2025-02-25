
# A camera used for tensorflow object detection
# TODO - these values are all guesses
define normal_camera fiducial
(
	range_min 0.0
	range_max 7.0
	range_max_id 7.0
	fov 100
	ignore_zloc 1
	size [ 0.2 0.2 0.2 ]
	color "purple"
)

define non_normal_camera fiducial
(
	range_min 0.0
	range_max 7.0
	range_max_id 7.0
	fov 100
	ignore_zloc 1
	size [ 0.2 0.2 0.2 ]
	color "green"
)

# terabee evo 3m
define terabee_distance sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ]
  # define the range bounds [min max]
  range [0.01 3.0]
  # define the angular field of view in degrees
  fov 2
  # define the number of samples spread over the fov
  samples 1
)

define single_terrabee ranger
(
  terabee_distance(pose [ 0 0 0 0 ] )
)

define omnidir_robot position
(
  update_interval 5
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "gps"
  odom_error [ 0.02 0.02 0.0 0.0 ]

  size [ 0.8382 0.8382 0.10 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # shape from https://player-stage-manual.readthedocs.io/en/latest/WORLDFILES/
  # A tool to turn an SVG into a polygon compatible with Stage would be pretty awesome
  block
  (
        points 6
        point[5] [0 0]
        point[4] [0 1]
        point[3] [0.75 1]
        point[2] [1 0.75]
        point[1] [1 0.25]
        point[0] [0.75 0]
        z [0 1]
  )

  block
  (
        points 4
        point[3] [1 0.75]
        point[2] [1.25 0.75]
        point[1] [1.25 0.625]
        point[0] [1 0.625]
        z [0 0.5]
  )

  block
  (
        points 4
        point[3] [1 0.375]
        point[2] [1.25 0.375]
        point[1] [1.25 0.25]
        point[0] [1 0.25]
        z [0 0.5]
  )

  # kinematics
  drive "omni"

  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-5.5 5.5 -5.5 5.5 0 0 -720 720]
  acceleration_bounds [-4.5 4.5 -4.5 4.5 0 0 -720 720]

  # spawn sensors
  normal_camera(pose [ 0.1 -0.1 0.65 0.0 ] )
 
  non_normal_camera(pose [ -0.1 -0.1 0.65 180.0 ] )

  single_terrabee(pose [0.4445 0.0 0.0 0.0 ] )

  single_terrabee(pose [0.0 0.4445 0.0 90.0 ] )

	#camera
	#(
	#  # laser properties
	#  resolution [ 100 100 ]
	#  range [ 0.2 8.0 ]
	#  fov [ 80.0 50.0 ]
	#  pantilt [ 0.0 0.0 ]

	  # model properties
	#  size [0.2286 0.66675 0.15 ]
	#  pose [0.4445 0.0 -0.3 0 ]
	#  color "blue"
	#  watts 0.0
	#)
)
