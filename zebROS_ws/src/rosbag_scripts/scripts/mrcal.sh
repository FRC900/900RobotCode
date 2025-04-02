# various tools which will eventually be installed in the container but aren't yet : 
#   sudo apt install feedgnuplot  libmrcal-dev mrcal python3-mrcal  vnlog

# This will dump a series of jpg images from the given camera topic
# From the mrcal docs :
#    Fill as much of the camera frame as possible
#    Hold the calibration target level with the camera - that is, if
#       flat to the camera, the sides of the chessboard should be parallel
#       to the sides of the camera image (as much as possible)
#    Then tilt the target up and down, left and right, but try not to
#     rotate it.
rosrun image_view extract_images image:=/ov2311_10_9_0_9_video0/image_raw

# run bagfile playback to generate images
# This could be done for live cameras as well

# Generate corner data from charuco board
# If you can find a pure chessboard target without arucotags, there's mrgingham as described here
#  https://mrcal.secretsauce.net/how-to-calibrate.html#org524f742
python3 /home/ubuntu/900RobotCode/zebROS_ws/src/rosbag_scripts/scripts/charuco_calibrator.py  | tee corners.vnl

# Generate corner data from chessboard target
#    This is for a 10x10 chessboard - there's 9 interor corners so use gridn=9
mrgingham -j 32 --gridn 9 '*.jpg' > corners.vnl 

# Check that corners cover the entire image
< corners.vnl       \
  vnl-filter -p x,y | \
  feedgnuplot --domain --square --set 'xrange [0:1599] noextend' --set 'yrange [1299:0] noextend'

# Run calibration for mrcal's stereographic lens model
# fov_x_deg is the horizontal field of view of the camera, this is a guess which can be refined later
mrcal-calibrate-cameras                                                         \
  --corners-cache corners.vnl                                                   \
  --lensmodel LENSMODEL_OPENCV5  \
  --focal 1000                                                                  \
  --object-spacing 0.12                                                         \
  --object-width-n 4                                                            \
  --object-height-n 3                                                            \
  '*.jpg'

# Check error
mrcal-show-projection-uncertainty camera-0.cameramodel --cbmax 4 --unset key

# use the resulting file to find extrinsics
mrcal-to-cahvor camera-0.cameramodel