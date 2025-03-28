# various tools which will eventually be installed in the container but aren't yet : 
#   sudo apt install feedgnuplot  libmrcal-dev mrcal python3-mrcal  vnlog

# This will dump a series of jpg images from the given camera topic
rosrun image_view extract_images image:=/ov2311_10_9_0_9_video0/image_raw

# run bagfile playback to generate images
# This could be done for live cameras as well

# Generate corner data from charuco board
# If you can find a pure chessboard target, there's mrgingham as described here
#  https://mrcal.secretsauce.net/how-to-calibrate.html#org524f742
python3 /home/ubuntu/900RobotCode/zebROS_ws/src/rosbag_scripts/scripts/charuco_calibrator.py  | tee corners.vnl

# Check that corners cover the entire image
< corners.vnl       \
  vnl-filter -p x,y | \
  feedgnuplot --domain --square --set 'xrange [0:2000] noextend' --set 'yrange [2000:0] noextend'

# Run calibration
mrcal-calibrate-cameras                                                         \
  --corners-cache corners.vnl                                                   \
  --lensmodel LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=30_Ny=18_fov_x_deg=150 \
  --focal 1000                                                                  \
  --object-spacing 0.0254                                                       \
  --object-width-n 7                                                            \
  '*.jpg'

# Check error
mrcal-show-projection-uncertainty camera-0.cameramodel --cbmax 4 --unset key
