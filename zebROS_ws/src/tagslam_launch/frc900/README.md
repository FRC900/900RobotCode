


# how to redetect from bag

```
roslaunch ./sync_and_detect.launch bag:=`pwd`/tagslam_testing.bag output_bag:=`pwd`/frc900_detections.bag
```
NOTE: you need to set ``use_approximate_sync`` to ``true`` since your
wheeled odometry is not synchronized with the camera images. Without
that flag, all data is dropped because the time stamps never line up.
With that flag, all odometry data is dropped except for the one
closest to the camera frames. For tagslam you don't really need the odometry data
at a higher rate than the camera frames.



# about transforms

For TagSLAM the relevant reference frame is ``zed_objdetect_left_camera_optical_frame``. Note: using the optical
frame is crucial. This is the transform between base_link and left camera optical frame:

position: [0.150244 -0.164 0.820355]
rotation: q = [0.612973, -0.612973, 0.352511, -0.352511]

It is now in camera_poses.yaml


# how to run tagslam real-time

```
rosparam set use_sim_time true
roslaunch ./tagslam_realtime.launch data_dir:=`pwd`
rosbag play --clock tagslam_testing.bag  # (the bag you sent me)
```

Use my rviz file if you want to see the tags (needs "RobotModel")

# how to run tagslam from (pre-detected) bag

```
roslaunch ./tagslam_from_bag.launch data_dir:=`pwd`
```


# Notes

- Remove amnesia=true for mapping run, the do "dump" service call and
  look at ``~/.ros/poses.yaml``. See TagSLAM online instructions.

- You can dial in how much you want to rely on the odometry by
  tweaking these two parameters in the tagslam.yaml file. You can also
  bump up ``pixel_noise`` to reduce the weight put on tag detections,
  should have similar effect.
```
     # assumed odom noise in meters(!): the larger this parameter, the less you rely on odom
     odom_translation_noise: 0.01
     # assomed odom noise in radians(!)
     odom_rotation_noise: 0.01
```

- Bigger tags, more tags, further away from each other, always helps.
- Stick with 36h11 or you can  experiment with the more recent
  tags. Don't use lower bit sizes, you will get false positives.
- Make sure to run the UMich detector, not the MIT detector. It is
  faster, more sensitive.
- You may be able to get away without running the tag detector every
  frame. Hack it to only run on every nth frame, lean on your odom and see what
  happens.


  
