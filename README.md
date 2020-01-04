# 2019 OffeasonCode  [![Build Status](https://travis-ci.com/FRC900/2020RobotCode.svg?token=T6hJWxFcNyyzxifXQqW5&branch=master)](https://travis-ci.com/FRC900/2020RobotCode)

2020 Robot Code

 -----
Offseason dev sandbox for 2019 fall offseason

Notable updates since the 2019 comp season release


- improments to auto-align hatch panel pickup and placement
- add feed-forward to ROS' default PID node
- add a node to publish system uptime
- add a node to zero IMU data (for aligning field-centric swerve)
- optimization to reduce frequency of various HAL calls on the Rio
- add code to sim driver station to set limit switches and line break sensors in simulator
- add code to base trajectory node to auto-generate and optimize spline-based paths
- fixed swerve base odometry calculations
- log Talon / Victor firmware verions
- switch hardware interface time functions to std C++ - prep for running on Windows
- script to make sure USB-attached CAN device is actually attached (can\_up.sh)
- Experimental / hacked up branch to use ROS PinholeCameraModel for screen to world coord conversions
- Updates for 2020 beta wpilib & Rio image
- clean up lots of unused packages and configs
- vim doesn't flash the screen on errors!
