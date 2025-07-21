#!/bin/bash

CodeChecker check --ignore /home/ubuntu/900RobotCode/scripts/wpilib.skipfile --logfile /home/ubuntu/900RobotCode/zebROS_ws/build/$@/compile_commands.json --disable clang-diagnostic-reserved-macro-identifier --disable clang-diagnostic-reserved-identifier
