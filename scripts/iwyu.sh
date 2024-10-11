#!/bin/bash
python3 ~/include-what-you-use/iwyu_tool.py -j 24 -p /home/ubuntu/900RobotCode/zebROS_ws/build/$@ -- -I /usr/include/c++/11 -I /usr/include/x86_64-linux-gnu/c++/11 | tee ~/x
