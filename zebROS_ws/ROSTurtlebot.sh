# Setup ROS for Turtlebot Development

source /opt/ros/noetic/setup.bash
source ~/900RobotCode/devel/setup.bash
export ROS_MASTER_URI=http://10.0.0.120:11311

# Set to local device IP?
export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages
