# Actual ros build and install :

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential ninja-build
sudo rosdep init
rosdep update
mkdir ~/melodic_arm_cross_ws
cd ~/melodic_arm_cross_ws
rosinstall_generator ros ros_comm robot angles serial robot_localization controller_interface controller_manager combined_robot_hw joint_limits_interface transmission_interface controller_manager controller_interface hardware_interface controller_manager_tests controller_manager_msgs combined_robot_hw combined_robot_hw_tests tf2_tools tf2_eigen tf2_sensor_msgs rosparam_shortcuts rqt_controller_manager actionlib_tutorials image_transport image_geometry --rosdistro melodic --deps --wet-only > melodic-ros_comm-wet.rosinstall

#edit melodic-ros_comm-wet.rosinstall and remove entries for realtime_tools, filter
sed -i -e '/local-name: filters/{N;N;N;d}' melodic-ros_comm-wet.rosinstall

mkdir -p ~/melodic_arm_cross_ws/src
cd ~/melodic_arm_cross_ws/src

git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
wget https://raw.githubusercontent.com/ros-gbp/urdfdom_headers-release/master/indigo/package.xml
# Fix the version in package.xml to read 1.0.0
sed -i -e 's/:{version}/1.0.0/' package.xml 

cd ~/melodic_arm_cross_ws/src
#git clone https://github.com/jbeder/yaml-cpp.git

# Grab urdfdom, add a boilerplate package.xml in it
# so it will build as a ROS package
wget https://github.com/ros/urdfdom/archive/1.0.0.tar.gz
tar -xzvf 1.0.0.tar.gz
rm 1.0.0.tar.gz
mv urdfdom-1.0.0 urdfdom
cd urdfdom
echo '<?xml version="1.0"?>
<package>
  <name>urdfdom</name>
  <version>1.0.0</version>
  <description>URDF DOM</description>
  <maintainer email="a@google.com">Nobody</maintainer>
  <license>BSD</license>
  <buildtool_depend>cmake</buildtool_depend>
  <build_depend>urdfdom_headers</build_depend>

  <export>
  </export>

</package>
' > package.xml

cd ~/melodic_arm_cross_ws/src

touch .rosinstall
wstool merge ../melodic-ros_comm-wet.rosinstall
wstool update -j8

# add "<depend>urdfdom_headers</depend>" to src/urdf/urdf_parser_plugin/package.xml
sed -i -e '/<\/package>/i  <build_depend>urdfdom_headers<\/build_depend>' urdf/urdf_parser_plugin/package.xml 

# localization_ekf - -Werror needs to be removed due to eigen warnings
# Add class_loader to src/urdf/urdf package.xml exec_depend and CMakeLists CATKIN_DEPENDS

# In a docker container : 
# docker run -it --user ubuntu -v /home/kjaget/2020RobotCode:/home/ubuntu/2020RobotCode -v ~/melodic_arm_cross_ws:/home/ubuntu/melodic_arm_cross_ws  frc900/zebros-2020-dev /bin/bash

# Then run the following from inside the container :

cd ~/melodic_arm_cross_ws
# Do a fresh build - kill off any lingering dependencies
rm -rf ~/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic devel_isolated build_isolated

#catkin config --install --install-space $HOME/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic 
#catkin build -DCMAKE_TOOLCHAIN_FILE=$HOME/2020RobotCode/zebROS_ws/rostoolchain.cmake

# Note - if this fails looking for gencpp*cmake, run from a new terminal
# window where no ROS setup.bash has previously been sourced
./src/catkin/bin/catkin_make_isolated --install --use-ninja -DCMAKE_INSTALL_PREFIX=$HOME/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2020RobotCode/zebROS_ws/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF

# Add newly built cross-libs to git repo so they are
# used for subsequent Rio imagings
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi
rm ~/2020RobotCode/roscore_roborio.tar.bz2
tar -cf ~/2020RobotCode/roscore_roborio.tar opt/ros/melodic
bzip2 -9 ~/2020RobotCode/roscore_roborio.tar

# !!!NOTE!!! - important - copy roscore_melodic_roborio.tar.bz2 into the docker repo
# so it gets picked up in subsequent builds

# I needed to add "-DYAML_CPP_INCLUDE_DIRS=/$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/include
# -DYAML_CPP_LIBRARIES=/$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/libyaml-cpp.a" to
# catkin_make_isolated to get it to find yaml-cpp

