#!/bin/bash
# script to install cross-development environment on a laptop
# This should let people build robot (roboRIO) code on a laptop

# Install toolchain 
# wget frc2019
# extract to $HOME/frc2019
# run scripts/ToolsUpdater.sh
# add $HOME/frc2019/roborio/bin to path

# Install CTRE libraries for TalonSRX controller
# included in above for now but keep handy
# for upgrading if needed
#
# Grab using maven?

# Install wpilib env
# Extract maven zip files into lib/wpilib & include/wpilib
mkdir -p $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib
cd $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib
find ../../../../maven/ -name \*head\*zip | xargs -n1 unzip -o
mkdir -p $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/wpilib
cd $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/wpilib
find ../../../../maven/ -name \*athena\*zip | xargs -n1 unzip -o

# Same for CTRE headers
http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix

# And navX
http://www.kauailabs.com/maven2/com/kauailabs/navx/navx/
http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/
# Get ros for RoboRIO libraries
cd $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi
sudo tar -xjf $HOME/2018Offseason/roscore_roborio_2018.tar.bz2
cd
rm roscore_roborio_2018.tar.bz2

# Clone 2018Offseason repo
cd
git clone https://github.com/FRC900/2018Offseason.git

# Install roboRIO packages into the cross-root
sudo perl $HOME/2018Offseason/install_cross_package.pl

# Build/install cross version of console_bridge
cd
git clone https://github.com/ros/console_bridge
cd console_bridge
git checkout 0.3.2
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=~/frc2019/roborio/arm-frc2019-linux-gnueabi -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j4 install
cd
sudo rm -rf console_bridge

# Build and install Poco libraries
cd
wget https://pocoproject.org/releases/poco-1.7.9/poco-1.7.9p1.tar.gz
tar xzf poco-1.7.9p1.tar.gz
cd poco-1.7.9p1/
CROSS_COMPILE=arm-frc2019-linux-gnueabi- ./configure --no-tests --no-samples --omit=Data/ODBC,Data/MySQL --minimal --prefix=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/usr/local --static --cflags="-fPIC"
CROSS_COMPILE=arm-frc2019-linux-gnueabi- make -j4 install
cd
sudo rm -rf poco-1.7.9p1.tar.gz poco-1.7.9p1

#Hack in a cmake file for Eigen3
mkdir -p $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/usr/lib/cmake/eigen3
sed -e 's_/usr_\$ENV{HOME}/frc2019/roborio/arm-frc2019-linux-gnueabi/usr_' < /usr/lib/cmake/eigen3/Eigen3Config.cmake > $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/usr/lib/cmake/eigen3/Eigen3Config.cmake

# Build and instll SIP libraries
cd
wget https://downloads.sourceforge.net/project/pyqt/sip/sip-4.17/sip-4.17.tar.gz
tar -xzvf sip-4.17.tar.gz
cd sip-4.17
python configure.py CC=arm-frc2019-linux-gnueabi-gcc CXX=arm-frc2019-linux-gnueabi-g++ LINK=arm-frc2019-linux-gnueabi-g++ LINK_SHLIB=arm-frc2019-linux-gnueabi-g++ --sysroot=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi --incdir=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/usr/include/python2.7 STRIP=arm-frc2019-linux-gnueabi-strip

#edit siplib/Makefile to change CPP flags include to -I/usr/arm-frc-linux-gnueabi/usr/include/python2.7
sed -i '/^CPPFLAGS/ s_include_usr/include_' siplib/Makefile
make -j8 install
cd
sudo rm -rf sip-4.17.tar.gz sip-4.17

# Build and install tinyxml
wget https://github.com/leethomason/tinyxml2/archive/6.0.0.tar.gz
tar -xzvf 6.0.0.tar.gz
cd tinyxml2-6.0.0
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=~/frc2019/roborio/arm-frc2019-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_STATIC_LIBS:BOOL=ON -DBUILD_TESTS:BOOL=OFF .
make -j8 install
cd
rm -rf 6.0.0.tar.gz tinyxml2-6.0.0

# Different code uses different versions of tinyxml
cd
wget https://downloads.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.zip
unzip tinyxml_2_6_2.zip
cd tinyxml
wget https://gist.githubusercontent.com/TNick/7960323/raw/3046ecda1d4d54d777c407f43ac357846a192e05/TinyXML-CmakeLists.txt
mv TinyXML-CmakeLists.txt CMakeLists.txt
#add a line to CMakeLists.txt :  
sed -i "14i  set_target_properties(tinyxml PROPERTIES PUBLIC_HEADER \"tinyxml.h;tinystr.h\")" CMakeLists.txt
#add a line to tinyxml.h before line 46 :
sed -i '45i  #define TIXML_USE_STL' tinyxml.h
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
make -j8 install
cd
sudo rm -rf tinyxml_2_6_2.zip tinyxml

# Build and install qhull libraries
cd
wget http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz
tar -xzvf qhull-2015-src-7.2.0.tgz
cd qhull-2015.2/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi .
make -j8 install
cd
sudo rm -rf qhull-2015-src-7.2.0.tgz qhull-2015.2

# Build and install assimp libraries
cd
git clone https://github.com/assimp/assimp.git
cd assimp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi ..
make -j8 install
cd
sudo rm -rf assimp

# Build and install UUID libraries
cd
wget https://downloads.sourceforge.net/project/libuuid/libuuid-1.0.3.tar.gz
tar -xzvf libuuid-1.0.3.tar.gz
cd libuuid-1.0.3
CFLAGS="-O2" CXXFLAGS="-O2" ./configure --host=arm-frc2019-linux-gnueabi --prefix=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/usr/local
make install
cd
sudo rm -rf libuuid-1.0.3.tar.gz libuuid-1.0.3

# Build and install yaml-cpp
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS:BOOL=OFF -DYAML_CPP_BUILD_TESTS=OFF ..
make -j8 install
cd
sudo rm -rf yaml-cpp

cd
git clone https://github.com/rdiankov/collada-dom.git
cd collada-dom
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$HOME/2018Offseason/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi .
make -j8 install
cd
sudo rm -rf collada-dom
