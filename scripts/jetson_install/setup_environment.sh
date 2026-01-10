# Script to setup Jetson Orin  NX environment. 
# See _xavier.sh script for a similar script for the Jetson Xavier NX

#install basic dependencies
sudo apt-add-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update

# Keep these the original version to line up with kernel versions supported by arducam?
# TODO : Test updating after installing camera stuff
sudo apt-mark hold nvidia-l4t-bootloader nvidia-l4t-display-kernel nvidia-l4t-initrd nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers nvidia-l4t-tools nvidia-l4t-xusb-firmware
sudo apt -y upgrade

# These are listed 1 package per line to hopefully make git merging easier
# They're also sorted alphabetically to keep packages from being listed multiple times

    # libclang1-9 \
    # libgtsam-dev \
    # libgtsam-unstable-dev \
sudo apt install -y \
    build-essential \
    can-utils \
    ccache \
    chromium-browser \
    clang-12 \
    cmake \
    cowsay \
    dbus-x11 \
    exfat-fuse \
    gcc-11 \
    gcc-12 \
	gcc-13 \
    g++-11 \
    g++-12 \
    g++-13 \
    gdb \
    gfortran \
    git \
    git-lfs \
    gstreamer1.0-plugins-* \
    hdf5-tools \
    htop \
    libatlas-base-dev \
    libboost-all-dev \
    libblas-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libclang-12-dev \
    libeigen3-dev \
    libflann-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgoogle-perftools-dev \
    libgmock-dev \
    libgpiod-dev \
    libgtk2.0-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libjpeg8-dev \
    liblapack-dev \
    libleveldb-dev \
    liblmdb-dev \
    liblua5.3-dev \
    libnlopt-cxx-dev \
    libnlopt-dev \
    libpcl-dev \
    libproj-dev \
    libqt5designer5 \
    libqt5designercomponents5 \
    libsnappy-dev \
    libsuitesparse-dev \
    libtinyxml2-dev \
    net-tools \
    ninja-build \
    nmap \
    ntp \
    ntpstat \
    openssh-client \
    pkg-config \
    pyqt5-dev-tools \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-pyqt5 \
    python3-pyqtgraph \
    python3-scipy \
    python3 \
    rsync \
    software-properties-common \
    terminator \
    tree \
    unzip \
    v4l-conf \
    v4l-utils \
    vim-gtk \
    wget \
    xfonts-scalable \
    zip \
    zlib1g-dev \
    zstd

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 130 --slave /usr/bin/g++ g++ /usr/bin/g++-13
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 120 --slave /usr/bin/g++ g++ /usr/bin/g++-12
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110 --slave /usr/bin/g++ g++ /usr/bin/g++-11
sudo update-alternatives --auto gcc

# CUDA with c++20 requires a newer version of cmake than is provided via apt
cd
wget https://github.com/Kitware/CMake/releases/download/v3.31.8/cmake-3.31.8.tar.gz
tar -xf cmake-3.31.8.tar.gz
cd cmake-3.31.8
cmake -GNinja -DCMAKE_BUILD_TYPE:STRING=Release .
sudo ninja install
sudo mv /usr/bin/cmake /usr/bin/cmake.old
sudo ln -s /usr/local/bin/cmake /usr/bin/cmake
cd ..
sudo rm -rf cmake-3.31.8*

# Install tinyxml2
cd
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2
mkdir build
cd build
cmake -GNinja ..
sudo ninja install
cd ../..
sudo rm -rf tinyxml2

#install zed sdk
wget --no-check-certificate https://download.stereolabs.com/zedsdk/5.0/l4t36.4/jetsons
chmod 755 jetsons
./jetsons
rm ./jetsons
rm -rf /home/ubuntu/.local/lib/python3.8/site-packages/numpy

# Grab repo to make it easier to copy files from there to the Jetson
cd
git clone https://github.com/FRC900/900RobotCode.git
cd ~/900RobotCode

# Set up can0 network interface
#cd
#echo "auto can0" > can0
#echo "iface can0 inet manual" >> can0
#echo "  pre-up /sbin/ip link set can0 type can bitrate 1000000" >> can0
#echo "  up /sbin/ifconfig can0 up" >> can0
#echo "  down /sbin/ifconfig can0 down" >> can0
#sudo mv can0 /etc/network/interfaces.d

sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr.list "https://deb.ctr-electronics.com/ctr.list"
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr2024.list "https://deb.ctr-electronics.com/ctr2024.list"
sudo sed -i -e 's/tools stable main/tools jetson main/' /etc/apt/sources.list.d/ctr2024.list

sudo apt remove linux-headers-generic linux-headers-5.4.0-* linux-headers-5.4.0-*-generic 
sudo apt update
sudo apt install -y dkms
sudo apt install -y canivore-usb
# cd /tmp
# unzip /home/ubuntu/900RobotCode/scripts/jetson_install/canivore-usb-arm64-Ubuntu-20.04-v3.zip
# sudo dpkg -i canivore-usb-kernel_1.13_arm64.deb
# sudo dpkg -i canivore-usb_1.13_arm64.deb
# sudo apt-mark hold canivore-usb canivore-usb-kernel

# Re-enable if we want to use a canivore usb interface
# sudo bash -c "echo \"[Match\"] >> /etc/systemd/network/80-can.network"
# sudo bash -c "echo \"Name=can0\" >> /etc/systemd/network/80-can.network"
# sudo bash -c "echo \\"" >> /etc/systemd/network/80-can.network"
# sudo bash -c "echo \"[CAN\"] >> /etc/systemd/network/80-can.network"
# sudo bash -c "echo \"BitRate=1000K\" >> /etc/systemd/network/80-can.network"
# sudo systemctl enable systemd-networkd
# sudo systemctl restart systemd-networkd

sudo bash -c "echo \"# Modules for CAN interface\" >> /etc/modules"
sudo bash -c "echo can >> /etc/modules"
sudo bash -c "echo can_raw >> /etc/modules"
sudo bash -c "echo can_dev >> /etc/modules"
sudo bash -c "echo gs_usb >> /etc/modules"
#sudo bash -c "echo mttcan >> /etc/modules"

# This shouldn't be the least bit dangerous
#sudo rm /etc/modprobe.d/blacklist-mttcan.conf

# Disable l4tbridge - https://devtalk.nvidia.com/default/topic/1042511/is-it-safe-to-remove-l4tbr0-bridge-network-on-jetson-xavier-/
sudo systemctl disable nv-l4t-usb-device-mode.service
sudo systemctl stop nv-l4t-usb-device-mode.service

# Set up ssh host config (add port 5801)
sudo sed "s/#Port 22/Port 22\nPort 5801/g" /etc/ssh/sshd_config > sshd_config && sudo mv sshd_config /etc/ssh

#sudo bash -c "echo NTP=us.pool.ntp.org >> /etc/systemd/timesyncd.conf"
#sudo bash -c "echo FallbackNTP=ntp.ubuntu.com >> /etc/systemd/timesyncd.conf"
sudo cp ~/900RobotCode/scripts/jetson_install/ntp.conf /etc/ntp.conf
# On 10.9.0.9, uncommment last few lines of ntp.conf

sudo cp ~/900RobotCode/scripts/jetson_setup/hwrtc.service /etc/systemd/system
sudo chmod 664 /etc/systemd/system/hwrtc.service
# The ntp config should read from hwrtc -> system clock if it can't
# get to the internet to read from pool time servers
sudo systemctl enable hwrtc

    
# and keys for connections to Rio
mkdir -p ~/.ssh
cd ~/.ssh
tar -xjf ~/900RobotCode/scripts/jetson_setup/jetson_dot_ssh.tar.bz2
chmod 640 authorized_keys
cd ~
chmod 700 .ssh

sudo mkdir -p /root/.ssh
sudo tar -xjf /home/ubuntu/900RobotCode/scripts/jetson_setup/jetson_dot_ssh.tar.bz2 -C /root/.ssh
sudo chown root:root /root/.ssh/*
sudo chmod 640 /root/.ssh/authorized_keys
sudo chmod 700 /root/.ssh

cd ~/900RobotCode/scripts
sudo cp ./jetson_setup/10-local.rules ./jetson_setup/99-gpio.rules ./jetson_setup/99-terabee-pico.rules ./jetson_setup/99-terabee-teensy.rules /etc/udev/rules.d/
sudo service udev reload
sleep 2
sudo service udev restart

# Clean up Jetson
sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/libvisionworks*
# Save ~400MB
sudo apt remove --purge -y thunderbird libreoffice-* nsight-graphics-for-embeddedlinux-*
# Disable automatic updates
sudo sed -i -e 's/APT::Periodic::Update-Package-Lists "1"/APT::Periodic::Update-Package-Lists "0"/' /etc/apt/apt.conf.d/10periodic

# Install CTRE & navX libs
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/ctre
mkdir -p /home/ubuntu/ctre
cd /home/ubuntu/ctre
python3 /home/ubuntu/900RobotCode/scripts/jetson_install/download_maven.py https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json 
python3 /home/ubuntu/900RobotCode/scripts/jetson_install/download_maven.py https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json 
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include
find /home/ubuntu/ctre -name \*headers\*zip | grep -v debug | xargs -n 1 unzip -o
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/ctre
find /home/ubuntu/ctre -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
rm -rf /home/ubuntu/ctre

cd /home/ubuntu
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/4.0.433/navx-cpp-4.0.433-headers.zip
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include/navx
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include/navx
unzip -o /home/ubuntu/navx-cpp-4.0.433-headers.zip
rm /home/ubuntu/navx-cpp-4.0.433-headers.zip
cd /home/ubuntu
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/4.0.433/navx-cpp-4.0.433-linuxathena.zip
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/navx
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/navx
unzip -o /home/ubuntu/navx-cpp-4.0.433-linuxathena.zip
rm /home/ubuntu/navx-cpp-4.0.433-linuxathena.zip
cd /home/ubuntu
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/4.0.433/navx-cpp-4.0.433-linuxathenastatic.zip
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/navx
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/navx
unzip -o /home/ubuntu/navx-cpp-4.0.433-linuxathenastatic.zip
rm /home/ubuntu/navx-cpp-4.0.433-linuxathenastatic.zip

# And Rev sparkmax stuff
cd /home/ubuntu
mkdir sparkmax
cd sparkmax
python3 /home/ubuntu/900RobotCode/scripts/jetson_install/download_maven.py https://software-metadata.revrobotics.com/REVLib-2024.json
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include
find /home/ubuntu/sparkmax -name \*header\*zip | grep -v debug | xargs -n 1 unzip -o
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev
find /home/ubuntu/sparkmax -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
rm -rf /home/ubuntu/sparkmax

# Install wpilib headers by copying them from the local maven dir
export WPILIBVER=2025.1.1-beta-1
cd /home/ubuntu &&\
wget https://frcmaven.wpi.edu/ui/api/v1/download/contentBrowsing/installer/v$WPILIBVER/Linux/WPILib_Linux-$WPILIBVER.tar.gz &&\
mkdir -p /home/ubuntu/wpilib/2025 &&\
cd /home/ubuntu/wpilib/2025 &&\
tar -xzf /home/ubuntu/WPILib_Linux-$WPILIBVER.tar.gz &&\
tar -xzf WPILib_Linux-$WPILIBVER/WPILib_Linux-$WPILIBVER-artifacts.tar.gz &&\
rm /home/ubuntu/WPILib_Linux-$WPILIBVER.tar.gz &&\
cd /home/ubuntu/wpilib/2025/tools &&\
python3 ToolsUpdater.py &&\
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/wpilib &&\
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/wpilib &&\
find ../../../.. -name \*athena\*zip | grep -v debug | xargs -n1 unzip -o &&\
find . -name \*.debug -delete &&\
mkdir -p /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include/wpilib &&\
cd /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include/wpilib &&\
find ../../../.. -name \*headers\*zip | xargs -n1 unzip -o &&\
cd /home/ubuntu/wpilib/2025/tools &&\
mv roborioteamnumbersetter roboRIOTeamNumberSetter.py .. &&\
rm -rf /home/ubuntu/wpilib/2025/advantagescope /home/ubuntu/wpilib/2025/maven /home/ubuntu/wpilib/frc2024/jdk /home/ubuntu/wpilib/2025/WPILib_Linux-$WPILIBVER /home/ubuntu/wpilb2024/utility /home/ubuntu/wpilib/2025/jdk /home/ubuntu/wpilib/2025/documentation /home/ubuntu/wpilib/2025/vsCodeExtensions /home/ubuntu/wpilib/2025/vendordeps /home/ubuntu/wpilib/2025/utility /home/ubuntu/wpilib/2025/tools /home/ubuntu/wpilib/2025/frccode /home/ubuntu/wpilib/2025/installUtils /home/ubuntu/wpilib/2025/WPILIB_Linux-{$wpilibver} &&\
mv /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include/wpilib/google/protobuf /home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/include/wpilib/google/protobuf.bak &&\
sed -i -e 's/   || defined(__thumb__) \\/   || defined(__thumb__) \\\n   || defined(__aarch64__) \\/' /home/ubuntu/wpilib/2024/roborio/arm-frc2024-linux-gnueabi/include/wpilib/FRC_FPGA_ChipObject/fpgainterfacecapi/NiFpga.h

# Set up prereqs for deploy script
mv ~/900RobotCode ~/900RobotCode.orig
ln -s ~/900RobotCode.orig ~/900RobotCode
mkdir -p ~/900RobotCode.prod/zebROS_ws
mkdir -p ~/900RobotCode.dev/zebROS_ws

sudo mkdir -p /usr/local/zed/settings
sudo chmod 755 /usr/local/zed/settings
sudo cp ~/900RobotCode/scripts/jetson_install/calibration_files/*.conf /usr/local/zed/settings
sudo chmod 644 /usr/local/zed/settings/*

cp ~/900RobotCode/.vimrc ~/900RobotCode/.gvimrc ~
sudo cp ~/900RobotCode/kjaget.vim /usr/share/vim/vim82/colors

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson NX"

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

sudo ln -s /usr/include/opencv4 /usr/include/opencv

echo "source /home/ubuntu/900RobotCode/zebROS_ws/command_aliases.sh" >> /home/ubuntu/.bashrc

# Give the ubuntu user dialout permission, which is used by the ADI IMU
sudo adduser ubuntu dialout

git clone https://github.com/VundleVim/Vundle.vim.git /home/ubuntu/.vim/bundle/Vundle.vim
vim +PluginInstall +qall
ln -sf /home/ubuntu/.vim/bundle/vim-ros-ycm/.ycm_extra_conf.py /home/ubuntu/.vim/bundle/vim-ros-ycm/ycm_extra_conf.py
cd /home/ubuntu/.vim/bundle/YouCompleteMe
git fetch origin
git submodule update --init --recursive
python3 ./install.py --clang-completer --system-libclang --ninja

sudo pip3 install -U setuptools==70.3.0
sudo -H bash
export PATH=$PATH:/usr/local/cuda/bin
export CUDA_ROOT=/usr/local/cuda
pip3 install pycuda

# Ultralytics YOLOv8 prereqs here
sudo python3 -m pip install --no-cache-dir --upgrade 'pascal_voc==0.0.7'
sudo python3 -m pip install --no-cache-dir --upgrade 'matplotlib>=3.2.2'
sudo python3 -m pip install --no-cache-dir --upgrade 'opencv-python>=4.6.0'
sudo python3 -m pip install --no-cache-dir --upgrade 'Pillow>=7.1.2'
sudo python3 -m pip install --no-cache-dir --upgrade 'PyYAML==5.4.1'
sudo python3 -m pip install --no-cache-dir --upgrade 'requests>=2.23.0'
sudo python3 -m pip install --no-cache-dir --upgrade 'scipy>=1.4.1'
sudo python3 -m pip install --no-cache-dir --upgrade 'tqdm>=4.64.0'
sudo python3 -m pip install --no-cache-dir --upgrade 'pandas>=1.1.4'
sudo python3 -m pip install --no-cache-dir --upgrade 'seaborn>=0.11.0'
sudo python3 -m pip install --no-cache-dir --upgrade psutil

sudo python3 -m pip install --no-cache-dir --upgrade 'onnx>=1.12'
sudo python3 -m pip install --no-cache-dir --upgrade 'onnxsim>=0.4.1'
wget https://nvidia.box.com/shared/static/6l0u97rj80ifwkk8rqbzj1try89fk26z.whl -O onnxruntime_gpu-1-19.0-cp310-cp310-linux_aarch64.whl
wget https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/869/e41abdc35e093/onnxruntime_gpu-1.22.0-cp310-cp310-linux_aarch64.whl#sha256=869e41abdc35e09345876f047fce49267d699df3e44b67c2518b0469739484ff
wget https://pypi.jetson-ai-lab.io/jp6/cu126/+f/4eb/e6a8902dc7708/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl#sha256=4ebe6a8902dc7708434b2e1541b3fe629ebf434e16ab5537d1d6a622b42c622b

sudo pip3 install onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl 
rm onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl 

# cpu-only version : sudo python3 -m pip install --no-cache-dir --upgrade 'onnxruntime'
#sudo python3 -m pip install --no-cache-dir --upgrade nvidia-pyindex
#sudo python3 -m pip install --no-cache-dir --upgrade nvidia-tensorrt

sudo apt-get install -y libopenblas-base libopenmpi-dev
wget https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl#sha256=62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc
sudo pip3 install torch-2.8.0-cp310-cp310-linux_aarch64.whl 
rm torch-2.8.0-cp310-cp310-linux_aarch64.whl

wget https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl#sha256=907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1
sudo pip3 install torchvision-0.23.0-cp310-cp310-linux_aarch64.whl 
rm torchvision-0.23.0-cp310-cp310-linux_aarch64.whl

sudo python3 -m pip install --no-cache-dir --upgrade 'pytorch_pfn_extras'
sudo python3 -m pip install --no-cache-dir --upgrade ultralytics

cd /home/ubuntu
git clone https://github.com/triple-Mu/YOLOv8-TensorRT.git
# End of ultralytics YOLOv8 deps

echo "export PATH=\$PATH:/home/ubuntu/.local/bin:/home/ubuntu/tensorflow_workspace/tools:/usr/local/cuda/bin" >> /home/ubuntu/.bashrc

# Set up Gold linker - speed up libPCL links
# Do this after building protoc, since that fails with ld.gold
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.gold" 20
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.bfd" 10
sudo update-alternatives --auto ld

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

# This is handled by the ROS*.sh scripts
#echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev/linux/aarm64/shared:/usr/local/lib" >> /home/ubuntu/.bashrc

# Install pyserial (for 2023 intake reader)
sudo pip3 install pyserial
sudo pip3 install cupy-cuda11x

cd /home/ubuntu &&\
    git clone https://github.com/abseil/abseil-cpp.git &&\
    cd abseil-cpp &&\
    mkdir build &&\
    cd build &&\
    cmake -DABSL_BUILD_TESTING=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DABSL_USE_GOOGLETEST_HEAD=ON -DCMAKE_CXX_STANDARD=17 -GNinja .. &&\
    ninja &&\
    sudo ninja install &&\
    cd /home/ubuntu && \
    rm -rf abseil-cpp

# Jetvariety camera stuff
sudo pip3 install v4l2-fix jetson-stats

cd ~
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m arducam

# Install gtsam - tagslam prereq
cd /home/ubuntu &&\
    git clone https://github.com/borglab/gtsam.git &&\
    cd gtsam &&\
    git checkout release/4.2 &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_WITH_TBB=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -GNinja .. &&\
    ninja &&\
    sudo ninja install &&\
    cd /home/ubuntu &&\
    sudo rm -rf gtsam

# Needed to get catkin to put python libs in the correct locations
sudo pip3 install -U 'setuptools<66'

### ROS setup
# sudo sh -c "echo 'deb [arch=arm64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
sudo apt update
sudo apt install -y \
    nlohmann-json3-dev \
    libompl-dev \
    libturbojpeg0-dev \
    ompl-demos \
    python3-rosdistro \
    python3-rosinstall \
    python3-rosinstall-generator  
sudo python3 -m pip install --no-cache-dir "catkin-pkg==1.0.0" "catkin-tools==0.9.5" rosdep roslibpy vcstool vcstools
cd /usr/lib/python3/dist-packages/
sudo patch -p0 < /home/ubuntu/900RobotCode/scripts/jetson_install/catkin_pkg.patch 

sudo rosdep init
rosdep update

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
mkdir src
sudo apt update

# TODO - do we need rqt on the Jetson?
rosinstall_generator \
       apriltag \
       apriltag_ros \
       camera_calibration \
       controller_manager \
       control_msgs \
       control_toolbox \
       cv_bridge \
       ecl_geometry \
       hardware_interface \
       image_pipeline \
       image_transport_plugins \
       imu_filter_madgwick \
       joint_limits_interface \
       joint_state_publisher \
       joint_state_publisher_gui \
       joy \
       map_server \
       marker_msgs \
       moveit \
       navigation \
       pcl_conversions \
       pcl_ros \
       robot_localization \
       robot_state_publisher \
       ros_base \
       ros_type_introspection \
       rosbridge_suite \
       roslint \
       rosparam_shortcuts \
       rospy_message_converter \
       rqt \
       rqt_common_plugins \
       rqt_controller_manager \
       rqt_tf_tree \
       rosserial \
       serial \
       smach \
       smach_ros \
       tf \
       tf2_py \
       tf2_ros \
       tf2_tools \
       transmission_interface \
       turtlesim \
       twist_mux \
       twist_mux_msgs \
       usb_cam \
       xacro \
       --rosdistro noetic --deps --tar > .rosinstall
grep -n -A2 -B1 rosconsole$ .rosinstall  | sed -n 's/^\([0-9]\{1,\}\).*/\1d/p' | sed -i -f - .rosinstall
echo '- git:' >> .rosinstall
echo '    local-name: rosconsole' >> .rosinstall
echo '    uri: https://github.com/ros-o/rosconsole' >> .rosinstall
vcs import --input .rosinstall ./src
sed -i -e 's/python3-catkin-pkg-modules/python3-catkin-pkg/' src/rospack/package.xml
sed -i -e 's$<exec_depend condition="\$ROS_PYTHON_VERSION == 3">python3-rosdep-modules</exec_depend>$$' src/rospack/package.xml
sed -i -e 's/python3-rospkg-modules/python3-rospkg/' src/rqt/rqt_gui/package.xml
sed -i -e 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 20)/' src/robot_localization/CMakeLists.txt
sed -i -e 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 20)/' src/urdf/urdf/CMakeLists.txt
sed -i -e 's/add_compile_options(-std=c++14)/add_compile_options(-std=c++20)/' src/rosparam_shortcuts/CMakeLists.txt
sed -i -e 's/add_compile_options(-std=c++11)/add_compile_options(-std=c++20)/' src/rqt_image_view/CMakeLists.txt
sed -i -e 's/add_compile_options(-std=c++11)/add_compile_options(-std=c++20)/' src/geometry/tf/CMakeLists.txt
sed -i -e 's/add_compile_options(-std=c++14)/add_compile_options(-std=c++20)/' src/perception_pcl/pcl_ros/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/resource_retriever/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/sophus/CMakeLists.txt
sed -i -e 's/option(BUILD_TESTS "Build tests." ON)/option(BUILD_TESTS "Build tests." OFF)/' src/sophus/CMakeLists.txt
sed -i -e 's/option(BUILD_EXAMPLES "Build examples." ON)/option(BUILD_EXAMPLES "Build examples." OFF)/' src/sophus/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/rosparam_shortcuts/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/robot_state_publisher/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/roscpp_core/rostime/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/g' src/pluginlib/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/geometry/tf/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/geometric_shapes/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/fcl/CMakeLists.txt
sed -i -e 's/-std=c++11/-std=c++20/' src/ros_type_introspection/CMakeLists.txt
sed -i -e 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 20)/' src/class_loader/CMakeLists.txt
sed -i -e 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 20)/' src/kdl_parser/kdl_parser/CMakeLists.txt
sed -i -e 's/project(laser_geometry)/project(laser_geometry)\nset(CMAKE_CXX_STANDARD 17)/' src/laser_geometry/CMakeLists.txt
sed -i -e 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 20)/' src/sophus/CMakeLists.txt
sed -i -e 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 20)/' src/joystick_drivers/joy/CMakeLists.txt
sed -i -e 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 20)/' src/perception_pcl/pcl_ros/CMakeLists.txt
sed -i -e 's/${avcodec_LIBRARIES}/${avcodec_LIBRARIES} ${avutil_LIBRARIES}/' src/usb_cam/CMakeLists.txt
sed -i -e 's/pkg_check_modules(avcodec libavcodec REQUIRED)/pkg_check_modules(avcodec libavcodec REQUIRED)\npkg_check_modules(avutil libavutil REQUIRED)/' src/usb_cam/CMakeLists.txt
sed -i -e 's/project(rviz)/project(rviz)\nunset(CMAKE_CXX_STANDARD)\nset(CMAKE_CXX_STANDARD 17)/' src/rviz/CMakeLists.txt
sed -i -e 's/project(gazebo_ros)/project(gazebo_ros)\nunset(CMAKE_CXX_STANDARD)\nset(CMAKE_CXX_STANDARD 17)/' src/gazebo_ros_pkgs/gazebo_ros/CMakeLists.txt
sed -i -e 's/project(gazebo_plugins)/project(gazebo_plugins)\nunset(CMAKE_CXX_STANDARD)\nset(CMAKE_CXX_STANDARD 17)/' src/gazebo_ros_pkgs/gazebo_plugins/CMakeLists.txt
sed -i -e 's/project(gazebo_ros_control)/project(gazebo_ros_control)\nunset(CMAKE_CXX_STANDARD)\nset(CMAKE_CXX_STANDARD 17)/' src/gazebo_ros_pkgs/gazebo_ros_control/CMakeLists.txt
sed -i -e 's/project(rviz_imu_plugin)/project(rviz_imu_plugin)\nunset(CMAKE_CXX_STANDARD)\nset(CMAKE_CXX_STANDARD 17)/' src/imu_tools/rviz_imu_plugin/CMakeLists.txt
sed -i -e 's/OcTreeBase<NODE>(double res) : OcTreeBaseImpl<NODE,AbstractOcTree>(res) {};/OcTreeBase(double res) : OcTreeBaseImpl<NODE,AbstractOcTree>(res) {};/' src/octomap/octomap/include/octomap/OcTreeBase.h
sed -i -e 's/operator == (const ScanNode\& other)/operator == (const ScanNode\& other) const/' src/octomap/octomap/include/octomap/ScanGraph.h 
sed -i -e 's/operator == (const ScanEdge\& other)/operator == (const ScanEdge\& other) const/' src/octomap/octomap/include/octomap/ScanGraph.h 
sed -i -e 's/SphereSpecification<S>(S radius_, const Vector3<S>\& center_)/SphereSpecification(S radius_, const Vector3<S>\& center_)/' src/fcl/test/narrowphase/detail/convexity_based_algorithm/test_gjk_libccd-inl_signed_distance.cpp
sed -i -e 's/project(moveit_kinematics)/project(moveit_kinematics)\nset(CMAKE_CXX_STANDARD 17)/' src/moveit/moveit_kinematics/CMakeLists.txt
sed -i -e 's/project(moveit_ros_visualization)/project(moveit_ros_visualization)\nset(CMAKE_CXX_STANDARD 17)/' src/moveit/moveit_ros_visualization/CMakeLists.txt
sed -i -e 's/project(moveit_setup_assistant)/project(moveit_setup_assistant)\nset(CMAKE_CXX_STANDARD 17)/' src/moveit/moveit_setup_assistant/CMakeLists.txt
sed -i -e 's/PlanningContextBase<GeneratorT>(const std::string\& name, const std::string\& group/PlanningContextBase(const std::string\& name, const std::string\& group/' src/moveit/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_base.h
sed -i -e 's/#if span_CPP11_OR_GREATER && span_FEATURE( BYTE_SPAN ) \&\& ( span_HAVE( BYTE ) || span_HAVE( NONSTD_BYTE ) )/#if 0/' src/ros_type_introspection/include/ros_type_introspection/utils/span.hpp
sed -i -e 's/python/python3/' ~/ros_catkin_ws/src/gazebo_ros_pkgs/gazebo_plugins/cfg/*.cfg
cd ./src/roscpp_core/roscpp_serialization/include/ros
rm serialization.h
wget https://raw.githubusercontent.com/ros/roscpp_core/72ce04f8b2849e0e4587ea4d598be6ec5d24d8ca/roscpp_serialization/include/ros/serialization.h
cd ../../../../..
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
rosdep update
sed -i 's/${OMPL_LIBRARIES}/\/usr\/lib\/aarch64-linux-gnu\/libompl.so/' ./src/moveit/moveit_planners_ompl/ompl_interface/CMakeLists.txt
sudo cp -r /usr/include/ompl-1.5/ompl/ /usr/include/ompl/ 
cd src 
rm -rf actionlib
git clone -b noetic-devel https://github.com/FRC900/actionlib.git
rm -rf realtime_tools
git clone -b fix_non_realtime https://github.com/FRC900/realtime_tools
cd ..
catkin config --install --install-space /opt/ros/noetic -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3

sudo bash
pip3 install 'numpy<2.0.0'
catkin build -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DPYTHON_EXECUTABLE=/usr/bin/python3 catkin 
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages
#devel/env.sh catkin build -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -Wno-deprecated-copy -Wno-nonnull -Wno-float-conversion -Wno-class-memaccess -Wno-register -Wno-deprecated-copy -Wno-deprecated-enum-enum-conversion -Wno-deprecated-declarations -DNON_POLLING -ftrack-macro-expansion=0 -fno-var-tracking-assignments" -DPYTHON_EXECUTABLE=/usr/bin/python3 dynamic_reconfigure
#cp -r /opt/ros/noetic/lib/python3.10/site-packages/dynamic_reconfigure/* /opt/ros/noetic/local/lib/python3.10/dist-packages/dynamic_reconfigure
# Build this with a lower number of jobs to prevent running out of memory
devel/env.sh catkin build -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -Wno-deprecated-copy -Wno-nonnull -Wno-float-conversion -Wno-class-memaccess -Wno-register -Wno-deprecated-copy -Wno-deprecated-enum-enum-conversion -Wno-deprecated-declarations -DNON_POLLING -ftrack-macro-expansion=0 -fno-var-tracking-assignments" -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 eigenpy
devel/env.sh catkin build -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -Wno-deprecated-copy -Wno-nonnull -Wno-float-conversion -Wno-class-memaccess -Wno-register -Wno-deprecated-copy -Wno-deprecated-enum-enum-conversion -Wno-deprecated-declarations -DNON_POLLING -ftrack-macro-expansion=0 -fno-var-tracking-assignments" -DPYTHON_EXECUTABLE=/usr/bin/python3
rsync -avz /opt/ros/noetic/lib/python3.10/site-packages/ /opt/ros/noetic/local/lib/python3.10/dist-packages/
### Be sure to exit sudo here

