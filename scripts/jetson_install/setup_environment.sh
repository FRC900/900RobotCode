# Script to setup Jetson Xavier NX environment. Probably would also work
# with slight modifications on other Jetson hardware

#install basic dependencies
sudo apt-add-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-add-repository ppa:borglab/gtsam-release-4.1 -y
sudo apt update

# Keep these the original version to line up with kernel versions supported by arducam?
# TODO : Test updating after installing camera stuff
sudo apt-mark hold nvidia-l4t-bootloader nvidia-l4t-display-kernel nvidia-l4t-initrd nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers nvidia-l4t-tools nvidia-l4t-xusb-firmware
sudo apt -y upgrade

# These are listed 1 package per line to hopefully make git merging easier
# They're also sorted alphabetically to keep packages from being listed multiple times
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
    exfat-utils \
	gcc-10 \
    gcc-11 \
	gcc-13 \
	g++-10 \
    g++-11 \
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
    libclang1-9 \
    libeigen3-dev \
    libflann-dev \
    libgflags-dev \
    libgtsam-dev \
    libgtsam-unstable-dev \
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

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110 --slave /usr/bin/g++ g++ /usr/bin/g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90 --slave /usr/bin/g++ g++ /usr/bin/g++-9
sudo update-alternatives --auto gcc

#TensorRT requires a newer version of cmake than standard apt repos provide
cd
#wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
wget https://github.com/Kitware/CMake/releases/download/v3.27.0/cmake-3.27.0.tar.gz
tar -xf cmake-3.27.0.tar.gz
cd cmake-3.27.0
cmake -GNinja -DCMAKE_BUILD_TYPE:STRING=Release .
sudo ninja install
sudo mv /usr/bin/cmake /usr/bin/cmake.old
sudo ln -s /usr/local/bin/cmake /usr/bin/cmake
cd ..
sudo rm -rf cmake-3.27.0*

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
wget --no-check-certificate https://download.stereolabs.com/zedsdk/4.0/l4t35.4/jetsons
chmod 755 jetsons
./jetsons
rm ./jetsons
rm -rf /home/ubuntu/.local/lib/python3.8/site-packages/numpy

#mount and setup autostart script
sudo mkdir /mnt/900_2
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

sudo apt update
sudo apt install -y canivore-usb
#sudo apt-mark hold canivore-usb

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
sudo cp ~/900RobotCode/kjaget.vim /usr/share/vim/vim81/colors

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson NX"

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

sudo ln -s /usr/include/opencv4 /usr/include/opencv

echo "source /home/ubuntu/900RobotCode/zebROS_ws/command_aliases.sh" >> /home/ubuntu/.bashrc

# Install make 4.3 (>4.2 is required for -flto=jobserver support
cd
wget https://ftp.gnu.org/gnu/make/make-4.3.tar.gz
tar -xf make-4.3.tar.gz
mkdir make-4.3/build
cd make-4.3/build
../configure --prefix=/usr
sudo make -j`nproc --all` install
cd
sudo rm -rf make-4.3*

# Give the ubuntu user dialout permission, which is used by the ADI IMU
sudo adduser ubuntu dialout

git clone https://github.com/VundleVim/Vundle.vim.git /home/ubuntu/.vim/bundle/Vundle.vim
vim +PluginInstall +qall
ln -sf /home/ubuntu/.vim/bundle/vim-ros-ycm/.ycm_extra_conf.py /home/ubuntu/.vim/bundle/vim-ros-ycm/ycm_extra_conf.py
cd /home/ubuntu/.vim/bundle/YouCompleteMe
git fetch origin
git submodule update --init --recursive
python3 ./install.py --clang-completer --system-libclang --ninja

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
wget https://nvidia.box.com/shared/static/mvdcltm9ewdy2d5nurkiqorofz1s53ww.whl -O onnxruntime_gpu-1-15.0-cp38-cp38-linux_aarch64.whl
sudo pip3 install onnxruntime_gpu-1-15.0-cp38-cp38-linux_aarch64.whl
rm onnxruntime_gpu-1-15.0-cp38-cp38-linux_aarch64.whl

# cpu-only version : sudo python3 -m pip install --no-cache-dir --upgrade 'onnxruntime'
# not available sudo python3 -m pip install --no-cache-dir --upgrade onnxruntime-gpu
#sudo python3 -m pip install --no-cache-dir --upgrade nvidia-pyindex
#sudo python3 -m pip install --no-cache-dir --upgrade nvidia-tensorrt

#export PYTORCH_URL=https://nvidia.box.com/shared/static/rehpfc4dwsxuhpv4jgqv8u6rzpgb64bq.whl 
#export PYTORCH_WHL=torch-2.0.0a0+ec3941ad.nv23.2-cp38-cp38-linux_aarch64.whl 
export PYTORCH_URL=https://nvidia.box.com/shared/static/i8pukc49h3lhak4kkn67tg9j4goqm0m7.whl
export PYTORCH_WHL=torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

sudo apt-get install -y libopenblas-base libopenmpi-dev
wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL}
sudo pip3 install --no-cache-dir --verbose ${PYTORCH_WHL}
rm ${PYTORCH_WHL}


export TORCHVISION_VERSION=0.15
export TORCH_CUDA_ARCH_LIST="7.2;8.7" 
sudo apt install -y libjpeg-dev libpng-dev zlib1g-dev
git clone --branch release/${TORCHVISION_VERSION} --recursive --depth=1 https://github.com/pytorch/vision torchvision
cd torchvision
git checkout release/${TORCHVISION_VERSION}
sudo python3 setup.py install
cd ..
sudo rm -rf torchvision

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
