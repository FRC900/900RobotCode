#!/bin/bash
# Script to setup Jetson TX2 environment. Probably would also work
# with slight modifications on other Jetson hardware

#install basic dependencies
#sudo apt-add-repository ppa:ubuntu-toolchain-r/test -y 
sudo apt update
sudo apt -y upgrade

# These are listed 1 package per line to hopefully make git merging easier
# They're also sorted alphabetically to keep packages from being listed multiple times
sudo apt install -y \
    build-essential \
    can-utils \
    ccache \
    chromium-browser \
    cmake \
    cowsay \
    dbus-x11 \
    exfat-fuse \
    exfat-utils \
    gdb \
    gfortran \
    git \
    gstreamer1.0-plugins-* \
    htop \
    libatlas-base-dev \
    libboost-all-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libclang-9-dev \
    libclang1-9 \
    libeigen3-dev \
    libflann-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgoogle-perftools-dev \
    libgpiod-dev \
    libgtk2.0-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libleveldb-dev \
    liblmdb-dev \
    liblua5.3-dev \
	libnlopt-dev \
    libpcl-dev \
    libproj-dev \
    libsnappy-dev \
    libsuitesparse-dev \
    libtinyxml2-dev \
    net-tools \
    ninja-build \
    nmap \
    ntpdate \
    openssh-client \
    pkg-config \
    pyqt5-dev-tools \
    python-dev \
    python-matplotlib \
    python-numpy \
    python-opencv \
    python-pip \
    python-pyqt5 \
    python-pyqtgraph \
    python-scipy \
	python3 \
    qt4-designer \
    rsync \
    software-properties-common \
    terminator \
    unzip \
    v4l-conf \
    v4l-utils \
    vim-gtk \
    wget \
    xfonts-scalable

#TensorRT requires a newer version of cmake than standard apt repos provide
cd
#wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
wget https://github.com/Kitware/CMake/releases/download/v3.19.6/cmake-3.19.6.tar.gz 
tar -xf cmake-3.19.6.tar.gz
cd cmake-3.19.6
cmake -GNinja -DCMAKE_BUILD_TYPE:STRING=Release .
sudo ninja install
sudo mv /usr/bin/cmake /usr/bin/cmake.old
sudo ln -s /usr/local/bin/cmake /usr/bin/cmake
cd ..
sudo rm -rf cmake-3.19.6*

#sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 30 --slave /usr/bin/g++ g++ /usr/bin/g++-10
#sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 90 --slave /usr/bin/g++ g++ /usr/bin/g++-7

#install caffe
# cd
# git clone https://github.com/BVLC/caffe.git
# cd caffe
# mkdir build
# cd build

# if [ "$gpu" == "false" ] ; then
    # cmake -DCPU_ONLY=ON ..
# else
    # cmake -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF ..
# fi

# make -j4 all
#make test
#make runtest
# make -j4 install

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
wget --no-check-certificate https://download.stereolabs.com/zedsdk/3.6/jp45/jetsons
chmod 755 jetsons
./jetsons
rm ./jetsons

#mount and setup autostart script
sudo mkdir /mnt/900_2
cd ~/2020Offseason

# Set up can0 network interface
cd
echo "auto can0" > can0
echo "iface can0 inet manual" >> can0
echo "  pre-up /sbin/ip link set can0 type can bitrate 1000000" >> can0
echo "  up /sbin/ifconfig can0 up" >> can0
echo "  down /sbin/ifconfig can0 down" >> can0
sudo mv can0 /etc/network/interfaces.d

sudo bash -c "echo \"# Modules for CAN interface\" >> /etc/modules"
sudo bash -c "echo can >> /etc/modules"
sudo bash -c "echo can_raw >> /etc/modules"
sudo bash -c "echo can_dev >> /etc/modules"
sudo bash -c "echo gs_usb >> /etc/modules"
#sudo bash -c "echo mttcan >> /etc/modules"

# This shouldn't be the least bit dangerous
#sudo rm /etc/modprobe.d/blacklist-mttcan.conf 

# Disable l4tbridge - https://devtalk.nvidia.com/default/topic/1042511/is-it-safe-to-remove-l4tbr0-bridge-network-on-jetson-xavier-/
# sudo rm /etc/systemd/system/nv-l4t-usb-device-mode.sh /etc/systemd/system/multi-user.target.wants/nv-l4t-usb-device-mode.service
sudo systemctl disable nv-l4t-usb-device-mode.service
sudo systemctl stop nv-l4t-usb-device-mode.service

# Set up ssh host config (add port 5801) 
sudo sed "s/#Port 22/Port 22\nPort 5801/g" /etc/ssh/sshd_config > sshd_config && sudo mv sshd_config /etc/ssh

sudo bash -c "echo NTP=10.9.0.2 >> /etc/systemd/timesyncd.conf"
sudo bash -c "echo FallbackNTP=ntp.ubuntu.com >> /etc/systemd/timesyncd.conf"
    
# and keys for connections to Rio
mkdir -p ~/.ssh
cd ~/.ssh
tar -xjf ~/2020Offseason/scripts/jetson_setup/jetson_dot_ssh.tar.bz2 
chmod 640 authorized_keys
cd ~
chmod 700 .ssh

sudo mkdir -p /root/.ssh
sudo tar -xjf /home/ubuntu/2020Offseason/scripts/jetson_setup/jetson_dot_ssh.tar.bz2 -C /root/.ssh
sudo chmod 640 /root/.ssh/authorized_keys
sudo chmod 700 /root/.ssh

cd ~/2020Offseason/scripts
sudo cp ./jetson_setup/10-local.rules ./jetson_setup/99-gpio.rules /etc/udev/rules.d/
sudo service udev reload
sleep 2
sudo service udev restart

# Clean up Jetson
sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/TensorRT /home/nvidia/libvisionworks*
# Save ~400MB
sudo apt remove --purge -y thunderbird libreoffice-*
# Disable automatic updates
sudo sed -i -e 's/APT::Periodic::Update-Package-Lists "1"/APT::Periodic::Update-Package-Lists "0"/' /etc/apt/apt.conf.d/10periodic

# Install CTRE & navX libs
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include 
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/ctre 
cd /home/ubuntu
wget -e robots=off -U mozilla -r -np http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/ -A "*5.19.4*" -R "md5,sha1,pom,jar,*windows*,*debug*"
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include 
find /home/ubuntu/devsite.ctr-electronics.com -name \*headers\*zip | grep -v debug | xargs -n 1 unzip -o 
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/ctre 
find /home/ubuntu/devsite.ctr-electronics.com -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o 
rm -rf /home/ubuntu/devsite.ctr-electronics.com 

cd /home/ubuntu 
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.400/navx-cpp-3.1.400-headers.zip 
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include/navx 
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include/navx 
unzip -o /home/ubuntu/navx-cpp-3.1.400-headers.zip 
rm /home/ubuntu/navx-cpp-3.1.400-headers.zip 
cd /home/ubuntu 
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.400/navx-cpp-3.1.400-linuxathena.zip 
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/navx 
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/navx 
unzip -o /home/ubuntu/navx-cpp-3.1.400-linuxathena.zip 
rm /home/ubuntu/navx-cpp-3.1.400-linuxathena.zip 
cd /home/ubuntu 
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.400/navx-cpp-3.1.400-linuxathenastatic.zip 
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/navx 
unzip -o /home/ubuntu/navx-cpp-3.1.400-linuxathenastatic.zip 
rm /home/ubuntu/navx-cpp-3.1.400-linuxathenastatic.zip 

# And Rev sparkmax stuff
cd /home/ubuntu
wget http://www.revrobotics.com/content/sw/max/sdk/SPARK-MAX-SDK-v1.5.2.zip
mkdir sparkmax
cd sparkmax
unzip ../SPARK-MAX-SDK-v1.5.2.zip
rm ../SPARK-MAX-SDK-v1.5.2.zip
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/rev
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/rev
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-cpp -name \*athena\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-cpp -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-driver -name \*athena\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-driver -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-cpp -name \*header\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-driver -name \*header\*zip | grep -v debug | xargs -n 1 unzip -o
rm -rf /home/ubuntu/sparkmax

# Install wpilib headers by copying them from the local maven dir
cd /home/ubuntu 
wget https://github.com/wpilibsuite/allwpilib/releases/download/v2021.2.2/WPILib_Linux-2021.2.2.tar.gz
mkdir -p /home/ubuntu/wpilib/2021
cd /home/ubuntu/wpilib/2021
tar -xzf /home/ubuntu/WPILib_Linux-2021.2.2.tar.gz
tar -xzf WPILib_Linux-2021.2.2/WPILib_Linux-2021.2.2-artifacts.tar.gz
rm /home/ubuntu/WPILib_Linux-2021.2.2.tar.gz
cd /home/ubuntu/wpilib/2021/tools
python3 ToolsUpdater.py
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/wpilib
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/lib/wpilib
find ../../../.. -name \*athena\*zip | grep -v debug | xargs -n1 unzip -o
mkdir -p /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include/wpilib
cd /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include/wpilib
find ../../../.. -name \*headers\*zip | xargs -n1 unzip -o
rm -rf /home/ubuntu/wpilib/2021/maven /home/ubuntu/wpilib/frc2021/jdk /home/ubuntu/wpilib/2021/WPILib_Linux-2021.2.2 /home/ubuntu/wpilb2021/utility
sed -i -e 's/   || defined(__thumb__) \\/   || defined(__thumb__) \\\n   || defined(__aarch64__) \\/' /home/ubuntu/wpilib/2021/roborio/arm-frc2021-linux-gnueabi/include/wpilib/FRC_FPGA_ChipObject/fpgainterfacecapi/NiFpga.h

# Set up prereqs for deploy script
mv ~/2020Offseason ~/2020Offseason.orig
ln -s ~/2020Offseason.orig ~/2020Offseason
mkdir -p ~/2020Offseason.prod/zebROS_ws
mkdir -p ~/2020Offseason.dev/zebROS_ws

sudo mkdir -p /usr/local/zed/settings
sudo chmod 755 /usr/local/zed/settings
sudo cp ~/2020Offseason/scripts/jetson_install/calibration_files/*.conf /usr/local/zed/settings
sudo chmod 644 /usr/local/zed/settings/*

cp ~/2020Offseason/.vimrc ~/2020Offseason/.gvimrc ~
sudo cp ~/2020Offseason/kjaget.vim /usr/share/vim/vim80/colors

cd &&\
    wget https://github.com/git-lfs/git-lfs/releases/download/v2.13.1/git-lfs-linux-arm64-v2.13.1.tar.gz &&\
	mkdir git-lfs-install &&\
	cd git-lfs-install &&\
	tar -xzf ../git-lfs-linux-arm64-v2.13.1.tar.gz &&\
	sudo ./install.sh &&\
	cd &&\
	rm -rf git-lfs-linux-arm64-v2.13.1.tar.gz git-lfs-install &&\
	git lfs install &&\
	cd ~/2020Offseason &&\
	git lfs pull

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson NX"

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

sudo ln -s /usr/include/opencv4 /usr/include/opencv

echo "source /home/ubuntu/2020Offseason/zebROS_ws/command_aliases.sh" >> /home/ubuntu/.bashrc

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
git checkout legacy-py2
git submodule update --init --recursive
python2.7 ./install.py --clang-completer --system-libclang --ninja 

# Install tensorflow on Jetson
#sudo apt update
#sudo apt install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran python3-pip python3-opencv python3-pil python3-matplotlib
#sudo pip3 install --install-option="--jobs=6" -U pip testresources setuptools
#sudo pip3 install --install-option="--jobs=6" -U numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 futures pybind11
#sudo pip3 install --install-option="--jobs=6" --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow==1.15.2+nv20.04

#Build working version of protobuf from source
mkdir -p ~/src
cd ~/src
sudo apt-get install -y autoconf libtool
if [ ! -f protobuf-python-3.12.3.zip ]; then
  wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protobuf-all-3.12.3.zip
fi
if [ ! -f protoc-3.12.3-linux-aarch_64.zip ]; then
  wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protoc-3.12.3-linux-aarch_64.zip
fi

echo "** Install protoc"
unzip protobuf-all-3.12.3.zip
unzip protoc-3.12.3-linux-aarch_64.zip -d protoc-3.12.3
sudo cp protoc-3.12.3/bin/protoc /usr/local/bin/protoc
echo "** Build and install protobuf-3.12.3 libraries"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
cd protobuf-3.12.3/
#cd cmake
#mkdir build
#cd build
#cmake -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF ..

wget https://raw.githubusercontent.com/openembedded/meta-openembedded/master/meta-oe/recipes-devtools/protobuf/protobuf/0001-protobuf-fix-configure-error.patch
patch < 0001-protobuf-fix-configure-error.patch
./autogen.sh
./configure --prefix=/usr/local

sed -i 's/-g -O2/-g -O2 -fPIC/' Makefile
find . -name \*.map | xargs -n1 sed -i 's/    };/    };\n    scc_info_*;\n    descriptor_table_*;/' 
LDFLAGS="-pthread -Wl,--no-as-needed" make -j`nproc --all`
#make -j`nproc --all` check
sudo LDFLAGS="-pthread -Wl,--no-as-needed" make -j`nproc --all` install
sudo ldconfig
#cd ../..

echo "** Update python protobuf module"
# remove previous installation of python protobuf module
sudo python -m pip uninstall -y protobuf
sudo python -m pip install Cython
cd python/
# force compilation with c++11 standard
python setup.py build --cpp_implementation
# Probably fails?
python setup.py test --cpp_implementation
sudo python setup.py install --cpp_implementation
cd
sudo rm -rf src

cd ~/2020Offseason/scripts/jetson_install
sudo apt-get install -y libhdf5-serial-dev hdf5-tools
sudo dpkg -i libnccl*arm64.deb
sudo python -m pip install --upgrade pip six numpy wheel setuptools mock h5py
sudo python -m pip install --upgrade keras_applications
sudo python -m pip install --upgrade keras_preprocessing
sudo python -m pip install tensorflow-1.15.3-*.whl

cd /home/ubuntu
git clone https://github.com/tensorflow/models.git
cd models
git submodule update --init --recursive
cd /home/ubuntu/models/research
git checkout c787baad4fcf3e008107be0662a4138194b24522^
protoc object_detection/protos/*.proto --python_out=.
sudo python -m pip install --ignore-installed .
cd slim
sudo python -m pip install --ignore-installed .

cd
export PATH=$PATH:/usr/local/cuda/bin
git clone https://github.com/NVIDIA/TensorRT.git 
cd TensorRT 
git submodule update --init --recursive 
git checkout 20.11
mkdir build 
cd build 
cmake -GNinja -DBUILD_PARSERS=OFF -DBUILD_SAMPLES=OFF .. 
ninja

echo "export PATH=$PATH:/home/ubuntu/.local/bin:/home/ubuntu/tensorflow_workspace/tools:/usr/local/cuda/bin" >> /home/ubuntu/.bashrc

# Set up Gold linker - speed up libPCL links
# Do this after building protoc, since that fails with ld.gold
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.gold" 20
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.bfd" 10

echo 0 | sudo update-alternatives --config ld

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache
