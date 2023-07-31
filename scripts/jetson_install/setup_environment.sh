# Script to setup Jetson Xavier NX environment. Probably would also work
# with slight modifications on other Jetson hardware

#install basic dependencies
sudo apt-add-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update
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
    gcc-11 \
    g++-11 \
    gdb \
    gfortran \
    git \
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
    libgoogle-glog-dev \
    libgoogle-perftools-dev \
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
wget --no-check-certificate https://download.stereolabs.com/zedsdk/4.0/l4t35.3/jetsons
chmod 755 jetsons
./jetsons
rm ./jetsons
rm -rf /home/ubuntu/.local/lib/python3.8/site-packages/numpy

#mount and setup autostart script
sudo mkdir /mnt/900_2
cd
git clone https://github.com/FRC900/2023RobotCode.git
cd ~/2023RobotCode

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
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr2023.list "https://deb.ctr-electronics.com/ctr2023.list"
sudo apt update
sudo apt install -y canivore-usb

sudo bash -c "echo \"[Match\"] >> /etc/systemd/network/80-can.network"
sudo bash -c "echo \"Name=can0\" >> /etc/systemd/network/80-can.network"
sudo bash -c "echo \\"" >> /etc/systemd/network/80-can.network"
sudo bash -c "echo \"[CAN\"] >> /etc/systemd/network/80-can.network"
sudo bash -c "echo \"BitRate=1000K\" >> /etc/systemd/network/80-can.network"
sudo systemctl enable systemd-networkd
sudo systemctl restart systemd-networkd

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

#sudo bash -c "echo NTP=us.pool.ntp.org >> /etc/systemd/timesyncd.conf"
#sudo bash -c "echo FallbackNTP=ntp.ubuntu.com >> /etc/systemd/timesyncd.conf"
sudo cp ~/2023RobotCode/scripts/jetson_install/ntp.conf /etc/ntp.conf
# On 10.9.0.9, uncommment last few lines of ntp.conf

sudo cp ~/2023RobotCode/scripts/jetson_setup/hwrtc.service /etc/systemd/system
sudo chmod 664 /etc/systemd/system/hwrtc.service
# The ntp config should read from hwrtc -> system clock if it can't
# get to the internet to read from pool time servers
sudo systemctl enable hwrtc

    
# and keys for connections to Rio
mkdir -p ~/.ssh
cd ~/.ssh
tar -xjf ~/2023RobotCode/scripts/jetson_setup/jetson_dot_ssh.tar.bz2
chmod 640 authorized_keys
cd ~
chmod 700 .ssh

sudo mkdir -p /root/.ssh
sudo tar -xjf /home/ubuntu/2023RobotCode/scripts/jetson_setup/jetson_dot_ssh.tar.bz2 -C /root/.ssh
sudo chown root:root /root/.ssh/*
sudo chmod 640 /root/.ssh/authorized_keys
sudo chmod 700 /root/.ssh

cd ~/2023RobotCode/scripts
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
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/ctre
mkdir -p /home/ubuntu/ctre
cd /home/ubuntu/ctre
python3 /home/ubuntu/2023RobotCode/scripts/jetson_install/download_maven.py https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6And5-frc2023-latest.json 
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include
find /home/ubuntu/ctre -name \*headers\*zip | grep -v debug | xargs -n 1 unzip -o
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/ctre
find /home/ubuntu/ctre -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
rm -rf /home/ubuntu/ctre

cd /home/ubuntu
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/4.0.433/navx-cpp-4.0.433-headers.zip
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include/navx
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include/navx
unzip -o /home/ubuntu/navx-cpp-4.0.433-headers.zip
rm /home/ubuntu/navx-cpp-4.0.433-headers.zip
cd /home/ubuntu
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/4.0.433/navx-cpp-4.0.433-linuxathena.zip
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/navx
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/navx
unzip -o /home/ubuntu/navx-cpp-4.0.433-linuxathena.zip
rm /home/ubuntu/navx-cpp-4.0.433-linuxathena.zip
cd /home/ubuntu
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/4.0.433/navx-cpp-4.0.433-linuxathenastatic.zip
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/navx
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/navx
unzip -o /home/ubuntu/navx-cpp-4.0.433-linuxathenastatic.zip
rm /home/ubuntu/navx-cpp-4.0.433-linuxathenastatic.zip

# And Rev sparkmax stuff
cd /home/ubuntu
mkdir sparkmax
cd sparkmax
python3 /home/ubuntu/2023RobotCode/scripts/jetson_install/download_maven.py https://software-metadata.revrobotics.com/REVLib-2023.json
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include
find /home/ubuntu/sparkmax -name \*header\*zip | grep -v debug | xargs -n 1 unzip -o
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/rev
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/rev
find /home/ubuntu/sparkmax -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
rm -rf /home/ubuntu/sparkmax

# Install wpilib headers by copying them from the local maven dir
cd /home/ubuntu
wget https://github.com/wpilibsuite/allwpilib/releases/download/v2023.4.3/WPILib_Linux-2023.4.3.tar.gz
mkdir -p /home/ubuntu/wpilib/2023
cd /home/ubuntu/wpilib/2023
tar -xzf /home/ubuntu/WPILib_Linux-2023.4.3.tar.gz
tar -xzf WPILib_Linux-2023.4.3/WPILib_Linux-2023.4.3-artifacts.tar.gz
rm /home/ubuntu/WPILib_Linux-2023.4.3.tar.gz
cd /home/ubuntu/wpilib/2023/tools
python3 ToolsUpdater.py
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/wpilib
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/wpilib
find ../../../.. -name \*athena\*zip | grep -v debug | xargs -n1 unzip -o
mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include/wpilib
cd /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include/wpilib
find ../../../.. -name \*headers\*zip | xargs -n1 unzip -o
rm -rf /home/ubuntu/wpilib/2023/maven /home/ubuntu/wpilib/2023/jdk /home/ubuntu/wpilib/2023/WPILib_Linux-2023.4.3 /home/ubuntu/wpilb/2023/utility /home/ubuntu/wpilib/2023/tools /home/ubuntu/wpilib/2023/documentation /home/ubuntu/wpilib/2023/installUtils /home/ubuntu/wpilib/2023/vsCodeExtensions
sed -i -e 's/   || defined(__thumb__) \\/   || defined(__thumb__) \\\n   || defined(__aarch64__) \\/' /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/include/wpilib/FRC_FPGA_ChipObject/fpgainterfacecapi/NiFpga.h
find ~/wpilib -name \*.debug | xargs rm -rf
find ~/wpilib -name athena | xargs rm -rf
find ~/wpilib -name x86-64| xargs rm -rf
find ~/wpilib -name raspbian | xargs rm -rf

# Set up prereqs for deploy script
mv ~/2023RobotCode ~/2023RobotCode.orig
ln -s ~/2023RobotCode.orig ~/2023RobotCode
mkdir -p ~/2023RobotCode.prod/zebROS_ws
mkdir -p ~/2023RobotCode.dev/zebROS_ws

sudo mkdir -p /usr/local/zed/settings
sudo chmod 755 /usr/local/zed/settings
sudo cp ~/2023RobotCode/scripts/jetson_install/calibration_files/*.conf /usr/local/zed/settings
sudo chmod 644 /usr/local/zed/settings/*

cp ~/2023RobotCode/.vimrc ~/2023RobotCode/.gvimrc ~
sudo cp ~/2023RobotCode/kjaget.vim /usr/share/vim/vim81/colors

cd
wget https://github.com/git-lfs/git-lfs/releases/download/v3.3.0/git-lfs-linux-arm64-v3.3.0.tar.gz
mkdir git-lfs-install
cd git-lfs-install
tar -xzf ../git-lfs-linux-arm64-v3.3.0.tar.gz
cd git-lfs-3.3.0
sudo ./install.sh
cd
rm -rf git-lfs-linux-arm64-v3.3.0.tar.gz git-lfs-install
git lfs install
cd ~/2023RobotCode
git lfs pull

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson NX"

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

sudo ln -s /usr/include/opencv4 /usr/include/opencv

echo "source /home/ubuntu/2023RobotCode/zebROS_ws/command_aliases.sh" >> /home/ubuntu/.bashrc

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

# Install tensorflow on Jetson
sudo pip3 install -U pip testresources setuptools==49.6.0
sudo pip3 install nvidia-pyindex
sudo pip3 install uff graphsurgeon
sudo pip3 install --ignore-installed -U cython
sudo pip3 install -U --no-deps numpy==1.21.1 future==0.18.2 mock==3.0.5 h5py==3.6.0 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 'protobuf<4.0.0,>=3.6.1' pybind11 pkgconfig
#sudo env H5PY_SETUP_REQUIRES=0 pip3 install -U h5py==3.1.0
sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v502 tensorflow==1.15.5+nv22.12
sudo pip3 install matplotlib

# Fails with error install grpcio?
#sudo pip3 install protobuf-compiler
sudo apt install -y protobuf-compiler
cd /home/ubuntu
git clone https://github.com/tensorflow/models.git
cd models
git submodule update --init --recursive
cd /home/ubuntu/models/research
git checkout c787baad4fcf3e008107be0662a4138194b24522^
protoc object_detection/protos/*.proto --python_out=.
sudo pip3 install --ignore-installed .
cd slim
sudo pip3 install --ignore-installed .



# Install cuda manually for now?
#wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/cuda-keyring_1.0-1_all.deb
#sudo dpkg -i cuda-keyring_1.0-1_all.deb
#sudo apt-get update
#sudo apt-get -y install cuda nvidia-cudnn8 nvidia-cudnn8-dev libnvinfer-dev libnvinfer-plugin-dev python3-libnvinfer-dev

echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/cuda-12.0/compat" >> /home/ubuntu/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/cuda-11.4/targets/aarch64-linux/lib" >> /home/ubuntu/.bashrc

cd
export PATH=$PATH:/usr/local/cuda/bin
git clone https://github.com/NVIDIA/TensorRT.git
cd TensorRT
git checkout 8.5.2
git submodule update --init --recursive
mkdir build
cd build
cmake -GNinja -DBUILD_PARSERS=OFF -DBUILD_SAMPLES=OFF -DCMAKE_CXX_STANDARD=17 ..
cmake -GNinja -DBUILD_PARSERS=OFF -DBUILD_SAMPLES=OFF -DGPU_ARCHS="72 87" -DCUDA_VERSION=11.4 -DCUDNN_VERSION=8.6.0 ..

sudo ninja install

sudo -H bash
export PATH=$PATH:/usr/local/cuda/bin
export CUDA_ROOT=/usr/local/cuda
pip3 install pycuda

sudo apt install -y  libsoup2.4-dev libjson-glib-dev libgstrtspserver-1.0-dev
cd
git clone https://github.com/FRC900/jetson-utils.git
cd jetson-utils
git checkout -t origin/ssd_norm
mkdir build
cd build
cmake -GNinja ..
sudo ninja install

# Ultralytics YOLOv8 prereqs here
sudo python3 -m pip install --no-cache-dir --upgrade pascal_voc
sudo python3 -m pip install --no-cache-dir --upgrade 'matplotlib>=3.2.2'
sudo python3 -m pip install --no-cache-dir --upgrade 'opencv-python>=4.6.0'
sudo python3 -m pip install --no-cache-dir --upgrade 'Pillow>=7.1.2'
sudo python3 -m pip install --no-cache-dir --upgrade 'PyYAML>=5.3.1'
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


export TORCHVISION_VERSION=v0.15.0
export TORCH_CUDA_ARCH_LIST=7.2;8.7 
sudo apt install -y libjpeg-dev libpng-dev zlib1g-dev
git clone --branch ${TORCHVISION_VERSION} --recursive --depth=1 https://github.com/pytorch/vision torchvision
cd torchvision
git checkout ${TORCHVISION_VERSION}
sudo python3 setup.py install
cd ..
sudo rm -rf torchvision

sudo python3 -m pip install --no-cache-dir --upgrade ultralytics
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
#echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/rev/linux/aarm64/shared:/usr/local/lib" >> /home/ubuntu/.bashrc

# Install pyserial (for 2023 intake reader)
sudo pip3 install pyserial
