#!/bin/bash

set -e

trt_version=$(echo /usr/lib/aarch64-linux-gnu/libnvinfer.so.? | cut -d '.' -f 3)

chip_id=$(cat /sys/module/tegra_fuse/parameters/tegra_chip_id)
case ${chip_id} in
  "33" )  # Nano and TX1
    cuda_compute=5.3
    local_resources=2048.0,1.0,1.0
    ;;
  "24" )  # TX2
    cuda_compute=6.2
    local_resources=6144.0,6.0,1.0
    ;;
  "25" )  # AGX Xavier
    cuda_compute=7.2
    local_resources=8192.0,16.0,1.0
    ;;
  * )     # default
    cuda_compute=5.3,6.2,7.2
    local_resources=2048.0,1.0,1.0
    ;;
esac

folder=/mnt/900_2/src
mkdir -p $folder

# Create swapfile
sudo fallocate -l 8G $folder/../swapfile
sudo chmod 600 $folder/../swapfile
sudo mkswap $folder/../swapfile
sudo swapon $folder/../swapfile

if (/bin/false); then
if python -m pip list | grep tensorflow > /dev/null; then
  echo "ERROR: tensorflow is installed already"
  exit
fi

if ! which bazel > /dev/null; then
  echo "ERROR: bazel has not been installled"
  exit
fi

echo "** Install requirements"
sudo apt-get install -y libhdf5-serial-dev hdf5-tools
sudo python -m pip install -U pip six numpy wheel setuptools mock h5py
sudo python -m pip install -U keras_applications
sudo python -m pip install -U keras_preprocessing
fi

export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH

echo "** Download and patch tensorflow-1.15.3"
pushd $folder
if [ ! -f tensorflow-1.15.3.tar.gz ]; then
  wget https://github.com/tensorflow/tensorflow/archive/v1.15.3.tar.gz -O tensorflow-1.15.3.tar.gz
fi
#tar xzvf tensorflow-1.15.3.tar.gz
cd tensorflow-1.15.3

patch -N -p1 < /home/ubuntu/2020RobotCode/tensorflow-1.15.3.patch && echo "tensorflow-1.15.0 source tree appears to be patched already.  Continue..."
# Replace cudnn.h with cudnn_version.h in third_party/gpus/find_cuda_config.py
# remove ./third_party/nccl/build_defs.bzl.tpl:            "--bin2c-path=%s" % bin2c.dirname,

echo "** Configure and build tensorflow-1.15.3"
export TMP=/mnt/900_2/tmp
PYTHON_BIN_PATH=$(which python) \
PYTHON_LIB_PATH=$(python -c 'import site; print(site.getsitepackages()[0])') \
TF_CUDA_PATHS=/usr,/usr/local/cuda \
TF_CUDA_COMPUTE_CAPABILITIES=${cuda_compute} \
TF_CUDA_VERSION=10.2 \
TF_CUDA_CLANG=0 \
TF_CUDNN_VERSION=8.0.0 \
TF_TENSORRT_VERSION=${trt_version} \
CUDA_TOOLKIT_PATH=/usr/local/cuda \
CUDNN_INSTALL_PATH=/usr \
TENSORRT_INSTALL_PATH=/usr/lib/aarch64-linux-gnu \
TF_NEED_IGNITE=0 \
TF_ENABLE_XLA=0 \
TF_NEED_OPENCL_SYCL=0 \
TF_NEED_COMPUTECPP=0 \
TF_NEED_ROCM=0 \
TF_NEED_CUDA=1 \
TF_NEED_TENSORRT=1 \
TF_NEED_OPENCL=0 \
TF_NEED_MPI=0 \
GCC_HOST_COMPILER_PATH=$(which gcc) \
CC_OPT_FLAGS="-march=native -fvect-cost-model" \
TF_SET_ANDROID_WORKSPACE=0 \
NCCL_INSTALL_PATH=/usr \
    ./configure
bazel build --config=opt \
	    --config=cuda \
	    --local_resources=${local_resources} \
            //tensorflow/tools/pip_package:build_pip_package
bazel-bin/tensorflow/tools/pip_package/build_pip_package wheel/tensorflow_pkg

echo "** Install tensorflow-1.15.3"
sudo python -m pip install wheel/tensorflow_pkg/tensorflow-1.15.3-*.whl

popd
python -c "import tensorflow as tf; print('tensorflow version: %s' % tf.__version__)"

echo "** Build and install tensorflow-1.15.3 successfully"
