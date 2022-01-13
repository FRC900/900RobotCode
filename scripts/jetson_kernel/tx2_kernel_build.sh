# Kernel module build steps for TX2 : https://gist.github.com/sauhaardac/9d7a82c23e4b283a1e79009903095655
# Not needed unless Jetpack is updated with a new kernel version and modules
# for a given kernel version aren't already built
#
# This is a PR into the jetsonhacks repo of the same name, will likely be
# gone the next time anyone looks for it
# Also, this says Xavier but it really seems to mean Jetpack 4.x, in the
# case of this particular PR, 4.2. Which is what we're using for the TX2
# systems as well.
cd
git clone https://github.com/klapstoelpiloot/buildLibrealsense2Xavier.git 
# Note - need to switch back to the default linker to build the kernel image
cd
#wget https://developer.nvidia.com/embedded/r32-2-3_Release_v1.0/Sources/T186/public_sources.tbz2 
wget https://developer.nvidia.com/embedded/dlc/r32-3-1_Release_v1.0/Sources/T186/public_sources.tbz2
tar -xf public_sources.tbz2 Linux_for_Tegra/source/public/kernel_src.tbz2
mkdir jetson_kernel
cd jetson_kernel
tar -xf ../Linux_for_Tegra/source/public/kernel_src.tbz2
patch -p0 < ~/2022RobotCode/scripts/jetson_kernel/patch_j120_l4t32.2.3.txt

## Apply realsense patches to modules
cd ~/jetson_kernel/kernel/kernel-4.9
patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-camera-formats_ubuntu-bionic-Xavier-4.9.140.patch 
patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-metadata-ubuntu-bionic-Xavier-4.9.140.patch
patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-hid-ubuntu-bionic-Xavier-4.9.140.patch
patch -p1 < ~/realsense_src/librealsense-2.31.0/scripts/realsense-powerlinefrequency-control-fix.patch
# These are for the librealsense code, but don't actually seem to be used
#patch -p1 < ~/buildLibrealsense2Xavier/patches/model-views.patch
#patch -p1 < ~/buildLibrealsense2Xavier/patches/incomplete-frame.patch
rm -rf ~/buildLibrealsense2Xavier

# turn on various config settings needed for USB tty, realsense, nvme, etc
zcat /proc/config.gz > .config
bash scripts/config --file .config \
	--set-str LOCALVERSION -tegra \
	--enable IIO_BUFFER \
	--enable IIO_KFIFO_BUF \
	--module IIO_TRIGGERED_BUFFER \
	--enable IIO_TRIGGER \
	--set-val IIO_CONSUMERS_PER_TRIGGER 2 \
	--module HID_SENSOR_IIO_COMMON \
	--module HID_SENSOR_IIO_TRIGGER \
	--module HID_SENSOR_HUB \
	--module HID_SENSOR_ACCEL_3D \
	--module HID_SENSOR_GYRO_3D \
	--module USB_ACM \
	--module CAN_GS_USB \
	--module JOYSTICK_XPAD \
	--enable CONFIG_BLK_DEV_NVME

make -j6 clean
make -j6 prepare
make -j6 modules_prepare
make -j6 Image zImage
make -j6 modules
sudo make -j6 modules_install
make -j6 dtbs

sudo depmod -a

tar -C ~/jetson_kernel/kernel/kernel-4.9 -cjf ~/j120_hardware_dtb_l4t32-2-3-1.tbz2 \
	`find ~/jetson_kernel -name tegra186-quill-p3310-1000-a00-00-base.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra186-quill-p3310-1000-as-0888.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra186-quill-p3310-1000-c03-00-base.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra186-quill-p3489-0888-a00-00-base.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra186-quill-p3489-1000-a00-00-ucm1.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra186-quill-p3489-1000-a00-00-ucm2.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra194-p2888-0001-p2822-0000.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name tegra194-p2888-0001-p2822-0000-maxn.dtb | grep -v _ddot_` \
	`find ~/jetson_kernel -name Image | grep -v _ddot_` \
	`find ~/jetson_kernel -name zImage | grep -v _ddot_` 


tar -cjf ~/l4t32-2-3-1-modules.tbz2 \
	`find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-iio-common.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-trigger.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-hub.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-accel-3d.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-gyro-3d.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name cdc-acm.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name gs_usb.ko` \
	`find /lib/modules/4.9.140-tegra/kernel -name xpad.ko`

# make -j6 M=drivers/usb/class
# make -j6 M=drivers/usb/serial
# make -j6 M=drivers/net/can
# make -j6 M=net/can
# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
# sudo cp drivers/usb/class/cp210x-acm.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x-acm.ko
# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
# sudo cp drivers/usb/serial/cdc-acm.ko /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
# sudo cp drivers/net/can/usb/gs_usb.ko /lib/modules/`uname -r`/kernel/drivers/net/can/usb

# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/joystick
# sudo cp xpad.ko /lib/modules/`uname -r`/kernel/drivers/joystick/xpad.ko
# sudo depmod -a

