#!/usr/bin/perl
#
# Script used to install RoboRIO ipk packages into a cross-root development
# environment.  
#
use strict;
use File::Temp;
use Cwd;

sub install_package
{
	my $url = @_[0];

	my $oldcwd = getcwd;
	# Create a temporary directory
	my $dirname = File::Temp->newdir;
    # Go to the temporary directory
	chdir $dirname;
	`wget $url`;

	if ($? != 0)
	{
		print("****** Error - could not download $url\n");
		return;
	}
	# ipk files are an ar archive. Inside is a file
	# called data.tar.gz which is the actual contents
	# that have to be extracted. Put them in the
	# /usr/arm-frc-linux-gnueabi sysroot directory
	if (rindex($url, "/") != -1)
	{
		my $filename = substr $url, rindex($url, "/")+1;
		`ar xv $filename`;
		print `tar xzvf data.tar.gz -C ~/frc2019/roborio/arm-frc2019-linux-gnueabi`;
	}
	
	chdir $oldcwd;
}

# These are all needed to build ROS from source for the RoboRIO.  They
# might not all be needed to build programs against the ROS code ...
# some are static libs which get linked into ROS programs.  Still,
# be safe rather than sorry since disk space isn't at a huge premium 
# on our development systems.

my $base_uri = "http://download.ni.com/ni-linux-rt/feeds/2018/arm/cortexa9-vfpv3/";

install_package($base_uri . "lz4-dev_131+git0+d86dc91677-r0.16_cortexa9-vfpv3.ipk");
install_package($base_uri . "lz4-staticdev_131+git0+d86dc91677-r0.16_cortexa9-vfpv3.ipk");
install_package($base_uri . "lz4_131+git0+d86dc91677-r0.16_cortexa9-vfpv3.ipk");
install_package($base_uri . "libcurl4_7.53.1-r0.4_cortexa9-vfpv3.ipk");
install_package($base_uri . "curl-dev_7.53.1-r0.4_cortexa9-vfpv3.ipk");
install_package($base_uri . "curl-staticdev_7.53.1-r0.4_cortexa9-vfpv3.ipk");
install_package($base_uri . "curl_7.53.1-r0.4_cortexa9-vfpv3.ipk");
install_package($base_uri . "boost-dev_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "boost-serialization_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "boost-staticdev_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "boost-test_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "boost_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-atomic1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-chrono1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-date-time1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-filesystem1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-graph1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-iostreams1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-program-options1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-regex1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-signals1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-system1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libboost-thread1.63.0_1.63.0-r1.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libeigen-dev_3.2.8-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libeigen_3.2.8-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libpython2_2.7.13-r1.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "python-core_2.7.13-r1.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "python-dev_2.7.13-r1.13_cortexa9-vfpv3.ipk");

install_package($base_uri . "bzip2-dev_1.0.6-r5.451_cortexa9-vfpv3.ipk");
install_package($base_uri . "bzip2-staticdev_1.0.6-r5.451_cortexa9-vfpv3.ipk");
install_package($base_uri . "bzip2_1.0.6-r5.451_cortexa9-vfpv3.ipk");
install_package($base_uri . "libbz2-1_1.0.6-r5.451_cortexa9-vfpv3.ipk");

install_package($base_uri . "cmake-dev_3.7.2-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "cmake_3.7.2-r0.2_cortexa9-vfpv3.ipk");

install_package($base_uri . "libxml2-dev_2.9.4-r0.150_cortexa9-vfpv3.ipk");
install_package($base_uri . "libxml2-staticdev_2.9.4-r0.150_cortexa9-vfpv3.ipk");
install_package($base_uri . "libxml2_2.9.4-r0.150_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgnutls-bin_3.5.9-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgnutls-dev_3.5.9-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgnutls-openssl27_3.5.9-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgnutls30_3.5.9-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgnutlsxx28_3.5.9-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunistring-dev_0.9.7-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunistring-dev_0.9.7-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunistring-staticdev_0.9.7-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunistring2_0.9.7-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "nettle-dev_3.3-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "nettle-staticdev_3.3-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "nettle_3.3-r0.6_cortexa9-vfpv3.ipk");

install_package($base_uri . "libgmp-dev_6.1.2-r0.23_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgmp-staticdev_6.1.2-r0.23_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgmp10_6.1.2-r0.23_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgmpxx4_6.1.2-r0.23_cortexa9-vfpv3.ipk");

install_package($base_uri . "libz-dev_1.2.11-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "libz-staticdev_1.2.11-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "libz1_1.2.11-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "protobuf-dev_3.1.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "protobuf-staticdev_3.1.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "protobuf_3.1.0-r0.2_cortexa9-vfpv3.ipk");

install_package($base_uri . "gflags-bash-completion_2.2.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "gflags-dbg_2.2.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "gflags-dev_2.2.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "gflags_2.2.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libglog-dev_0.3.4-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libglog-staticdev_0.3.4-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libglog0_0.3.4-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunwind-dev_1.1+git0+bc8698fd7e-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunwind-dev_1.1+git0+bc8698fd7e-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunwind-staticdev_1.1+git0+bc8698fd7e-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libunwind_1.1+git0+bc8698fd7e-r0.2_cortexa9-vfpv3.ipk");

# OpenCV - needed for WPILib
install_package($base_uri . "libopencv-calib3d-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-calib3d3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-core-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-core3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-features2d-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-features2d3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-flann-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-flann3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-highgui-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-highgui3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-imgcodecs-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-imgcodecs3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-imgproc-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-imgproc3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-ml-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-ml3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-objdetect-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-objdetect3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-photo-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-photo3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-shape-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-shape3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-stitching-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-stitching3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-superres-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-superres3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-video-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-video3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-videoio-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-videoio3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-videostab-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "libopencv-videostab3.2_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "opencv-apps_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "opencv-dbg_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "opencv-dev_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "opencv-samples_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");
install_package($base_uri . "opencv_3.2+git0+70bbf17b13-r0.5_cortexa9-vfpv3.ipk");

install_package($base_uri . "tbb-dev_20170412+0+a2cfdfe946-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "tbb_20170412+0+a2cfdfe946-r0.2_cortexa9-vfpv3.ipk");

# rsync packages needed for deployment script
install_package($base_uri . "libacl1_2.2.52-r0.126_cortexa9-vfpv3.ipk");
install_package($base_uri . "rsync_3.1.2-r0.10_cortexa9-vfpv3.ipk");

install_package($base_uri . "libidn11_1.33-r0.6_cortexa9-vfpv3.ipk");

install_package($base_uri . "libusb-1.0-dev_1.0.21-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libusb-1.0-0_1.0.21-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libusb-1.0-staticdev_1.0.21-r0.6_cortexa9-vfpv3.ipk");

install_package($base_uri . "libgnutls-openssl27_3.5.9-r0.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-conf_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-dbg_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-dev_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-doc_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-engines_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-misc_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl-staticdev_1.0.2k-r0.13_cortexa9-vfpv3.ipk");
install_package($base_uri . "openssl_1.0.2k-r0.13_cortexa9-vfpv3.ipk");

# Various gpg stuff from rosbag
install_package($base_uri . "gpgme-dev_1.8.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "gpgme_1.8.0-r0.2_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgpg-error-dev_1.26-r1.6_cortexa9-vfpv3.ipk");
install_package($base_uri . "libgpg-error0_1.26-r1.6_cortexa9-vfpv3.ipk");
linstall_package($base_uri . "ibassuan-dev_2.4.3-r0.2_cortexa9-vfpv3.ipk");
linstall_package($base_uri . "ibassuan0_2.4.3-r0.2_cortexa9-vfpv3.ipk");
