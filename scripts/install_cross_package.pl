#!/usr/bin/perl
#
# Script used to install RoboRIO ipk packages into a cross-root development
# environment.  
#
use strict;
use File::Temp;
use Cwd;

my %package_full_name  = {};
my %package_requires   = {};
my %packages_installed = {};
my %package_size       = {};
my $total_bytes        = 0;

#my $base_url = "http://download.ni.com/ni-linux-rt/feeds/2019/arm/cortexa9-vfpv3/";
#my $install_dir = "~/wpilib/2020/roborio/arm-frc2020-linux-gnueabi";
my $base_url = $ARGV[0];
my $install_dir = $ARGV[1];
sub install_package
{
	if (!defined $package_full_name{@_[0]})
	{
		print STDERR "install_package : Didn't read package @_[0] from Packages file\n";
		return;
	}

	return if (defined $packages_installed{@_[0]});
	$packages_installed{$_[0]} = 1;

	foreach my $p (split ',', $package_requires{$_[0]})
	{
		if (!defined $packages_installed{$p})
		{
			#print "Installing dependency $p\n";
			install_package($p);
		}
	}

	my $url = $base_url.$package_full_name{@_[0]};

	#print "installing $url\n";
	$total_bytes += $package_size{$_[0]};
	#print "total_bytes = $total_bytes\n";
	#return;

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
		print `tar xzvf data.tar.gz -C $ARGV[1]`;
	}
	
	chdir $oldcwd;
}

sub process_package
{
	open my $fh, "curl @_[0] |" or die "Cannot run wget $_[0] : $!";
	my $package_name = "";
	while (my $line = <$fh> )
	{
		if ($line =~ /Package: (.+)/)
		{
			$package_name = $1
		}
		elsif ($line =~ /Depends: (.+)/)
		{
			# split by commas
			# remove version requirement
			# push onto package_requires{package_name}
			my @depends = split ',',$1;
			#print "$package_name : @depends\n";
			foreach my $d (@depends)
			{
				my $dep = (split ' ', $d)[0];
				$package_requires{$package_name} .= "," if (defined $package_requires{$package_name});
				$package_requires{$package_name} .= $dep;
			}
			#print "Package $package_name requires $package_requires{$package_name}\n";
		}
		elsif ($line =~ /Filename: (.+)/)
		{
			$package_full_name{$package_name} = $1;
			#print "Name = $package_name, full_name = $1\n";
		}
		elsif ($line =~ /Size: (\d+)/)
		{
			$package_size{$package_name} = int($1);
		}
	}
}

process_package($base_url . "Packages");
# These are all needed to build ROS from source for the RoboRIO.  They
# might not all be needed to build programs against the ROS code ...
# some are static libs which get linked into ROS programs.  Still,
# be safe rather than sorry since disk space isn't at a huge premium 
# on our development systems.

install_package("lz4-staticdev");
install_package("curl-staticdev");
install_package("boost-dev");
install_package("boost-staticdev");

install_package("libeigen-dev");
install_package("libeigen");
install_package("libpython2.7-1.0");
install_package("python-core");
install_package("python-dev");

install_package("bzip2-staticdev");
install_package("libbz2-1");

install_package("cmake-dev");

install_package("libxml2-staticdev");

install_package("libgnutls-bin");
install_package("libgnutls-dev");
install_package("libgnutlsxx28");

install_package("libunistring-staticdev");

install_package("nettle-staticdev");

install_package("libgmp-staticdev");

install_package("libz-staticdev");

install_package("libprotobuf-staticdev");
install_package("protobuf-staticdev");

#install_package("gflags-dbg");
install_package("gflags-dev");
install_package("gflags");

install_package("libglog-staticdev");

install_package("libunwind-staticdev");

# OpenCV - needed for WPILib
#install_package("libopencv-calib3d-dev");
#install_package("libopencv-core-dev");
#install_package("libopencv-features2d-dev");
#install_package("libopencv-flann-dev");
#install_package("libopencv-highgui-dev");
#install_package("libopencv-imgcodecs-dev");
#install_package("libopencv-imgproc-dev");
#install_package("libopencv-ml-dev");
#install_package("libopencv-objdetect-dev");
#install_package("libopencv-photo-dev");
#install_package("libopencv-shape-dev");
#install_package("libopencv-stitching-dev");
#install_package("libopencv-superres-dev");
#install_package("libopencv-video-dev");
#install_package("libopencv-videoio-dev");
#install_package("libopencv-videostab-dev");
#install_package("opencv-apps");
#install_package("opencv-dbg");
install_package("opencv-dev");
#install_package("opencv-samples");
install_package("opencv");

#install_package("tbb");

# rsync packages needed for deployment script
install_package("rsync");

install_package("libidn11");

install_package("libusb-1.0-staticdev");

install_package("openssl-conf");
#install_package("openssl-dbg");
install_package("openssl-staticdev");
install_package("openssl-doc");
install_package("openssl-engines");
install_package("openssl-misc");
#install_package("openssl-staticdev");
#install_package("openssl");

# Various gpg stuff from rosbag
install_package("gpgme-dev");
install_package("libgpg-error-dev");
install_package("libassuan-dev");

install_package("gperftools-dev");

install_package("python-psutil");
