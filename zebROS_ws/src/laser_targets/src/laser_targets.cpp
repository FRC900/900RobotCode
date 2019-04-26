#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/geometry.h>

#include <opencv2/opencv.hpp>

laser_geometry::LaserProjection projector_;

const size_t minLinePoints = 5;

cv::VideoWriter vw;

static bool comparePoints(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
        if (p1.x < p2.x)
		{
			return true;
		}
		else if ((p1.x == p2.x) && (p1.y < p2.y))
		{
			return true;
		}
		else
		{
			return false;
		}
}

void laserScanCallback(sensor_msgs::LaserScanConstPtr msg)
{
	sensor_msgs::PointCloud2 ros_cloud;
	projector_.projectLaser(*msg, ros_cloud);
	//ros::message_operations::Printer< ::sensor_msgs::LaserScan_<std::allocator<void>> >::stream(std::cout, "", *msg);
	//ros::message_operations::Printer< ::sensor_msgs::PointCloud2_<std::allocator<void>> >::stream(std::cout, "", ros_cloud);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(ros_cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	pcl::ModelCoefficients coefficients;
	pcl::ModelCoefficients prevCoefficients;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(.005);
#if 0
	// Limit search radius
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>);
	search->setInputCloud(temp_cloud);
	seg.setSamplesMaxDist(0.5, search);
#endif

	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
	const size_t origSize = temp_cloud->points.size();

	// Draw point cloud using opencv
	constexpr double scale = 100;
	constexpr double shift = 450;
	constexpr int radius = 3.5;
	cv::Mat m(shift * 2, shift * 2, CV_8UC3, cv::Scalar(0));
	if (!vw.isOpened())
	   vw.open("laser.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30., cv::Size(shift*2, shift*2), true);
	for (const auto p : *temp_cloud)
	{
		cv::circle(m, // opencv mat to write to
				cv::Point2f(p.x * scale + shift, p.y * scale + shift), // center of circle - scale it so it is visible
				radius, // make it large enough to be seen
				cv::Scalar(128, 0, 128), // yellow?
				-1); // fill the circle in
	}

	while (temp_cloud->points.size() > (0.01 * origSize))
	{
		seg.setInputCloud(temp_cloud);
		seg.segment(*inliers, coefficients);

		extract.setInputCloud(temp_cloud);
		extract.setIndices(inliers);

		if(inliers->indices.size() == 0)
		{
			//ROS_INFO("Could not estimate a planar model for the remining dataset.");
			break;
		}
		if(inliers->indices.size() < minLinePoints)
			break;

#if 0
		ROS_INFO_STREAM("Model coefficients: "
				<< coefficients.values[0] << " " // point on line x
				<< coefficients.values[1] << " " // point on line y
				<< coefficients.values[2] << " " // point on line z
				<< coefficients.values[3] << " " // direction of line x
				<< coefficients.values[4] << " " // direction of line y
				<< coefficients.values[5]);      // direction of line z
		ROS_INFO_STREAM("Model slope = " << coefficients.values[4] / coefficients.values[3]);
#endif

		//ROS_INFO_STREAM("Model inliers: " << inliers->indices.size());
		// Grab inlier points - these are the ones that fit the given
		// line found by RANSAC above
		extract.setNegative(false);
		extract.filter(*in);

		std::sort(in->begin(), in->end(), comparePoints);
#if 0
		ROS_INFO_STREAM("Inliers:");
		for (const auto &p : *in)
			ROS_INFO_STREAM("    " << p.x << "," << p.y);
		if (in->size() >= 3)
		{
			std::sort(in->begin(), in->end(), comparePoints);
			const pcl::PointXYZ p1 = (*in)[0];
			const pcl::PointXYZ p2 = in->back();
			cv::line(m, cv::Point2f(p1.x * scale + shift, p1.y * scale + shift), cv::Point2f(p2.x * scale + shift, p2.y * scale + shift), cv::Scalar(0,255,0), 2);
		}
#endif
		in_filtered->clear();
		//const size_t enough_points = std::max(in->size() * .4, 1.);
		//ROS_INFO_STREAM("enough_points = " << enough_points);
		// The line segmentation algorithm can get aggressive at grouping
		// points into lines even if they're separated by a large distance.
		// Some of this can be controlled by the seg.SetDistanceThreshold() call.
		// But not always - there are cases where it ends up combining points
		// which are too far apart into a single line.
		// Here, if consecutive points are too far away, split them
		// into separate line segments
		constexpr double maxPointDist = 0.15;
		for (size_t i = 0; i < (in->size() - 1); )
		{
			const pcl::PointXYZ p1 = (*in)[i];
			const pcl::PointXYZ p2 = (*in)[i + 1];
			const double dp1_p2 = hypot(p2.x - p1.x, p2.y - p1.y);
			if (dp1_p2 > maxPointDist)
			{
				//ROS_INFO_STREAM("Distance too large (" << dp1_p2 << ") : " << p1 << " : " << p2);
				if (i > (in->size() / 2))
				{
					//ROS_INFO_STREAM("Moving " << (*in)[i+1] << " to filtered from back");
					in_filtered->push_back((*in)[i+1]);
					in->erase(in->begin() + i+1);
				}
				else
				{
					// Check distance between previous and next point. If they
					// are within range, the middle point is just a weird outlier
					// If so, remove only that one
					// TODO - check that this works as expected
					if (i > 0)
					{
						const pcl::PointXYZ p0 = (*in)[i-1];
						const double dp0_p2 = hypot(p2.x - p0.x, p2.y - p0.y);
						if (dp0_p2 <= maxPointDist)
						{
							//ROS_INFO_STREAM("Moving " << (*in)[i] << " to filtered from middle");
							in_filtered->push_back((*in)[i]);
							in->erase(in->begin() + i);
							continue;
						}
					}
					// Otherwise, all the prior points are too far from
					// the current one. Thus, move all of them out of the
					// current set of linear points and put them back
					// into the unclassified temp_cloud list
					for (size_t j = 0; j <= i; j++)
					{
						//ROS_INFO_STREAM("Moving " << (*in)[0] << " to filtered");

						in_filtered->push_back((*in)[0]);
						in->erase(in->begin());
					}
					i = 0;
				}
			}
			else
				i += 1;
		}

		if (in->size() >= minLinePoints)
		{
			std::sort(in->begin(), in->end(), comparePoints);
			const pcl::PointXYZ p1 = (*in)[0];
			const pcl::PointXYZ p2 = in->back();
			const double slope = (p2.y - p1.y) / (p2.x - p1.x);
			//ROS_INFO_STREAM("Segment from " << p1 << " to " << p2 << " slope = " << slope);
			cv::line(m, cv::Point2f(p1.x * scale + shift, p1.y * scale + shift), cv::Point2f(p2.x * scale + shift, p2.y * scale + shift), cv::Scalar(0,255,255));
		}

		// Filter the used points out of temp_cloud, leaving a set of
		// points which didn't fit on that particular line.
		extract.setNegative(true);
		extract.filter(*out);
		//ROS_INFO_STREAM("temp_cloud.size() = " << temp_cloud->size() << " out.size() = " << out->size() << " in.size() = " << in->size());

		temp_cloud.swap(out);
		temp_cloud->insert(temp_cloud->end(), in_filtered->begin(), in_filtered->end());
#if 0
		ROS_INFO("Inliers, post-filtering:");
		for (const auto &p : *in)
			ROS_INFO_STREAM("    " << p.x << "," << p.y);
		ROS_INFO("Filtered:");
		for (const auto &p : *in_filtered)
			ROS_INFO_STREAM("    " << p.x << "," << p.y);
#endif
	}

	cv::imshow("PointCloud", m);
	vw.write(m);
	cv::waitKey(5);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_detect");
	ros::NodeHandle nh("~");

	auto sub = nh.subscribe("/rplidar/scan", 10, laserScanCallback);

	ros::spin();

	vw.release();

	return 0;
}

