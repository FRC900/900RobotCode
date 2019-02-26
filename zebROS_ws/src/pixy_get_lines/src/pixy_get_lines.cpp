#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "libpixyusb2.h"
#include "pixy_get_lines/PixyLine.h"

void demosaic(size_t width, size_t height, const uint8_t *bayerImage, cv::Mat &mat)
{
	mat.create(height, width, CV_8UC3);
	size_t x, y, xx, yy;
	uint32_t r, g, b;
	const uint8_t *pixel0, *pixel;

	for (y=0; y<height; y++)
	{
		yy = y;
		if (yy==0)
			yy++;
		else if (yy==height-1)
			yy--;
		pixel0 = (const uint8_t *)bayerImage + yy*width;
		for (x=0; x<width; x++)
		{
			uchar *ptr = mat.ptr<uchar>(y);
			xx = x;
			if (xx==0)
				xx++;
			else if (xx==width-1)
				xx--;
			pixel = pixel0 + xx;
			if (yy&1)
			{
				if (xx&1)
				{
					r = *pixel;
					g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
					b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
				}
				else
				{
					r = (*(pixel-1)+*(pixel+1))>>1;
					g = *pixel;
					b = (*(pixel-width)+*(pixel+width))>>1;
				}
			}
			else
			{
				if (xx&1)
				{
					r = (*(pixel-width)+*(pixel+width))>>1;
					g = *pixel;
					b = (*(pixel-1)+*(pixel+1))>>1;
				}
				else
				{
					r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
					g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
					b = *pixel;
				}
			}
			*ptr++ = b;
			*ptr++ = g;
			*ptr++ = r;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pixy_get_lines");
	ros::NodeHandle n;

	Pixy2 pixy;
	int rc = pixy.init();
	if (rc < 0)
	{
		ROS_ERROR_STREAM("pixy.init() returned " << rc);
		return rc;
	}
	ROS_INFO("pixy.init() success\n");

	pixy.line.setMode(LINE_MODE_WHITE_LINE);

	rc = pixy.getVersion();
	if (rc < 0)
	{
		ROS_ERROR_STREAM("pixy.getVersion() returned " << rc);
		return rc;
	}
	ROS_INFO_STREAM("Pixy version HW:"
			<< pixy.version->hardware
			<< "FW:" << pixy.version->firmwareMajor
			<< "." << pixy.version->firmwareMinor
			<< " Build:" << pixy.version->firmwareBuild
			<< " FW Type:" << pixy.version->firmwareType);

	// Set Pixy2 to line feature identification program //
	pixy.changeProg("line");

	ros::Publisher pub = n.advertise<pixy_get_lines::PixyLine>("pixy_line", 1);
	image_transport::ImageTransport it(n);
	image_transport::Publisher img_pub = it.advertise("pixy_image", 1);
	ros::Rate r(100);
	cv::Mat mat;
	while(ros::ok())
	{
		// Query Pixy for line features //
		if (pixy.line.getMainFeatures() >= 0)
		{
			pixy_get_lines::PixyLine msg;

			msg.header.stamp = ros::Time::now();

			for (uint8_t i= 0; i < pixy.line.numVectors; ++i)
			{
				pixy_get_lines::Vector vector;

				vector.x0 = pixy.line.vectors[i].m_x0;
				vector.y0 = pixy.line.vectors[i].m_y0;
				vector.x1 = pixy.line.vectors[i].m_x1;
				vector.y1 = pixy.line.vectors[i].m_y1;
				vector.index = pixy.line.vectors[i].m_index;
				vector.flags = pixy.line.vectors[i].m_flags;

				msg.vectors.push_back(vector);
			}

			for (uint8_t i = 0; i < pixy.line.numIntersections; ++i)
			{
				pixy_get_lines::Intersection intersection;

				intersection.x = pixy.line.intersections[i].m_x;
				intersection.y = pixy.line.intersections[i].m_y;
				intersection.n = pixy.line.intersections[i].m_n;
				for (uint8_t n = 0; n < pixy.line.intersections[i].m_n; ++n)
				{
					pixy_get_lines::IntersectionLine intersection_line;
					intersection_line.index =  pixy.line.intersections[i].m_intLines[n].m_index;
					intersection_line.angle =  pixy.line.intersections[i].m_intLines[n].m_angle;
					intersection.int_lines.push_back(intersection_line);
				}
				msg.intersections.push_back(intersection);
			}

			for (uint8_t i = 0; i < pixy.line.numBarcodes; ++i)
			{
				pixy_get_lines::Barcode barcode;
				barcode.x = pixy.line.barcodes[i].m_x;
				barcode.y = pixy.line.barcodes[i].m_y;
				barcode.flags = pixy.line.barcodes[i].m_flags;
				barcode.code = pixy.line.barcodes[i].m_code;
				msg.barcodes.push_back(barcode);
			}
			pub.publish(msg);
		}
		/*
		{
			pixy.m_link.stop();
			uint8_t *bayerFrame;
			pixy.m_link.getRawFrame(&bayerFrame);
			demosaic(PIXY2_RAW_FRAME_WIDTH, PIXY2_RAW_FRAME_HEIGHT, bayerFrame, mat);
			pixy.m_link.resume();
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat).toImageMsg();
			img_pub.publish(msg);
		}
		*/
		r.sleep();
	}

	return EXIT_SUCCESS;
}
