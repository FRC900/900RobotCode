#include "ros/ros.h"

#include "libpixyusb2.h"
#include "pixy_get_lines/PixyLine.h"

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

	ros::Rate r(100);
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
		r.sleep();
	}

	return EXIT_SUCCESS;
}
