#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>
#include "cube_detection/CubeDetection.h"

#include <sstream>

#include <cv_bridge/cv_bridge.h>

int erosion_size = 1;
const int sliderMax = 255;

static ros::Publisher pub;

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

int hLo = 18;
int sLo = 65;
int vLo = 180;
int hUp = 47;

int maxTrans = 15900;
int minTrans = 6500;

int pixelError = .06;

//orig: 193695.3745 * .2226^x

//min: 193695.3745 * .2226^x - 9000

//max: 193695.3745 * .2226^x + 9000

static bool down_sample = false;
//This funtion along with the commented out slider code is useful when getting new HSV values for the threshold
//To get the trackbars active, comment out the lines marked "mark1", uncomment the lines marked "mark2",
//multiline comment from "markS" to "markE"
void on_trackbar(int, void*){}

void callback(const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg)
{
	cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1); 
	
	// Avoid copies by using pointers to RGB and depth info
	// These pointers are either to the original data or to
	// the downsampled data, depending on the down_sample flag
	const Mat *framePtr = &cvFrame->image;
	const Mat *depthPtr = &cvDepth->image;

	// To hold downsampled images, if necessary
	Mat frame;
	Mat depth;

 	Mat gray;
	Mat thresh;

	// Downsample for speed purposes
	if (down_sample)
	{
		pyrDown(*framePtr, frame);
		pyrDown(*depthPtr, depth);

		framePtr = &frame;
		depthPtr = &depth;
	}

	Mat hsv;
	Mat threshold;
	Mat contour;

	cvtColor(*framePtr,hsv,COLOR_BGR2HSV);
	cvtColor(*framePtr, gray, COLOR_BGR2GRAY);
	inRange(hsv, Scalar(hLo,sLo,vLo),Scalar(hUp,255,255),threshold); //mark2

	erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(7,7)));
	dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(7,7)));

	dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
	erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

	vector<vector<Point> > contours;
	vector<Vec4i> rank;

	findContours(threshold, contours, rank, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	vector<vector<Point>> contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<float> contourDepth;
	vector<int> approxPoly;
	
	cube_detection::CubeDetection cd_msg;

	


	for(size_t i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]),contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		
	}

	for(size_t i = 0; i < contours.size(); i++)
	{
		const int x = boundRect[i].x + (boundRect[i].width/2);
		const int y = boundRect[i].y + (boundRect[i].height/2);
		float depth_value = depthPtr->at<float>(y,x);
		contourDepth.push_back(depth_value);
	}
	
	
	Mat drawing = Mat::zeros(threshold.size(),CV_8UC3);
	for(size_t i = 0; i< contours.size(); i++)
	{
		double minArea = sqrt(193695.3745 * (pow(0.2226,contourDepth[i])) - minTrans); 
		//double maxArea = sqrt(193695.3745 * (pow(0.2226,contourDepth[i])) + maxTrans); 
		double areaContour = boundRect[i].height * boundRect[i].width;
		Scalar rect_color = Scalar(0,0,255);
		Scalar color = Scalar(0,255,0);		

		//filter contours by area based on depth and side ratio
		if (areaContour < minArea) {
			continue;
		} else if (areaContour <= (drawing.rows * drawing.cols * pixelError)) {
			continue;
		} else if (abs((boundRect[i].height/boundRect[i].width)) > 2) {
			continue;
		} else if (abs((boundRect[i].width/boundRect[i].height)) > 2) {
			continue;
		} else if (contours_poly[i].size() < 6 || contours_poly[i].size() > 3) {	
			putText(drawing, to_string(contourDepth[i]), Point(boundRect[i].x, boundRect[i].y 		- 15), FONT_HERSHEY_SIMPLEX, 0.45, (0,0,255), 1);
			drawContours(drawing, contours,i,color,2,8,rank,0,Point());
			rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), rect_color, 2, 8, 0);
			//ROS_INFO_STREAM(contours_poly[i].size());
			/*ROS_INFO_STREAM("x = " << boundRect[i].x + (boundRect[i].width/2));
			ROS_INFO_STREAM("y = " << boundRect[i].y + (boundRect[i].height/2));
			ROS_INFO_STREAM("z = " << contourDepth[i]);*/
			cd_msg.header.seq = frameMsg->header.seq;
			cd_msg.header.stamp = frameMsg->header.stamp;
			cd_msg.header.frame_id = frameMsg->header.frame_id;
			cd_msg.location.x = boundRect[i].x + boundRect[i].width/2;
			cd_msg.location.y = boundRect[i].y + boundRect[i].height/2;
			cd_msg.location.z = contourDepth[i];
			pub.publish(cd_msg);
		}
	}

	imshow("threshold",threshold); 
	imshow("hsv",hsv);
	imshow("drawing",drawing); 
	imshow("depth", *depthPtr);
	waitKey(5);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cube_detect");

	ros::NodeHandle nh("~");
	int sub_rate = 5;
	int pub_rate = 1;

	
	
	message_filters::Subscriber<Image> frame_sub(nh, "/zed_goal/left/image_rect_color", sub_rate);
	message_filters::Subscriber<Image> depth_sub(nh, "/zed_goal/depth/depth_registered", sub_rate);

	typedef sync_policies::ApproximateTime<Image, Image > MySyncPolicy2;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(xxx)
	Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(50), frame_sub, depth_sub);
	sync2.registerCallback(boost::bind(&callback, _1, _2));

	pub = nh.advertise<cube_detection::CubeDetection>("cube_detect_msg", pub_rate);

	namedWindow("drawing",1);
		
	//trackbars for testing hsv and range values
	createTrackbar( "Lower H", "drawing", &hLo, 180);
	createTrackbar( "Lower S", "drawing", &sLo, 255);  
	createTrackbar( "Lower V", "drawing", &vLo, 255);
	createTrackbar( "Higher H", "drawing", &hUp, 180);
	createTrackbar( "minTrans", "drawing", &minTrans, 30000);
	createTrackbar( "maxTrans", "drawing", &maxTrans, 30000);	
	//createTrackbar( "minArea", "drawing", &minArea, 5000);
	//createTrackbar( "maxArea", "drawing", &maxArea, 1000000);

	ros::spin();

	return 0;
}
