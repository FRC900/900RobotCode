#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "pixy_get_lines/PixyLine.h"
#include "screen_to_world/World.h"

using namespace std;

int hfov;
int vfov;
int frameWidth;
int frameHeight;
int camera_height;

ros::Publisher pub;

struct point
{
     int x;
     int y;
};

point getWorldCoords(int x, int y){
	point worldPoint;
	//dx is the distance between the viewer vertex and the image plane using vertical FOV
	int dx = (frameHeight)/(2 * tan(vfov/2));
	int vtheta = atan((frameHeight - y)/dx);
	int worldx = camera_height/(tan(vtheta));
	//dy is the distance between the viewer vertex and the image plane using horizontal FOV
	int dy = (frameWidth)/(2 * tan(hfov/2));
	int htheta = atan((frameWidth - x)/dy);	
	int worldy = worldx * tan(htheta);
	worldPoint.x = worldx;
	worldPoint.y = worldy;
	return worldPoint;
}

void callback(const pixy_get_lines::PixyLine line)
{	
	screen_to_world::Vector vector;
	int size = line.vectors.size();
    ROS_INFO("array size: %d", size);	
	if (size != 0){
		
		int x0 = line.vectors[0].x0;
		int y0 = line.vectors[0].y0;
		int x1 = line.vectors[0].x1;
		int y1 = line.vectors[0].y1;
		
		point start = getWorldCoords(x0,y0);
		point end = getWorldCoords(x1,y1);

		vector.x0 = start.x;
		vector.y0 = start.y;
		vector.x1 = end.x;
		vector.y1 = end.y;
		
		pub.publish(vector);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "screen_to_world");
  ros::NodeHandle n;

  int sub_rate;
  int pub_rate;

  n.getParam("pub_rate", pub_rate);
  n.getParam("sub_rate", sub_rate);
  n.getParam("hfov", hfov);
  n.getParam("vfov", vfov);
  n.getParam("frameWidth", frameWidth);
  n.getParam("frameHeight", frameHeight);
  n.getParam("camera_height", camera_height);

  pub = n.advertise<screen_to_world::Vector>("world", pub_rate);

  ros::Subscriber sub = n.subscribe("pixy_line", sub_rate, callback);

  ros::spin();

  return 0;
}
