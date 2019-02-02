#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "pixy_get_lines/PixyLine.h"
#include "screen_to_world/World.h"

#define _USE_MATH_DEFINES

using namespace std;

int hfov;
int vfov;
int frameWidth;
int frameHeight;
double camera_height;
double angle;

ros::Publisher pub;

struct point
{
     double x, y;
};

double toRad(int deg){

	double rad = deg * (M_PI/180);
	return rad;
}

struct point getWorldCoords(int x, int y){
	
	struct point worldPoint;

	double centerx = frameWidth/2;
	double centery = frameHeight/2;
	double h = x - centerx;
	double k = y - centery;

	double d = (frameHeight)/(2 * tan(toRad(vfov/2)));
	double vtheta = atan(((frameHeight/2) - (frameHeight - y))/d);
 	double htheta = vtheta + toRad(angle);
	double worldx = camera_height/tan(htheta);

	double D = sqrt(pow(camera_height, 2) + pow(worldx, 2));
	double ytheta = acos((pow(d, 2) + pow(k, 2))/(sqrt(pow(d, 2) + pow(k, 2)) * sqrt(pow(d, 2) + pow(h, 2) + pow(k, 2))));

	double worldy = D * tan(ytheta);
	
	worldPoint.x = worldx;
	worldPoint.y = worldy;

	return worldPoint;
}

void callback(const pixy_get_lines::PixyLine line)
{	
	screen_to_world::WorldVector vector;
	int size = line.vectors.size();
	if (size == 1){
		
		int x0 = line.vectors[0].x0;
		int y0 = line.vectors[0].y0;
		int x1 = line.vectors[0].x1;
		int y1 = line.vectors[0].y1;
		
		struct point start = getWorldCoords(x0,y0);
		struct point end = getWorldCoords(x1,y1);

		vector.x0 = start.x;
		vector.y0 = start.y;
		vector.x1 = end.x;
		vector.y1 = end.y;
		vector.slope = (end.y - start.y)/(end.x - start.x);
		
		pub.publish(vector);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "screen_to_world");
  ros::NodeHandle n("~");

  int sub_rate;
  int pub_rate;

  n.getParam("pub_rate", pub_rate);
  n.getParam("sub_rate", sub_rate);
  n.getParam("hfov", hfov);
  n.getParam("vfov", vfov);
  n.getParam("frameWidth", frameWidth);
  n.getParam("frameHeight", frameHeight);
  n.getParam("camera_height", camera_height);
  n.getParam("angle", angle);

  pub = n.advertise<screen_to_world::WorldVector>("world", pub_rate);

  ros::Subscriber sub = n.subscribe("/pixy_line", sub_rate, callback);

  ros::spin();

  return 0;
}
