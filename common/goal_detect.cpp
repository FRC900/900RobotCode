#include <iostream>
#include <opencv2/opencv.hpp>

#include "zedcamerain.hpp"
#include "zedsvoin.hpp"
#include "zmsin.hpp"
#include "GoalDetector.hpp"
#include "frameticker.hpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	MediaIn *cap = NULL;
	if (argc == 2)
		cap = new ZMSIn(argv[1]);
	else
		cap = new ZedCameraIn(true);

	if (cap == NULL)
	{
		cerr << "Error creating input" << endl;
		return -1;
	}

	GoalDetector gd(Point2f(cap->getCameraParams().fov.x,
				            cap->getCameraParams().fov.y),
			        Size(cap->width(),cap->height()), true);
	Mat image;
	Mat depth;
	Rect bound;
	FrameTicker frameTicker;
	bool paused = false;
	bool frame  = false;
	while (cap->getFrame(image, depth, paused))
	{
		if (frame)
		{
			paused = true;
			frame = false;
		}

		frameTicker.mark();

		gd.findBoilers(image, depth);
		gd.drawOnFrame(image, gd.getContours(image));

		stringstream ss;
		ss << fixed << setprecision(2) << cap->FPS() << " C:" << frameTicker.getFPS() << " GD FPS" << cap->frameNumber() << " FrameNum:";
		putText(image, ss.str(), Point(image.cols - 15 * ss.str().length(), 50), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255));
		//rectangle(image, gd.goal_rect(), Scalar(255,0,0), 2);
		imshow ("Image", image);

		switch((uchar)waitKey(5))
		{
			case ' ':
				paused = !paused;
				break;
			case 'f':
				frame = true;
				paused = false;
				break;
			case 27: // ESC
				return 0;
		}
	}
	return 0;
}
