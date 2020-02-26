#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include "GoalDetector.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace cv;

#define VERBOSE
#define VERBOSE_DEEP

int camera_angle_common = 25;

void angleCallback(int value, void *data)
{
	std::cout << "running AngleCallback with value " << value << endl;
	//double angle = static_cast<double>(value)/-10;
	//gd->setCameraAngle(angle);
}

GoalDetector::GoalDetector(const cv::Point2f &fov_size, const cv::Size &frame_size, bool gui) :
	_fov_size(fov_size),
	_frame_size(frame_size),
	_isValid(false),
	_min_valid_confidence(0.30),
	_otsu_threshold(5),
	_blue_scale(90),
	_red_scale(80),
	_camera_angle(0), // in tenths of a degree
	_target_num(POWER_PORT_2020)
{
	if (gui)
	{
		cv::namedWindow("Goal Detect Adjustments", CV_WINDOW_NORMAL);
		createTrackbar("Blue Scale","Goal Detect Adjustments", &_blue_scale, 100);
		createTrackbar("Red Scale","Goal Detect Adjustments", &_red_scale, 100);
		createTrackbar("Otsu Threshold","Goal Detect Adjustments", &_otsu_threshold, 255);
		createTrackbar("Camera Angle","Goal Detect Adjustments", &camera_angle_common, 900, angleCallback, this);
	}
}

// Compute a confidence score for an actual measurement given
// the expected value and stddev of that measurement
// Values around 0.5 are good. Values away from that are progressively
// worse.  Wrap stuff above 0.5 around 0.5 so the range
// of values go from 0 (bad) to 0.5 (good).
float GoalDetector::createConfidence(float expectedVal, float expectedStddev, float actualVal)
{
	pair<float,float> expectedNormal(expectedVal, expectedStddev);
	float confidence = zv_utils::normalCFD(expectedNormal, actualVal);
	return confidence > 0.5 ? 1 - confidence : confidence;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                      Point2f &r)
{
    const Point2f x = o2 - o1;
    const Point2f d1 = p1 - o1;
    const Point2f d2 = p2 - o2;

    const float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    const double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

void GoalDetector::findTargets(const cv::Mat& image, const cv::Mat& depth) {
	clear();
	const vector<vector<Point>> goal_contours = getContours(image);
	if (goal_contours.size() == 0)
		return;
	const vector<DepthInfo> goal_depths = getDepths(depth, goal_contours, _target_num, ObjectType(_target_num).real_height());

	//compute confidences for power port tapes
	const vector<GoalInfo> power_port_info = getInfo(goal_contours, goal_depths, _target_num);
	if(power_port_info.size() == 0)
		return;
#ifdef VERBOSE
	cout << power_port_info.size() << " goals found" << endl;
#endif

	//loop through every power port goal found
	for(size_t i = 0; i < power_port_info.size(); i++) {
#ifdef VERBOSE_DEEP
			cout << "i:" << i << endl;
			cout << power_port_info[i].contour_index << " cidx" << endl;
#endif

			// If this is the first valid pair
			// or if this pair has a higher combined
			// confidence than the previously saved
			// pair, keep it as the best result
			if (power_port_info[i].confidence > _min_valid_confidence)
			{
				GoalFound goal_found;
				goal_found.confidence        = power_port_info[i].confidence;
				goal_found.contour_index     = power_port_info[i].contour_index;
				goal_found.rect		           = power_port_info[i].rect;
				goal_found.rotated_rect      = power_port_info[i].rtRect;
				goal_found.distance             = power_port_info[i].distance;
				goal_found.id                = getObjectId(_target_num);

				//These are the saved values for the best goal before moving on to
				//try and find another one.
				bool repeated = false;
				if(_return_found.size() > 0)
				{
					size_t gf_ci = goal_found.contour_index;
					for(size_t k = 0; k < _return_found.size(); k++)
					{
						size_t rf_ci = _return_found[k].contour_index;
						// compare contour indexes of goal_found vs _return_found[k].  If neither
						// match, this can't be a repeated goal so add it to return_found and continue.
						if (gf_ci != rf_ci)
						{
							continue;
						}
						repeated = true;

						if (goal_found.confidence > _return_found[k].confidence)
						{
							_return_found[k] = goal_found;
							break;
						}

						// TODO : then, if confidence is higher for goal_found compared to return_found[k],
						// replace return_found[k] with goal info.  This might not be needed
						// TODO : otherwise, discard goal_found since the previously found goal in return_found
						// was closer to the ideal goal
#if 0
						if(abs(power_port_info[i].pos.x - _return_found[k].pos.x) < min_dist_bwn_goals)
						{
							break;
						}
						for(size_t l = 0; l < _return_found.size(); l++)
						{
							if(abs(power_port_info[i].pos.x - _return_found[l].pos.x) < min_dist_bwn_goals)
								repeated = true;
						}
#endif
					}
				}
				if(repeated == false)
				{
					_return_found.push_back(goal_found);
				}
				_isValid = true;

#ifdef VERBOSE_DEEP
				cout << "Number of goals: " << _return_found.size() << endl;
				for(size_t n = 0; n < _return_found.size(); n++)
				{
					cout << "Goal " << n << " " << _return_found[n].contour_index <<
						" distance: " << _return_found[n].distance << " confidence: "<< _return_found[n].confidence << endl;
				}
#endif
			}
			else
			{
#ifdef VERBOSE_DEEP
				cout << "goal " << i << " confidence too low -> " << power_port_info[i].confidence << endl;
#endif
			}
	}
}

// Reset previous detection vars
void GoalDetector::clear()
{
	_isValid = false;
	_return_found.clear();
}

const vector< vector < Point > > GoalDetector::getContours(const Mat& image) {
	// Look for parts the the image which are within the
	// expected bright green color range
	static Mat threshold_image;
	vector < vector < Point > > return_contours;
	if (!generateThresholdAddSubtract(image, threshold_image))
	{
		_isValid = false;
		//_pastRects.push_back(SmartRect(Rect()));
		return return_contours;
	}

	// find contours in the thresholded image - these will be blobs
	// of green to check later on to see how well they match the
	// expected shape of the goal
	// Note : findContours modifies the input mat
	vector<Vec4i> hierarchy;
	findContours(threshold_image.clone(), return_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	return return_contours;
}

const vector<DepthInfo> GoalDetector::getDepths(const Mat &depth, const vector< vector< Point > > &contours, const ObjectNum &objtype, float expected_height) {
	// Use to mask the contour off from the rest of the
	// image - used when grabbing depth data for the contour
	static Mat contour_mask(_frame_size, CV_8UC1, Scalar(0));
	vector<DepthInfo> return_vec;
	DepthInfo depthInfo;
	for(size_t i = 0; i < contours.size(); i++) {
		// get the minimum and maximum depth values in the contour,
		const Rect br(boundingRect(contours[i]));
		//const Moments mu = moments(contours[i], false);
		//const Point com = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
		//Point center(rect.tl().x+rect.size().width/2, rect.tl().y+rect.size().height/2);

		//create a mask which is the same shape as the contour
		// TODO - don't clear these?
		contour_mask.setTo(Scalar(0));
		drawContours(contour_mask, contours, i, Scalar(255), CV_FILLED);
		// copy them into individual floats
		//pair<float, float> minMax = zv_utils::minOfDepthMat(depth, contour_mask, br, 10);
		const float average_depth = zv_utils::avgOfDepthMat(depth, contour_mask, br);
		float depth_z_min = average_depth;
		float depth_z_max = average_depth;

#ifdef VERBOSE
		cout << "Depth " << i << ": " << depth_z_min << " " << depth_z_max<< endl;
#endif

		// If no depth data, calculate it using FOV and height of
		// the target. This isn't perfect but better than nothing
		if ((depth_z_min <= 0.) || (depth_z_max <= 0.)) {
			depthInfo.error = true;
			//ObjectType ot(objtype);
			depth_z_min = depth_z_max = distanceUsingFOV(objtype, br);
		}
		else
			depthInfo.error = false;
		depthInfo.depth = depth_z_max;

		return_vec.push_back(depthInfo);
	}
	return return_vec;
}

const vector<GoalInfo> GoalDetector::getInfo(const vector<vector<Point>> &contours, const vector<DepthInfo> &depth_maxs, ObjectNum objtype) {
	const ObjectType goal_shape(objtype);
#ifdef VERBOSE
	std::cout << __PRETTY_FUNCTION__ << " : for goal_shape.name() = " << goal_shape.name() << std::endl;
#endif
	vector<GoalInfo> return_info;
	// Create some target stats based on our idealized goal model
	//center of mass as a percentage of the object size from left left
	const Point2f com_percent_expected(goal_shape.com().x / goal_shape.width(),
									   goal_shape.com().y / goal_shape.height());

	// Ratio of contour area to bounding box area
	const float filledPercentageExpected = goal_shape.area() / goal_shape.boundingArea();

	// Aspect ratio of the goal
	const float expectedRatio = (float)std::min(goal_shape.height(), goal_shape.width()) / std::max(goal_shape.height(), goal_shape.width());

	for (size_t i = 0; i < contours.size(); i++)
	{
		// ObjectType computes a ton of useful properties so create
		// one for what we're looking at
		const Rect br(boundingRect(contours[i]));

		// Remove objects which are obviously too small
		if (br.area() <= (_frame_size.width * _frame_size.height * .0004))
		{
#ifdef VERBOSE
			cout << "Contour " << i << " area out of range " << br.area() << " vs " << _frame_size.width * _frame_size.height * .0004 << endl;
#endif
			continue;
		}

		//width to height ratio
		// Use rotated rect to get a more accurate guess at the real
		// height and width of the contour
		//const RotatedRect rr(minAreaRect(contours[i]));
		const float actualRatio = (float)std::min(br.height, br.width) / std::max(br.height, br.width);
		if ((actualRatio < .30) || (actualRatio > 1.0))
		{
#ifdef VERBOSE
			cout << "Contour " << i << " height/width ratio fail" << br.size() << " " << actualRatio << endl;
#endif
			continue;
		}

		//create a trackedobject to get various statistics
		//including area and x,y,z position of the goal
		ObjectType goal_actual(contours[i], "Actual Goal", 0);
		TrackedObject goal_tracked_obj(0, goal_shape, br, depth_maxs[i].depth, _fov_size, _frame_size, -((float)_camera_angle/10.) * M_PI / 180.0);

		// Gets the bounding box area observed divided by the
		// bounding box area calculated given goal size and distance
		// For an object the size of a goal we'd expect this to be
		// close to 1.0 with some variance due to perspective
		const float exp_area = goal_tracked_obj.getScreenPosition(_fov_size, _frame_size).area();
		const float actualScreenArea = (float)br.area() / exp_area;

		//percentage of the object filled in
		const float filledPercentageActual = goal_actual.area() / goal_actual.boundingArea();

		//center of mass as a percentage of the object size from left and top
		Point2f com_percent_actual((goal_actual.com().x - br.tl().x) / goal_actual.width(),
								   (goal_actual.com().y - br.tl().y) / goal_actual.height());

		//parameters for the normal distributions
		//values for standard deviation were determined by
		//taking the standard deviation of a bunch of values from the goal
		//confidence is near 0.5 when value is near the mean
		//confidence is small or large when value is not near mean
		const float confidence_height      = createConfidence(goal_shape.real_height(), 0.2, goal_tracked_obj.getPosition().z - ( goal_shape.height() / 2.0 ));
		const float confidence_com_x       = createConfidence(com_percent_expected.x, 0.13,  com_percent_actual.x);
		const float confidence_com_y       = createConfidence(com_percent_expected.y, 0.13,  com_percent_actual.y);
		const float confidence_filled_area = createConfidence(filledPercentageExpected, 0.33, filledPercentageActual);
		const float confidence_ratio       = createConfidence(expectedRatio, 1.5,  actualRatio);
		const float confidence_screen_area = createConfidence(1.0, 0.50, actualScreenArea);

		// higher confidence is better
		const float confidence = (/*confidence_height*/ + confidence_com_x + confidence_com_y + confidence_filled_area + confidence_ratio + confidence_screen_area) / 5;

#ifdef VERBOSE
		cout << "-------------------------------------------" << endl;
		cout << "Contour " << i << endl;
		cout << "confidence_height: " << confidence_height << endl;
		cout << "confidence_com_x: " << confidence_com_x << endl;
		cout << "confidence_com_y: " << confidence_com_y << endl;
		cout << "confidence_filled_area: " << confidence_filled_area << endl;
		cout << "confidence_ratio: " << confidence_ratio << endl;
		cout << "confidence_screen_area: " << confidence_screen_area << endl;
		cout << "confidence: " << confidence << endl;
		cout << "Height exp/act: " << goal_shape.real_height() << "/" <<  goal_tracked_obj.getPosition().z - goal_shape.height() / 2.0 << endl;
		cout << "Depth max: " << depth_maxs[i].depth << " " << depth_maxs[i].error << endl;
		cout << "Screen Area exp/act: " << (int)exp_area << "/" << br.area() << endl;
		cout << "Aspect ratio exp/act : " << expectedRatio << "/" << actualRatio << endl;
		cout << "br: " << br << endl;
		cout << "com: " << goal_actual.com() << endl;
		cout << "com_expected / actual: " << com_percent_expected << " " << com_percent_actual << endl;
		//cout << "Angle: " << minAreaRect(contours[i]).angle << endl;
		//cout << "lineStart: " << start_line << endl;
		//cout << "lineEnd: " << end_line << endl;
		cout << "-------------------------------------------" << endl;
#endif

		GoalInfo goal_info;

		// This goal passes the threshold required for us to consider it a goal
		// Add it to the list of best goals
		goal_info.confidence    = confidence;
		goal_info.distance      = depth_maxs[i].depth * cosf((_camera_angle/10.0) * (M_PI/180.0));
		goal_info.rect		    = br;
		goal_info.contour_index = i;
		goal_info.depth_error   = depth_maxs[i].error;
		goal_info.com           = goal_actual.com();
		goal_info.br            = br;
		return_info.push_back(goal_info);

	}
	return return_info;
}


// We're looking for pixels which are mostly green
// with a little bit of blue - that should match
// the LED reflected color.
// Do this by splitting channels and combining
// them into one grayscale channel.
// Start with the green value.  Subtract the red
// channel - this will penalize pixels which have red
// in them, which is good since anything with red
// is an area we should be ignoring. Do the same with
// blue, except multiply the pixel values by a weight
// < 1. Using this weight will let blue-green pixels
// show up in the output grayscale
bool GoalDetector::generateThresholdAddSubtract(const Mat& imageIn, Mat& imageOut)
{
    static vector<Mat> splitImage;
    static Mat         bluePlusRed;

    split(imageIn, splitImage);
	addWeighted(splitImage[0], _blue_scale / 100.0,
			    splitImage[2], _red_scale / 100.0, 0.0,
				bluePlusRed);
	subtract(splitImage[1], bluePlusRed, imageOut);

    static const Mat erodeElement(getStructuringElement(MORPH_RECT, Size(1, 1)));
    static const Mat dilateElement(getStructuringElement(MORPH_RECT, Size(3, 3)));
    erode(imageOut, imageOut, erodeElement, Point(-1, -1), 1);
	dilate(imageOut, imageOut, dilateElement, Point(-1, -1), 1);

	// Use Ostu adaptive thresholding.  This will turn
	// the gray scale image into a binary black and white one, with pixels
	// above some value being forced white and those below forced to black
	// The value to used as the split between black and white is returned
	// from the function.  If this value is too low, it means the image is
	// really dark and the returned threshold image will be mostly noise.
	// In that case, skip processing it entirely.
	const double otsuThreshold = threshold(imageOut, imageOut, 0., 255., CV_THRESH_BINARY | CV_THRESH_OTSU);
	//const double otsuThreshold = threshold(imageOut, imageOut, 20., 255., CV_THRESH_BINARY);
#ifdef VERBOSE
	cout << "OTSU THRESHOLD " << otsuThreshold << endl;
#endif
	if (otsuThreshold < _otsu_threshold)
		return false;
    return countNonZero(imageOut) != 0;
}

// Use the camera FOV, a known target size and the apparent size to
// estimate distance to a target
float GoalDetector::distanceUsingFOV(ObjectType _goal_shape, const Rect &rect) const
{
	float percent_image = (float)rect.height / _frame_size.height;
	float size_fov = percent_image * _fov_size.y; //TODO fov size
	return _goal_shape.height() / (2.0 * tanf(size_fov / 2.0));
}

float GoalDetector::distanceUsingFixedHeight(const Rect &/*rect*/, const Point &center, float expected_delta_height) const {
	/*
	cout << "Center: " << center << endl;
	float percent_image = ((float)center.y - (float)_frame_size.height/2.0)  / (float)_frame_size.height;
	cout << "Percent Image: " << percent_image << endl;
	cout << "FOV Size: " << _fov_size << endl;
	float size_fov = (percent_image * 1.3714590199999497) + (((float)_camera_angle/10.0) * (M_PI/180.0));
	cout << "Size FOV: " << size_fov << endl;
	cout << "Depth: " << expected_delta_height / (2.0 * tanf(size_fov/2.0)) << endl;
	return expected_delta_height / (2.0 * tanf(size_fov / 2.0));*/

	/*float focal_length_px = 0.5 * _frame_size.height / tanf(_fov_size.y / 2.0);
	cout << "Focal Length: " << focal_length_px << endl;
	float distance_centerline = (expected_delta_height * focal_length_px) / ((float)center.y - (float)_frame_size.height/2);
	cout << "Distance to centerline: " << distance_centerline << endl;
	float distance_ground = cos((_camera_angle/10.0) * (M_PI/180.0)) * distance_centerline;
	cout << "Distance to ground: " << distance_ground << endl;
	return distance_ground; */

	//float focal_length_px = 750.0;
	const float focal_length_px = (_frame_size.height / 2.0) / tanf(_fov_size.y / 2.0);
	const float to_center = _frame_size.height / 2.0 - (float)center.y;
	const float distance_diagonal = (focal_length_px * expected_delta_height) / (focal_length_px * sin((_camera_angle/10.0) * (M_PI/180.0)) + to_center);
	return distance_diagonal;
}

vector< GoalFound > GoalDetector::return_found(void) const
{
	return _return_found;
}



bool GoalDetector::Valid(void) const
{
	return _isValid;
}


// Draw debugging info on frame - all non-filtered contours
// plus their confidence. Highlight the best bounding rect in
// a different color
void GoalDetector::drawOnFrame(Mat &image, const vector<vector<Point>> &contours, const std::vector< GoalFound > &goals) const
{

	for (size_t i = 0; i < contours.size(); i++)
	{
		drawContours(image, contours, i, Scalar(0,0,255), 3);
#if 0
		Vec4f fit_line;
		fitLine(contours[i], fit_line, CV_DIST_L2, 0, 0.01, 0.01);

		const float vx = fit_line[0];
		const float vy = fit_line[1];
		const float x = fit_line[2];
		const float y = fit_line[3];
		const float leftY =((-x * vy / vx) + y);
		const float rightY =(((image.cols- x) * vy / vx) + y);

		const float angle = atan2(vy, vx);
		cout << "   image: " << image.size() << endl;
		cout << "   leftY: " << leftY << endl;
		cout << "   rightY: " << rightY << endl;
		cout << "   angle: " << angle * 180. / M_PI << endl;

		if ((fabs(vx) > 1e-5) && (fabs(vy) > 1e-5))
			line(image, Point2f(image.cols - 1, rightY), Point2f(0 ,leftY), Scalar(0,128,0), 2, CV_AA);
#endif
		Rect br(boundingRect(contours[i]));
		//rectangle(image, br, Scalar(255,0,0), 3);
		putText(image, to_string(i), br.br(), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0));
	}
	//if(!(_pastRects[_pastRects.size() - 1] == SmartRect(Rect()))) {



	//creates a rotated rectangle by drawing four lines. Useful for finding the angle with the horizontal.
	/*
	vector<RotatedRect> minRect(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(Mat(contours[i]));
		float tape_angle = minRect[i].angle;

		vector<Point> points;
		Point2f vtx[4];
		minRect[i].points(vtx);
		for (int i = 0; i < 4; i++)
			line(image, vtx[i], vtx[(i+1)%4], Scalar(153,50,204), 2);
	}
	*/
	for(size_t i = 0; i < goals.size(); i++)
	{
		const auto lr = goals[i].rect;
		rectangle(image, lr, Scalar(0,255,0), 3);
		putText(image, goals[i].id, lr.br(), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255));
	}
}

// Look for the N most recent detected rectangles to be
// the same before returning them as valid. This makes sure
// the camera has stopped moving and has settled
// TODO : See if we want to return different values for
// several frames which have detected goals but at different
// locations vs. several frames which have no detection at all
// in them?
void GoalDetector::isValid()
{
#if 0
	SmartRect currentRect = _pastRects[0];
	for(auto it = _pastRects.begin() + 1; it != _pastRects.end(); ++it)
	{
		if(!(*it == currentRect))
		{
			_isValid = false;
			return;
		}
	}
	_isValid = true;
#endif
}

void GoalDetector::setCameraAngle(double camera_angle)
{
	_camera_angle = camera_angle * 10;
}

void GoalDetector::setBlueScale(double blue_scale)
{
	_blue_scale = blue_scale * 100;
}
void GoalDetector::setRedScale(double red_scale)
{
	_red_scale = red_scale * 100;
}

void GoalDetector::setOtsuThreshold(int otsu_threshold)
{
	_otsu_threshold = otsu_threshold;
}

void GoalDetector::setMinConfidence(double min_valid_confidence)
{
	_min_valid_confidence = min_valid_confidence;
}

void GoalDetector::setTargetNum(ObjectNum target_num)
{
	_target_num = target_num;
}

const string GoalDetector::getObjectId(ObjectNum type)
{
	switch (type)
		{
		    case POWER_PORT_2020:
						return "power_port_2020";
		    case LOADING_BAY_2020:
		        return "loading_bay_2020";
		    default:
						return "unknown_type";
		}
}
