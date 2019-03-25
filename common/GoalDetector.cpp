#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include "GoalDetector.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace cv;

#define VERBOSE
#define VERBOSE_BOILER

GoalDetector::GoalDetector(const cv::Point2f &fov_size, const cv::Size &frame_size, bool gui) :
	_fov_size(fov_size),
	_frame_size(frame_size),
	_isValid(false),
	_min_valid_confidence(0.60),
	_otsu_threshold(5),
	_blue_scale(90),
	_red_scale(80),
	_camera_angle(-250) // in tenths of a degree
{
	if (gui)
	{
		cv::namedWindow("Goal Detect Adjustments", CV_WINDOW_NORMAL);
		createTrackbar("Blue Scale","Goal Detect Adjustments", &_blue_scale, 100);
		createTrackbar("Red Scale","Goal Detect Adjustments", &_red_scale, 100);
		createTrackbar("Otsu Threshold","Goal Detect Adjustments", &_otsu_threshold, 255);
		createTrackbar("Camera Angle","Goal Detect Adjustments", &_camera_angle, 900);
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
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

void GoalDetector::findBoilers(const cv::Mat& image, const cv::Mat& depth) {
	clear();
	const vector<vector<Point>> goal_contours = getContours(image);
	if (goal_contours.size() == 0)
		return;
	const vector<DepthInfo> goal_depths = getDepths(depth,goal_contours, LEFT_CARGO_2019, ObjectType(LEFT_CARGO_2019).real_height());

	//compute confidences for both the left piece of
	//tape and the right piece of tape
	const vector<GoalInfo> left_info = getInfo(goal_contours,goal_depths,LEFT_CARGO_2019);
	if(left_info.size() == 0)
		return;
	const vector<GoalInfo> right_info = getInfo(goal_contours,goal_depths,RIGHT_CARGO_2019);
	if(right_info.size() == 0)
		return;
#ifdef VERBOSE
	cout << left_info.size() << " left goals found and " << right_info.size() << " right"  << endl;
#endif

	//loop through every combination of left and right goal and check for the following conditions:
	//left is actually to the left of right
	//confidences are higher than any previous one
	for(size_t i = 0; i < left_info.size(); i++) {
		for(size_t j = 0; j < right_info.size(); j++) {
#ifdef VERBOSE_BOILER
			cout << "i:" << i << " j:" << j << endl;
			cout << left_info[i].contour_index << " " << right_info[j].contour_index << " cidx" << endl;
#endif
			// Index of the contour corrsponding to
			// left and right goal. Since we filter out
			// some contours prior to testing goal data
			// these can be different than i&j
			const int left_vindex = left_info[i].contour_index;
			const int right_vindex = right_info[j].contour_index;
			if (left_vindex == right_vindex)
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << left_vindex << " " << right_vindex << " same contour" << endl;
#endif
				continue;
			}

			if(left_info[i].com.x > right_info[j].com.x)
			{
#ifdef VERBOSE_BOILER
				cout << "Left too far to the right " << left_info[i].com.x << " " << right_info[j].com.x << endl;
#endif
				continue;
			}

#if 0 // This is the same check as above
			if(right_info[j].pos.x < left_info[i].pos.x)
			{
#ifdef VERBOSE_BOILER
				cout << "Right goal too far to the left" << endl;
#endif
				continue;
			}
#endif

			// Make sure the goal parts are reasonably close
			// together on the screen and proportionally accurate to the tapes.
			const float screendx = left_info[i].com.x - right_info[j].com.x;
			const float screendy = left_info[i].com.y - right_info[j].com.y;
			const float screenDist = sqrtf(screendx * screendx + screendy * screendy);

			if (screenDist > (3.0 * (left_info[i].br.width + right_info[i].br.width)))
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << screenDist << " max screen dist check failed" << endl;
#endif
				continue;
			}

			if (screenDist < (0.5 * (left_info[i].br.width + right_info[i].br.width)))
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << screenDist << " min screen dist check failed" << endl;
#endif
				continue;
			}

			const Rect leftBr = left_info[i].br;
			const Rect rightBr = right_info[j].br;

#ifdef VERBOSE_BOILER
			cout << leftBr << " " << rightBr << endl;
#endif
			if ((leftBr & rightBr).area() > 0)
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << " ovelapping bounding rects failed" << endl;
#endif
				continue;
			}

			// Make sure the two contours are
			// similar in size
			//const float area_ratio = (float)leftBr.area() / rightBr.area();
			const float area_ratio = contourArea(goal_contours[left_vindex]) / contourArea(goal_contours[right_vindex]);
			constexpr float max_area_ratio = 4;
			if ((area_ratio > max_area_ratio) || (area_ratio < (1. / max_area_ratio)))
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << area_ratio << " screen area ratio failed" << endl;
#endif
				continue;
			}

			// Make sure the right contour overlaps at least
			// part of the left contour
			if ((leftBr.br().y - (leftBr.height * 0.)) < rightBr.y)
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << leftBr.br().y << " " << leftBr.height << " "  << rightBr.y << " stacked check 1 failed" << endl;
#endif
				continue;
			}

			if ((leftBr.y + (leftBr.height * 0.)) > rightBr.br().y)
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << leftBr.y << " " << leftBr.height << " "  << rightBr.br().y << " stacked check 2 failed" << endl;
#endif
				continue;
			}

			// Make sure the left contour overlaps at least
			// part of the right contour
			if ((rightBr.br().y - (rightBr.height * 0.)) < leftBr.y)
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << rightBr.br().y << " " << rightBr.height << " "  << leftBr.y << " stacked check 3 failed" << endl;
#endif
				continue;
			}

			if ((rightBr.y + (rightBr.height * 0.)) > leftBr.br().y)
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " " << rightBr.y << " " << rightBr.height << " "  << leftBr.br().y << " stacked check 4 failed" << endl;
#endif
				continue;
			}

			// Only do distance checks if we believe the
			// depth info is correct
			if (!left_info[i].depth_error && !right_info[j].depth_error)
			{
/*
				if (fabsf(left_info[i].angle) + fabsf(right_info[j].angle) < 50.0)
				{
#ifdef VERBOSE_BOILER
					cout << i << " " << j << " angle compare failed" << endl;
#endif
					continue;
				}
*/
				// Make sure there isn't too much
				// distance left to right between the goals
				const float dx = left_info[i].pos.x - right_info[j].pos.x;
				if (fabsf(dx) > .5)
				{
#ifdef VERBOSE_BOILER
					cout << i << " " << j << " " << dx << " dx check failed" << endl;
#endif
					continue;
				}
				const float dy = left_info[i].pos.y - right_info[j].pos.y;
				const float dz = left_info[i].pos.z - right_info[j].pos.z;
				const float dist = sqrt(dx * dx + dy * dy + dz * dz);
				if (dist > 1.25)
				{
#ifdef VERBOSE_BOILER
					cout << i << " " << j << " " << dist << " distance check failed" << endl;
#endif
					continue;
				}
			}
				/*
				// Keep detecting the goal, even if something gets in the way to
				// obstruct the goal.

				if (!found_goal || left_info[i].confidence[i] )
				{
#ifdef VERBOSE_BOILER
					cout << i << " " << j << " " <<  << " obstruction detected, detecting previous goal" << endl;
#endif
					continue;
				} */
/*
			rtRect[i] =
			_goal_left_rotated_rect = rtRect[left_info[i].contour_index];
			_goal_right_rotated_rect =rtRect[right_info[i].contour_index];
			vector<Point> points;
			Point2f vtx[4];
			rtR ect[i].points(vtx);
			for (int idx = 0; idx < 4; idx++)
				line(image, vtx[idx], vtx[(idx+1)%4], Scalar(153,50,204), 2);
*/
			Point2f intersection_point;

			if (!intersection(left_info[i].lineStart, left_info[i].lineEnd,
			                  right_info[j].lineStart, right_info[j].lineEnd,
							  intersection_point) ||
					(intersection_point.y > std::max(left_info[i].com.y, right_info[j].com.y)))
			{
				cout << i << " " << j << " intersection point below com of contours : " << intersection_point << endl;
				continue;
			}


#if 0
			// minAreaRect returns a multiple of 90 if it can't ID a rectangle's
			// orientation. If so, don't perform the angle check
			if((fabs(fmod(left_info[i].rtRect.angle, 90)) != 0.) &&
			   (fabs(fmod(right_info[j].rtRect.angle, 90)) != 0.))
			{
				// Normalize angles to -45 .. 45
				double lAngle = left_info[i].rtRect.angle;
				double rAngle = right_info[j].rtRect.angle;
				while (lAngle >= 45)
					lAngle -= 90;
				while (lAngle <= -45)
					lAngle += 90;
				while (rAngle >= 45)
					rAngle -= 90;
				while (rAngle <= -45)
					rAngle += 90;

				// Make sure contours are pointed in opposite directions
				if (signbit(lAngle) == signbit(rAngle))
				{
#ifdef VERBOSE_BOILER
					cout << i << " " << j << " Angle sign check failed " <<
						left_info[i].rtRect.angle  << " " <<
						right_info[j].rtRect.angle << " " <<
						lAngle << " " <<
						rAngle << endl;
#endif
					continue;
				}
				if (left_info[i].rtRect.angle > right_info[j].rtRect.angle)
				{
#ifdef VERBOSE_BOILER
					cout << i << " " << j << " Angle check failed " <<
						fabs(fmod(left_info[i].rtRect.angle, 90))  << " " <<
						fabs(fmod(right_info[j].rtRect.angle, 90)) << " " <<
						left_info[i].rtRect.angle  << " " <<
						right_info[j].rtRect.angle << " " <<
						lAngle << " " <<
						rAngle << endl;
#endif
					continue;
				}
			}
#endif


			//_goal_left_rotated_rect =  minAreaRect(Mat(goal_contours[left_info[i].contour_index]));
			//_goal_right_rotated_rect = minAreaRect(Mat(goal_contours[right_info[j].contour_index]));

			// This doesn't work near the edges of the frame?
			if ((left_info[i].rect & right_info[j].rect).area() > (.5 * min(left_info[i].rect.area(), right_info[j].rect.area())))
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " overlap check failed" << endl;
#endif
				continue;
			}

			// If this is the first valid pair
			// or if this pair has a higher combined
			// confidence than the previously saved
			// pair, keep it as the best result
			if ((left_info[i].confidence + right_info[j].confidence) > _min_valid_confidence)
			{
				GoalFound goal_found;
				goal_found.pos.x               = left_info[i].pos.x + ((right_info[j].pos.x - left_info[i].pos.x) / 2.);
				goal_found.pos.y			   = left_info[i].pos.y + ((right_info[j].pos.y - left_info[i].pos.y) / 2.);
				goal_found.pos.z			   = left_info[i].pos.z + ((right_info[j].pos.z - left_info[i].pos.z) / 2.);
				goal_found.left_pos            = left_info[i].pos;
				goal_found.right_pos           = right_info[i].pos;
				goal_found.distance            = sqrt(goal_found.pos.x * goal_found.pos.x + goal_found.pos.y * goal_found.pos.y);
				goal_found.angle               = atan2f(goal_found.pos.x, goal_found.pos.y) * 180. / M_PI;
				goal_found.confidence          = left_info[i].confidence + right_info[j].confidence;
				goal_found.left_contour_index  = left_info[i].contour_index;
				goal_found.right_contour_index = right_info[j].contour_index;
				goal_found.left_rect		   = left_info[i].rect;
				goal_found.right_rect		   = right_info[j].rect;
				goal_found.left_rotated_rect   = left_info[i].rtRect;
				goal_found.right_rotated_rect  = right_info[j].rtRect;

				//These are the saved values for the best goal before moving on to
				//try and find another one.
				bool repeated = false;
				if(_return_found.size() > 0)
				{
					const double min_dist_bwn_goals = 0.1;
					size_t gf_lci = goal_found.left_contour_index;
					size_t gf_rci = goal_found.right_contour_index;
					for(size_t k = 0; k < _return_found.size(); k++)
					{
						size_t rf_lci = _return_found[k].left_contour_index;
						size_t rf_rci = _return_found[k].right_contour_index;
						// compare contour indexes of goal_found vs _return_found[k].  If neither
						// match, this can't be a repeated goal so add it to return_found and continue.
						if ((gf_lci != rf_lci) && (gf_lci != rf_rci) &&
						    (gf_rci != rf_lci) && (gf_rci != rf_rci))
						{
							continue;
						}
						repeated = true;
						// TODO : next, check distance between (goal_found.right_pos.x - goal_found.left_pos.x) and
						// (return_found[k].right_pos.x - return_found[k].left_pos.x).  If the goal_found
						// distance is shorter, replace return_found with goal_found. This would be the case
						// where one contour is shared between both, but goal_found has closer second contour
						// than the one in return_found
						if ((goal_found.right_pos.x - goal_found.left_pos.x) < (_return_found[k].right_pos.x - _return_found[k].left_pos.x))
						{
							_return_found[k] = goal_found;
							break;
						}
						// TODO : then, if confidence is higher for goal_found compared to return_found[k],
						// replace return_found[k] with goal info.  This might not be needed
						// TODO : otherwise, discard goal_found since the previously found goal in return_found
						// was closer to the ideal goal
					#if 0 
						if(abs(left_info[i].pos.x - _return_found[k].pos.x) < min_dist_bwn_goals)
						{
							break;
						}
						for(size_t l = 0; l < _return_found.size(); l++)
						{
							if(abs(left_info[i].pos.x - _return_found[l].pos.x) < min_dist_bwn_goals)
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

				cout << "Number of goals: " << _return_found.size() << endl;
				for(size_t n = 0; n < _return_found.size(); n++)
				{
					cout << "Goal " << n + 1 << " " << _return_found[n].left_contour_index << " " <<
						_return_found[n].right_contour_index << " pos: " << _return_found[n].pos <<
						" distance: " << _return_found[n].distance << " angle: " << _return_found[n].angle << endl;
				}
			}
			else
			{
#ifdef VERBOSE_BOILER
				cout << i << " " << j << " confidence too low " <<
					left_info[i].confidence  << " " << right_info[j].confidence << endl;
#endif
			}
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
	Mat threshold_image;
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

const vector<DepthInfo> GoalDetector::getDepths(const Mat &depth, const vector< vector< Point > > &contours, ObjectNum objtype, float expected_height) {
	// Use to mask the contour off from the rest of the
	// image - used when grabbing depth data for the contour
	Mat contour_mask(_frame_size, CV_8UC1, Scalar(0));
	vector<DepthInfo> return_vec;
	DepthInfo depthInfo;
	for(size_t i = 0; i < contours.size(); i++) {
		// get the minimum and maximum depth values in the contour,
		const Rect br(boundingRect(contours[i]));
		const Moments mu = moments(contours[i], false);
		const Point com = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
		//Point center(rect.tl().x+rect.size().width/2, rect.tl().y+rect.size().height/2);

		//create a mask which is the same shape as the contour
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
			ObjectType ot(objtype);
			depth_z_min = depth_z_max = distanceUsingFixedHeight(br,com,expected_height);
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
	vector<GoalInfo> return_info;
	// Create some target stats based on our idealized goal model
	//center of mass as a percentage of the object size from left left
	const Point2f com_percent_expected(goal_shape.com().x / goal_shape.width(),
									   goal_shape.com().y / goal_shape.height());

	// Ratio of contour area to bounding box area
	const float filledPercentageExpected = goal_shape.area() / goal_shape.boundingArea();

	// Aspect ratio of the goal
	const float expectedRatio = goal_shape.width() / goal_shape.height();

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
		if ((actualRatio < .20) || (actualRatio > 1.0))
		{
#ifdef VERBOSE
			cout << "Contour " << i << " height/width ratio fail" << br.size() << " " << actualRatio << endl;
#endif
			continue;
		}

#if 0
		// TODO : Figure out how well this works in practice
		// Filter out goals which are too close or too far
		if (!depth_maxs[i].error && (6.2 < depth_maxs[i].depth || depth_maxs[i].depth < .1))
		{
#ifdef VERBOSE
			cout << "Contour " << i << " depth out of range " << depth_maxs[i].depth << endl;
#endif
			continue;
		}
#endif
		// Fit a line to the countor, calculate the start and end points on screen
		// for the line.
		Vec4f fit_line;
		fitLine(contours[i], fit_line, CV_DIST_L2, 0, 0.01, 0.01);

		const float vx = fit_line[0];
		const float vy = fit_line[1];
		const float x = fit_line[2];
		const float y = fit_line[3];
		const float leftY =((-x * vy / vx) + y);
		const float rightY =(((_frame_size.width - x) * vy / vx) + y);
		const float angle = atan2(vy, vx);
#if 0
		cout << "fit_line: " << fit_line << endl;
		cout << "   frame_size " << _frame_size << endl;
		cout << "   leftY: " << leftY << endl;
		cout << "   rightY: " << rightY << endl;
		cout << "   angle: " << angle * 180. / M_PI << endl;
#endif
		const Point2f start_line(_frame_size.width - 1, rightY);
		const Point2f end_line(0, leftY);

		//create a trackedobject to get various statistics
		//including area and x,y,z position of the goal
		ObjectType goal_actual(contours[i], "Actual Goal", 0);
		TrackedObject goal_tracked_obj(0, goal_shape, br, depth_maxs[i].depth, _fov_size, _frame_size,-((float)_camera_angle/10.) * M_PI / 180.0);

		// Gets the bounding box area observed divided by the
		// bounding box area calculated given goal size and distance
		// For an object the size of a goal we'd expect this to be
		// close to 1.0 with some variance due to perspective
		const float exp_area = goal_tracked_obj.getScreenPosition(_fov_size, _frame_size).area();
		const float actualScreenArea = (float)br.area() / exp_area;
/*
		if (((exp_area / br.area()) < 0.20) || ((exp_area / br.area()) > 5.00))
		{
#ifdef VERBOSE
			cout << "Contour " << i << " area out of range for depth (depth/act/exp/ratio):" << depth_maxs[i].depth << "/" << br.area() << "/" << exp_area << "/" << actualScreenArea << endl;
#endif
			continue;
		}*/
		//percentage of the object filled in
		float filledPercentageActual = goal_actual.area() / goal_actual.boundingArea();

		//center of mass as a percentage of the object size from left and top
		Point2f com_percent_actual((goal_actual.com().x - br.tl().x) / goal_actual.width(),
								   (goal_actual.com().y - br.tl().y) / goal_actual.height());

		/* I don't think this block of code works but I'll leave it in here
		Mat test_contour = Mat::zeros(640,640,CV_8UC1);
		std::vector<Point> upscaled_contour;
		for(int j = 0; j < goal_shape.shape().size(); j++) {
			upscaled_contour.push_back(Point(goal_shape.shape()[j].x * 100, goal_shape.shape()[j].y * 100));
			cout << "Upscaled contour point: " << Point(goal_shape.shape()[j].x * 100, goal_shape.shape()[j].y * 100) << endl;
			}
		std::vector< std::vector<Point> > upscaledcontours;
		upscaledcontours.push_back(upscaled_contour);
		drawContours(test_contour, upscaledcontours, 0, Scalar(0,0,0));
		imshow("Goal shape", test_contour);
		*/

		//parameters for the normal distributions
		//values for standard deviation were determined by
		//taking the standard deviation of a bunch of values from the goal
		//confidence is near 0.5 when value is near the mean
		//confidence is small or large when value is not near mean
		const float confidence_height      = createConfidence(goal_shape.real_height(), 0.2, goal_tracked_obj.getPosition().z - goal_shape.height() / 2.0);
		const float confidence_com_x       = createConfidence(com_percent_expected.x, 0.125,  com_percent_actual.x);
		const float confidence_com_y       = createConfidence(com_percent_expected.y, 0.125,  com_percent_actual.y);
		const float confidence_filled_area = createConfidence(filledPercentageExpected, 0.33, filledPercentageActual);
		const float confidence_ratio       = createConfidence(expectedRatio, 1.5,  actualRatio);
		const float confidence_screen_area = createConfidence(1.0, 1.50, actualScreenArea);

		// higher is better
		const float confidence = (confidence_height + confidence_com_x + confidence_com_y + confidence_filled_area + confidence_ratio/2. + confidence_screen_area) / 5.5;

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
		//cout << "Area exp/act: " << (int)exp_area << "/" << br.area() << endl;
		cout << "Aspect ratio exp/act : " << expectedRatio << "/" << actualRatio << endl;
		cout << "br: " << br << endl;
		cout << "com: " << goal_actual.com() << endl;
		cout << "com_expected / actual: " << com_percent_expected << " " << com_percent_actual << endl;
		cout << "position: " << goal_tracked_obj.getPosition() << endl;
		//cout << "Angle: " << minAreaRect(contours[i]).angle << endl;
		cout << "lineStart: " << start_line << endl;
		cout << "lineEnd: " << end_line << endl;
		cout << "-------------------------------------------" << endl;
#endif

		GoalInfo goal_info;

		// This goal passes the threshold required for us to consider it a goal
		// Add it to the list of best goals
		goal_info.pos           = goal_tracked_obj.getPosition();
		goal_info.confidence    = confidence;
		goal_info.distance      = depth_maxs[i].depth * cosf((_camera_angle/10.0) * (M_PI/180.0));
		goal_info.angle		    = atan2f(goal_info.pos.x, goal_info.pos.y) * 180. / M_PI;
		goal_info.rect		    = br;
		goal_info.contour_index = i;
		goal_info.depth_error   = depth_maxs[i].error;
		goal_info.com           = goal_actual.com();
		goal_info.br            = br;
		//goal_info.rtRect        = rr;
		goal_info.lineStart     = start_line;
		goal_info.lineEnd       = end_line;
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
    vector<Mat> splitImage;
    Mat         bluePlusRed;

    split(imageIn, splitImage);
	addWeighted(splitImage[0], _blue_scale / 100.0,
			    splitImage[2], _red_scale / 100.0, 0.0,
				bluePlusRed);
	subtract(splitImage[1], bluePlusRed, imageOut);

    static const Mat erodeElement(getStructuringElement(MORPH_RECT, Size(3, 3)));
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
void GoalDetector::drawOnFrame(Mat &image, const vector<vector<Point>> &contours) const
{

	for (size_t i = 0; i < contours.size(); i++)
	{
		drawContours(image, contours, i, Scalar(0,0,255), 3);

		Vec4f fit_line;
		fitLine(contours[i], fit_line, CV_DIST_L2, 0, 0.01, 0.01);

		const float vx = fit_line[0];
		const float vy = fit_line[1];
		const float x = fit_line[2];
		const float y = fit_line[3];
		const float leftY =((-x * vy / vx) + y);
		const float rightY =(((image.cols- x) * vy / vx) + y);
#if 0
		const float angle = atan2(vy, vx);
		cout << "fit_line: " << fit_line << endl;
		cout << "   image: " << image.size() << endl;
		cout << "   leftY: " << leftY << endl;
		cout << "   rightY: " << rightY << endl;
		cout << "   angle: " << angle * 180. / M_PI << endl;
#endif
		if ((fabs(vx) > 1e-5) && (fabs(vy) > 1e-5))
			line(image, Point2f(image.cols - 1, rightY), Point2f(0 ,leftY), Scalar(0,128,0), 2);

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
	for(size_t i = 0; i < _return_found.size(); i++)
	{
		const auto lr =_return_found[i].left_rect;
		const auto rr =_return_found[i].right_rect;
		rectangle(image, lr, Scalar(0,255,0), 3);
		rectangle(image, rr, Scalar(0,140,255), 3);
		line(image,
			 Point(lr.x + lr.width / 2.0, lr.y + lr.height / 2.0),
			 Point(rr.x + rr.width / 2.0, lr.y + lr.height / 2.0),
			 Scalar(0, 140, 255), 3);
		const double center_x = (lr.x + lr.width / 2.0 + rr.x + rr.width / 2.0) / 2.0;
		const double center_y =  lr.y + lr.height / 2.0;
		line(image,
			 Point(center_x, center_y - 30.), Point(center_x, center_y + 30.),
			 Scalar(0, 140, 255), 3);
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

