#include <iostream>
#include <limits>

#include "field_obj_tracker/track3d.hpp"
#include "hungarian.hpp"

using namespace std;
using namespace cv;

// How many consecutive frames a track must be missing before
// it is erased
constexpr int missedFrameCountMax = 10;

// Constructor for a tracked object
// Each object holds a unique ID for the tracked object
// It is constructed with an object type, information
// to locate it in space, and some params for the
// Kalman Filter used to track it
TrackedObject::TrackedObject(int                                       id,
							 const ObjectType                         &type_in,
							 const Rect                               &screen_position,
							 double                                    avg_depth,
							 const image_geometry::PinholeCameraModel &model,
							 float                                     dt,
							 float                                     accel_noise_mag,
							 size_t                                    historyLength) :
		type_(type_in),
		detectHistory_(historyLength),
		positionHistory_(historyLength),
		KF_(type_.screenToWorldCoords(screen_position, avg_depth, model),
			dt, accel_noise_mag),
		missedFrameCount_(0)
{
	// Set the robot-relative x,y,z coord of the object, and mark
	// it as detected for this frame of video
	setPosition(screen_position, avg_depth, model);
	setDetected();

	// Label with base-26 letter ID (A, B, C .. Z, AA, AB, AC, etc)
	do
	{
		id_ += (char)(id % 26 + 'A');
		id /= 26;
	}
	while (id != 0);
	reverse(id_.begin(), id_.end());
}


#if 0
TrackedObject::~TrackedObject()
{
}
#endif

// Set the position based on x,y,z coords

void TrackedObject::setPosition(const Point3f &new_position)
{
	position_ = new_position;
	addToPositionHistory(position_);
}

// Set the position based on a rect on the screen and depth info from the zed
void TrackedObject::setPosition(const Rect &screen_position, double avg_depth,
		                        const image_geometry::PinholeCameraModel &model)
{
	setPosition(type_.screenToWorldCoords(screen_position, avg_depth, model));
}

#if 0
void TrackedObject::adjustPosition(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot)
{
	//Eigen::AngleAxisd rot(0.5*M_PI, Eigen::Vector3d::UnitZ());

	Eigen::Vector3d old_pos_vec(_position.x, _position.y, _position.z);
	Eigen::Vector3d new_pos_vec = delta_robot * old_pos_vec;

	position_ = Point3f(new_pos_vec[0], new_pos_vec[1], new_pos_vec[2]);

	for (auto it = positionHistory_.begin(); it != positionHistory_.end(); ++it)
	{
		Eigen::Vector3d old_pos_vector(it->x, it->y, it->z);
		Eigen::Vector3d new_pos_vector = delta_robot * old_pos_vector;
		*it = Point3f(new_pos_vector[0], new_pos_vector[1], new_pos_vector[2]);
	}
}
#endif
// TODO - do this without going back and forth between screen and world coords
void TrackedObject::adjustPosition(const Mat &transform_mat, float depth, const image_geometry::PinholeCameraModel &model)
{
	//get the position of the object on the screen
	Rect screen_rect = getScreenPosition(model);
	Point screen_pos(screen_rect.tl().x + screen_rect.width / 2, screen_rect.tl().y + screen_rect.height / 2);

	//create a matrix to hold positon for matrix multiplication
	Mat pos_mat(3,1,CV_64FC1);
	pos_mat.at<double>(0,0) = screen_pos.x;
	pos_mat.at<double>(0,1) = screen_pos.y;
	pos_mat.at<double>(0,2) = 1.0;

	//correct the position
	Mat new_screen_pos_mat(3,1,CV_64FC1);
	new_screen_pos_mat = transform_mat * pos_mat;
	Point new_screen_pos(new_screen_pos_mat.at<double>(0),new_screen_pos_mat.at<double>(1));

	//create a dummy bounding rect because setPosition requires a bounding rect as an input rather than a point
	Rect new_screen_rect(new_screen_pos.x,new_screen_pos.y,0,0);
	setPosition(new_screen_rect,depth,model);
	//update the history
	for (auto it = positionHistory_.begin(); it != positionHistory_.end(); ++it)
	{
		screen_rect = type_.worldToScreenCoords(*it, model);
		screen_pos = Point(screen_rect.tl().x + screen_rect.width / 2, screen_rect.tl().y + screen_rect.height / 2);
		pos_mat.at<double>(0,0) = screen_pos.x;
		pos_mat.at<double>(0,1) = screen_pos.y;
		pos_mat.at<double>(0,2) = 1.0;
		new_screen_pos_mat = transform_mat * pos_mat;
		new_screen_pos = Point(new_screen_pos_mat.at<double>(0),new_screen_pos_mat.at<double>(1));
		new_screen_rect = Rect(new_screen_pos.x,new_screen_pos.y,0,0);
		*it = type_.screenToWorldCoords(new_screen_rect, depth, model);
	}
}

// Mark the object as detected in this frame
void TrackedObject::setDetected(void)
{
	detectHistory_.push_back(true);
	missedFrameCount_ = 0;
}

// Clear the object detect flag for this frame.
// Probably should only happen when moving to a new
// frame, but may be useful in other cases
void TrackedObject::clearDetected(void)
{
	detectHistory_.push_back(false);
	missedFrameCount_ += 1;
}

bool TrackedObject::tooManyMissedFrames(void) const
{
	// Hard limit on the number of consecutive missed
	// frames before dropping a track
	if (missedFrameCount_ > missedFrameCountMax)
		return true;

	// Be more aggressive about dropping tracks which
	// haven't been around long - kill them off if
	// they are seen in less than 33% of frames
	if (detectHistory_.size() <= 10)
	{
		size_t detectCount = 0;
		for (auto it = detectHistory_.begin();  it != detectHistory_.end(); ++it)
			if (*it)
				detectCount += 1;
		if (((double)detectCount / detectHistory_.size()) <= 0.34)
			return true;
	}
	return false;
}

// Keep a history of the most recent positions
// of the object in question
void TrackedObject::addToPositionHistory(const Point3f &pt)
{
	positionHistory_.push_back(pt);
}

// Return a vector of points of the position history
// of the object (hopefully) relative to current screen location
vector <Point> TrackedObject::getScreenPositionHistory(const image_geometry::PinholeCameraModel &model) const
{
	vector <Point> ret;

	for (auto it = positionHistory_.begin(); it != positionHistory_.end(); ++it)
	{
		Rect screen_rect(type_.worldToScreenCoords(*it, model));
		ret.push_back(Point(cvRound(screen_rect.x + screen_rect.width / 2.),cvRound( screen_rect.y + screen_rect.height / 2.)));
	}
	return ret;
}

constexpr double minDisplayRatio = 0.3;

// Return the percent of last detectHistory_.capacity() frames
// the object was seen
double TrackedObject::getDetectedRatio(void) const
{
	// Need at least 2 frames to believe there's something real
	if (detectHistory_.size() <= 1)
		return 0.01;

	// Don't display stuff which hasn't been detected recently.
	if (missedFrameCount_ >= 3)
		return 0.01;

	size_t detectCount = 0;
	for (auto it = detectHistory_.begin();  it != detectHistory_.end(); ++it)
		if (*it)
			detectCount += 1;

	// For newly added tracks make sure only 1 frame is missed at most
	// while the first quarter of the buffer is filled and at most
	// two are missed while filling up to half the size of the buffer
	if (detectHistory_.size() < (detectHistory_.capacity()/2))
	{
		if (detectCount < (detectHistory_.size() - 2))
			return 0.01;
		if ((detectHistory_.size() <= (detectHistory_.capacity()/4)) && (detectCount < (detectHistory_.size() - 1)))
			return 0.01;

		// Ramp up from minDisplayRatio so that at 10 hits it will
		// end up at endRatio = 10/20 = 50% or 9/20 = 45%
		// 2:2 = 32.5%
		// 3:3 = 35%
		// 4:4 = 37.5%
		// 5:5 = 40%
		// 6:6 = 42.5%
		// 7:7 = 45%
		// 8:8 = 47.5%
		// 9:9 = 50%
		double endRatio =  (detectHistory_.capacity() / 2.0 - (detectHistory_.size() - detectCount)) / detectHistory_.capacity();
		return minDisplayRatio + (detectHistory_.size() - 2.0) * (endRatio - minDisplayRatio) / (detectHistory_.capacity() / 2.0 - 2.0);
	}
	double detectRatio = (double)detectCount / detectHistory_.capacity();
	return detectRatio;
}


Rect TrackedObject::getScreenPosition(const image_geometry::PinholeCameraModel &model) const
{
	return type_.worldToScreenCoords(position_, model);
}


//fit the contour of the object into the rect of it and return the area of that
//kinda gimmicky but pretty cool and might have uses in the future
double TrackedObject::contourArea(const image_geometry::PinholeCameraModel &model) const
{
	const Rect screen_position = getScreenPosition(model);
	const float scale_factor_x = (float)screen_position.width / type_.width();
	const float scale_factor_y = (float)screen_position.height / type_.height();
	const float scale_factor   = min(scale_factor_x, scale_factor_y);

	vector<Point2f> scaled_contour;
	for(size_t i = 0; i < type_.shape().size(); i++)
	{
		scaled_contour.push_back(type_.shape()[i] * scale_factor);
	}

	return cv::contourArea(scaled_contour);
}


Point3f TrackedObject::predictKF(void)
{
	return KF_.GetPrediction();
}


Point3f TrackedObject::updateKF(Point3f pt)
{
	return KF_.Update(pt);
}

#if 0
void TrackedObject::adjustKF(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot)
{
	KF_.adjustPrediction(delta_robot);
}
#endif
void TrackedObject::adjustKF(Point3f delta_pos)
{
	KF_.adjustPrediction(delta_pos);
}


//Create a tracked object list
// those stay constant for the entire length of the run
TrackedObjectList::TrackedObjectList(const std::string &trackingBaseFrame)
	: detectCount_(0)
	, trackingBaseFrame_(trackingBaseFrame)
{
}
#if 0
// Adjust position for camera motion between frames using fovis
void TrackedObjectList::adjustLocation(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot)
{
	for (auto it = list_.begin(); it != list_.end(); ++it)
	{
		it->adjustPosition(delta_robot);
		it->adjustKF(delta_robot);
	}
}
#endif
// Adjust position for camera motion between frames using optical flow
void TrackedObjectList::adjustLocation(const Mat &transformMat, const image_geometry::PinholeCameraModel &model)
{
	for (auto it = list_.begin(); it != list_.end(); ++it)
	{
		//measure the amount that the position changed and apply the same change to the kalman filter
		const Point3f old_pos = it->getPosition();
		//compute r and use it for depth (assume depth doesn't change)
		const float r = sqrt(it->getPosition().x * it->getPosition().x + it->getPosition().y * it->getPosition().y + it->getPosition().z * it->getPosition().z);
		it->adjustPosition(transformMat, r, model);
		const Point3f delta_pos = it->getPosition() - old_pos;

		it->adjustKF(delta_pos);
	}
}

// Get position history for each tracked object
vector<vector<Point>> TrackedObjectList::getScreenPositionHistories(const image_geometry::PinholeCameraModel &model) const
{
	vector<vector<Point>> ret;
	for (auto it = list_.begin(); it != list_.end(); ++it)
		ret.push_back(it->getScreenPositionHistory(model));
	return ret;
}

// Simple printout of list into stdout
void TrackedObjectList::print(void) const
{
	for (auto it = list_.cbegin(); it != list_.cend(); ++it)
	{
		cout << it->getId() << " location ";
		const Point3f position = it->getPosition();
		cout << "(" << position.x << "," << position.y << "," << position.z << ")" << endl;
	}
}

// Return list of detect info for external processing
void TrackedObjectList::getDisplay(vector<TrackedObjectDisplay> &displayList, const image_geometry::PinholeCameraModel &model) const
{
	displayList.clear();
	TrackedObjectDisplay tod;
	for (auto it = list_.cbegin(); it != list_.cend(); ++it)
	{
		tod.id       = it->getId();
		tod.rect     = it->getScreenPosition(model);
		tod.ratio    = it->getDetectedRatio();
		tod.position = it->getPosition();
		tod.name	 = it->getType().name();
		displayList.push_back(tod);
	}
}

constexpr double dist_thresh_ = 1.0; // FIX ME!
//#define VERBOSE_TRACK

// Process a set of detected rectangles
// Each will either match a previously detected object or
// if not, be added as new object to the list
void TrackedObjectList::processDetect(const vector<Rect> &detectedRects,
									  const vector<float> &depths,
									  const vector<ObjectType> &types,
									  const image_geometry::PinholeCameraModel &model)
{
	vector<Point3f> detectedPositions;
#ifdef VERBOSE_TRACK
	if (!detectedRects.empty() || !list_.empty())
		cout << "---------- Start of process detect --------------" << endl;
	print();
	if (detectedRects.size() > 0)
		cout << detectedRects.size() << " detected objects" << endl;
#endif
	for (size_t i = 0; i < detectedRects.size(); i++)
	{
		detectedPositions.push_back(
				types[i].screenToWorldCoords(detectedRects[i], depths[i], model));
#ifdef VERBOSE_TRACK
		cout << "Detected rect [" << i << "] = " << detectedRects[i] << " positions[" << detectedPositions.size() - 1 << "]:" << detectedPositions[detectedPositions.size()-1] << endl;
#endif
	}
	// TODO :: Combine overlapping detections into one?

	// Maps tracks to the closest new detected object.
	// assignment[track] = index of closest detection
	vector<int> assignment;
	if (!list_.empty())
	{
		const size_t tracks = list_.size();		          // number of tracked objects from prev frames
		const size_t detections = detectedPositions.size(); // number of detections this frame

		//Cost[t][d] is the distance between old tracked location t
		//and newly detected object d's position
		vector< vector<double> > Cost(tracks,vector<double>(detections));

		// Calculate cost for each track->pair combo
		// The cost here is just the distance between them
		// Also check to see if the types are the same, if they are not then set the cost extremely high so that it's never matched
		auto it = list_.cbegin();
		for(size_t t = 0; t < tracks; ++t, ++it)
		{
			// Point3f prediction=tracks[t]->prediction;
			// cout << prediction << endl;
			for(size_t d = 0; d < detections; d++)
			{
				const ObjectType it_type = it->getType();
				if(types[d] == it_type) {
					Point3f diff = it->getPosition() - detectedPositions[d];
					Cost[t][d] = sqrtf(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
				} else {
					Cost[t][d] = numeric_limits<double>::max();
				}
			}
		}

		// Solving assignment problem (find minimum-cost assignment
		// between tracks and previously-predicted positions)
		AssignmentProblemSolver APS;
		APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);

#ifdef VERBOSE_TRACK
		// assignment[i] holds the index of the detection assigned
		// to track i.  assignment[i] is -1 if no detection was
		// matchedto that particular track
		cout << "After APS : "<<endl;
		for(size_t i = 0; i < assignment.size(); i++)
			cout << i << ":" << assignment[i] << endl;
#endif
		// clear assignment from pairs with large distance
		for(size_t i = 0; i < assignment.size(); i++)
			if ((assignment[i] != -1) && (Cost[i][assignment[i]] > dist_thresh_))
				assignment[i] = -1;
	}

	// Search for unassigned detects and start new tracks for them.
	// This will also handle the case where no tracks are present,
	// since assignment will be empty in that case - everything gets added
	for(size_t i = 0; i < detectedPositions.size(); i++)
	{
		if (find(assignment.begin(), assignment.end(), i) == assignment.end())
		{
#ifdef VERBOSE_TRACK
			cout << "New assignment created " << i << endl;
#endif
			list_.push_back(TrackedObject(detectCount_++, types[i], detectedRects[i], depths[i], model));

#ifdef VERBOSE_TRACK
			cout << "New assignment finished" << endl;
#endif
		}
	}

	auto tr = list_.begin();
	auto as = assignment.begin();
	while ((tr != list_.end()) && (as != assignment.end()))
	{
		// If track updated less than one time, than filter state is not correct.
		const Point3f prediction = tr->predictKF();
#ifdef VERBOSE_TRACK
		cout << "Predict: " << endl;
		cout << "prediction:" << prediction << endl;
#endif

		if(*as != -1) // If we have assigned detect, then update using its coordinates
		{
			tr->setPosition(tr->updateKF(detectedPositions[*as]));
#ifdef VERBOSE_TRACK
			cout << "Update match: " << endl;
			cout << tr->getScreenPosition(fovSize_, imageSize_) << endl;
#endif
			tr->setDetected();
		}
		else          // if not continue using predictions
		{
			tr->setPosition(tr->updateKF(prediction));
#ifdef VERBOSE_TRACK
			cout << "Update no match: " << endl;
			cout << tr->getScreenPosition(fovSize_, imageSize_) << endl;
#endif
			tr->clearDetected();
		}

		++tr;
		++as;
	}

	// Remove tracks which haven't been seen in a while
	for (auto it = list_.begin(); it != list_.end(); )
	{
		if (it->tooManyMissedFrames()) // For now just remove ones for
		{                              // which detectList is empty
#ifdef VERBOSE_TRACK
			cout << "Dropping " << it->getId() << endl;
#endif
			it = list_.erase(it);
		}
		else
		{
			++it;
		}
	}
#ifdef VERBOSE_TRACK
	print();
	if (!detectedRects.empty() || !list_.empty())
		cout << "---------- End of process detect --------------" << endl;
#endif
}
