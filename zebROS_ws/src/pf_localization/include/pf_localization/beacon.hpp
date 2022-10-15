// Used to store data about beacons. These can be beacons defined by
// the field map, or beacons detected from camera object detection
// Base class is pure virtual, have a separate derived class for
// beacons which have an x,y value (these are either field map
// definitions of beacons or detections from the RGB+D camera)
// and one for beacons which only have a bearing (detections
// from a non-depth camera, which give only a direction to
// the target rather than an x and y distance)
#ifndef BEACON_HEADER
#define BEACON_HEADER
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

//#include <ros/console.h>

struct PositionBeacon;
struct BeaconBase {
	const std::string type_;
	BeaconBase(const std::string &type) : type_(type) {}
	// Get the distance from this beacon (a detected field object) to a list of
	// measured beacon locations (the rel vector, for camera-relative measurements).  Returns
	// a vector with a distance value, each one corresponding to the distance from
	// this map beacon to a measured beacon position
	virtual std::vector<double> distances(const std::vector<PositionBeacon>& rel) const = 0;
	// Calculate a measurement's contribution to total particle weight. The weight is
	// the probability we'd get this measurement given the mapping from measurements to map
	// beacons. This function calculates the weight of one beacon->measurement mapping.
	// The object is the camera-relative measurement, and the b arg is camera-relative map beacon location
	// Sigmas holds the std dev expected from the measurement
	virtual double weight(const PositionBeacon &b, const std::vector<double> &sigmas) const = 0;
	virtual double debugSum(void) const = 0;
};

struct PositionBeacon;
std::ostream& operator<<(std::ostream &os, const PositionBeacon& b);
struct PositionBeacon : BeaconBase {
  const double x_;
  const double y_;
  PositionBeacon(double x, double y, const std::string& type) : BeaconBase(type), x_(x), y_(y) {}
  std::vector<double> distances(const std::vector<PositionBeacon>& rel) const override
  {
	  std::vector<double> res;
	  for (const PositionBeacon& b : rel) {
		  res.push_back(hypot(x_ - b.x_, y_ - b.y_));
	  }
	  return res;
  }
  double weight(const PositionBeacon &b, const std::vector<double> &sigmas) const override
  {
	  const double dx = b.x_ - x_;
	  const double dy = b.y_ - y_;
#if 0
	  ROS_INFO_STREAM("weight");
	  ROS_INFO_STREAM("this = " << *this);
	  ROS_INFO_STREAM("b = " << b);
	  ROS_INFO_STREAM("dx = " << dx << " dy = " << dy);
	  ROS_INFO_STREAM("sigmas = " << sigmas[0] << " " << sigmas[1]);
#endif
	  return exp(-(dx * dx) / (sigmas[0] * sigmas[0])) *
	         exp(-(dy * dy) / (sigmas[1] * sigmas[1]));

  }
  double debugSum(void) const override { return x_ + y_; }
};


struct BearingBeacon : BeaconBase {
  const double angle_;
  BearingBeacon(double x, double y, const std::string& type) : BeaconBase(type), angle_(atan2(y, x)) {}
  std::vector<double> distances(const std::vector<PositionBeacon>& rel) const override
  {
	  std::vector<double> res;
	  for (const PositionBeacon& b : rel) {
		  BearingBeacon ba {b.x_, b.y_, b.type_};
		  double diff = std::abs(fmod(angle_, (2 * M_PI)) - fmod(ba.angle_, (2 * M_PI)));
		  if (diff > M_PI) diff -= 2 * M_PI;
		  res.push_back(diff);
	  }
	  return res;
  }
  double weight(const PositionBeacon &b, const std::vector<double> &sigmas) const override
  {
	  BearingBeacon ba {b.x_, b.y_, b.type_};
	  double diff = std::abs(fmod(angle_, (2 * M_PI)) - fmod(ba.angle_, (2 * M_PI)));
	  if (diff > M_PI) diff -= 2 * M_PI;
	  return exp(-(diff * diff) / (sigmas[0] * sigmas[0]));

  }
  double debugSum(void) const override { return angle_; }
};
std::ostream& operator<<(std::ostream &os, const BearingBeacon& b);

#endif
