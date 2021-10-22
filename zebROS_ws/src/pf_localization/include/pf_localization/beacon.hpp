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

struct PositionBeacon;
struct BeaconBase {
	const std::string type_;
	BeaconBase(const std::string &type) : type_(type) {}
	// Get the distance from this beacon (a detected field object) to a list of
	// known beacon locations (the rel vector).  Returns a vector with a distance
	// measurements, each one corresponding to the distance (or cost) to the
	// beacon position in the rel vector
	virtual std::vector<double> distances(const std::vector<PositionBeacon>& rel) const = 0;
	virtual double debugSum(void) const = 0;
};

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
  double debugSum(void) const override { return x_ + y_; }
};

std::ostream& operator<<(std::ostream &os, const PositionBeacon& b);

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
  double debugSum(void) const override { return angle_; }
};
std::ostream& operator<<(std::ostream &os, const BearingBeacon& b);

#endif
