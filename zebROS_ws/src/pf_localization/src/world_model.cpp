#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>

#include "pf_localization/world_model.hpp"

WorldModel::WorldModel(const std::vector<PositionBeacon>& beacons,
                       double x_min, double x_max, double y_min, double y_max) :
  beacons_(beacons), x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max) {}

void WorldModel::get_boundaries(double &x_min, double &x_max, double &y_min, double &y_max) const {
	x_min = x_min_;
	x_max = x_max_;
	y_min = y_min_;
	y_max = y_max_;
}

#if 0
//checks if a given particle is within the defined boundaries
bool WorldModel::is_in_world(const Particle& p) const {
  if(p.x_ < x_min_ || p.x_ > x_max_){
    return false;
  }
  if(p.y_ < y_min_ || p.y_ > y_max_){
    return false;
  }
  return true;
}
#endif

//moves a given particle to the nearest position that is within the defined boundaries
void WorldModel::constrain_to_world(Particle& p) const {
  p.x_ = std::min(x_max_, std::max(x_min_, p.x_));
  p.y_ = std::min(y_max_, std::max(y_min_, p.y_));
}

//gets the coordinates of all the field beacons of a given type relative to a given particle
//returns - a vector, one entry per beacon of matching type. each entry is the offset (x,y)
//from the input particle position p and the corresponding beacon position
std::vector<PositionBeacon> WorldModel::single_particle_relative(const Particle& p, const std::vector<PositionBeacon> &bcns, const std::string &type) const {
  std::vector<PositionBeacon> res;
  for (const PositionBeacon& b : bcns) {
    if (b.type_ != type)
      continue;
    double x = b.x_ - p.x_;
    double y = b.y_ - p.y_;
    double r = hypot(x, y);
    double theta = atan2(y, x);
    theta -= p.rot_;
    x = r * cos(theta);
    y = r * sin(theta);
    res.emplace_back(PositionBeacon{x, y, b.type_});
  }
  return res;
}

//Uses hungarian algorithm to pair particle relative beacons and robot relative beacons and returns the total error
//(sum of distance errors from particle to robot beacons)
double WorldModel::total_distance(const Particle& p, const std::vector<std::shared_ptr<BeaconBase>>& m) const {
  // Create mapping of <type of detection, vector of detections of that type>
  // Use this to handle mapping from a set of detections of a given type to the
  // most likely set of beacons they line up with
  // TODO - might be simpler to just sort beacons by type, then loop through the array in order
  //        grabbing all of one type. Once we see the next type, calc the dists for the current
  //        type then repeat until all entries are processed
  std::map<std::string, std::vector<std::shared_ptr<BeaconBase>> > by_type;
  for (const auto & b : m) {
	// Create empty vector per type if not there already, then append
	// detection to that vector
	by_type.emplace(b->type_, std::vector<std::shared_ptr<BeaconBase>>());
    by_type[b->type_].push_back(b);
  }
  // Loop over each detection type. For that, do an optimal assigment
  // of detections with beacons of the same type as the detection
  double total_res = 0;
  for (const auto & m_of_type : by_type) { // m_of_type = detections of a given type
    std::vector<std::vector<double> > dists;
    // expected distance between the particle and the field beacons
    // Only match up beacons with the same type as the current detection type
    const std::vector<PositionBeacon> rel{single_particle_relative(p, beacons_, m_of_type.first)};
    // For each detection of this type, create a list of distances from
    // that detection to each field beacon of that same type
    for (const auto& b : m_of_type.second) { // b is each detection of this type
      dists.push_back(b->distances(rel));
    }
    if (dists.size() == 0)
      continue;
	// Match up detections with the most likely mapping to a beacon.
    std::vector<int> assignment;
    solver_.Solve(dists, assignment);
    // For each match from detection to beacon, add the
    // distance the detection is off from ideal to the
    // total cumulative distance for this particular particle
    for (size_t i = 0; i < assignment.size(); i++) {
      if (assignment[i] >= 0)
        total_res += dists[i][assignment[i]];
    }
  }

  return total_res;
}

