#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <string>
#include <utility>

#include "pf_localization/world_model.hpp"

WorldModel::WorldModel(std::vector<PositionBeacon>& beacons, const std::vector<PositionBeacon>& red_beacons,
                       double x_min, double x_max, double y_min, double y_max) :
  beacons_(beacons), blue_beacons_(beacons), red_beacons_(red_beacons), x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max) {}

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

// Returns true if it switched the beacon color
bool WorldModel::allianceColorCheck(bool amIBlueAlliance) {
  // match data says we have changed colors
  if (amIBlueAlliance != amIBlueAlliance_) {
    amIBlueAlliance_ = amIBlueAlliance;
    if (amIBlueAlliance_) {
      beacons_.clear();
      // itterate over the blue beacons and add them to the beacons_ vector
      // probably a better way to do this, but beacons_ = blue_beacons_ doesn't work
      // shouldn't be running this often so it probably won't slow anything down
      for (auto it = blue_beacons_.begin(); it != blue_beacons_.end(); ++it) {
        beacons_.push_back(*it);
      }
    } 
    else {
      beacons_.clear();
      // copy red beacons in to beacons
      for (auto it = red_beacons_.begin(); it != red_beacons_.end(); ++it) {
        beacons_.push_back(*it);
      }
    }
    return true;
  }
  return false;
}

//moves a given particle to the nearest position that is within the defined boundaries
void WorldModel::constrain_to_world(Particle& p) const {
  p.x_ = std::min(x_max_, std::max(x_min_, p.x_));
  p.y_ = std::min(y_max_, std::max(y_min_, p.y_));
}

static PositionBeacon particle_relative_beacon(const Particle &p, const PositionBeacon &b) {
    double x = b.x_ - p.x_;
    double y = b.y_ - p.y_;
    double r = hypot(x, y);
    double theta = atan2(y, x);
    theta -= p.rot_;
    x = r * cos(theta);
    y = r * sin(theta);
    return PositionBeacon{x, y, b.type_};
}

//gets the coordinates of all the field beacons of a given type relative to a given particle
//returns - a vector, one entry per beacon of matching type. each entry is the offset (x,y)
//from the input particle position p and the corresponding beacon position
static std::vector<PositionBeacon> single_particle_relative(const Particle& p, const std::vector<PositionBeacon> &bcns, const std::string &type) {
  std::vector<PositionBeacon> res;
  for (const PositionBeacon& b : bcns) {
    if (b.type_ != type) {
      continue;
    }
    res.emplace_back(particle_relative_beacon(p, b));
  }
  return res;
}

// A struct holding data about particles and their position relative to map beacons
// A map of this struct is filled in in total_distance, where each entry
// holds data about one type of beacon.
struct beacons {
  // List of positions of each beacon of a given type relative to a given particle
  std::vector<PositionBeacon> rel;
  // 2-d array of distances between measurements and beacons
  // each column is the distance between a given measurement and each beacon of
  //   that measurement's type
  // there's a row added for each measurement of that type
  std::vector<std::vector<double>> dists;
  // The measurement associated with each row of dists, used later to compute
  // the particle weight, based on the difference between the measurement and
  // expected beacon position).
  std::vector<std::shared_ptr<BeaconBase>> meas;
};
std::ostream& operator<<(std::ostream &os, const beacons& b)
{
  os << "rel" << std::endl;
  for (const auto &r : b.rel) {
    os << "\t" << r;
  }
  os << "dists" << std::endl;
  for (const auto &d : b.dists) {
    os << "\t";
    for (const auto &v : d) {
      os << " " << v;
    }
    os << std::endl;
  }
  os << "meas" << std::endl;
  for (const auto &m : b.meas) {
    os << "\t" << *std::dynamic_pointer_cast<PositionBeacon>(m) << std::endl;
  }
  return os;
}

//Uses hungarian algorithm to pair particle relative beacons and robot relative beacons and returns the
//weight for that particular particle.  Weights are calculated based on how likely the measurement is given
//the std.deviation for the camera detection, passed in the sigmas array.
double WorldModel::total_distance(const Particle& p,
                                  const std::vector<std::shared_ptr<BeaconBase>>& measurements,
                                  const std::vector<double> &sigmas) const {
  std::unordered_map<std::string, beacons> beacons_by_type;

  // Put relative distances of field beacons into beacons_by_type
  for (const auto &b : beacons_) {
    beacons_by_type[b.type_].rel.push_back(particle_relative_beacon(p, b));
  }

  // For each detection of this type, create a list of distances from
  // that detection to each field beacon of that same type
  for (const auto &m : measurements) {
    const auto it = beacons_by_type.find(m->type_);
    if (it != beacons_by_type.cend()) {
        it->second.dists.push_back(m->distances(it->second.rel));
        it->second.meas.push_back(m);
    }
  }

  // Loop over each detection type. For that, do an optimal assigment
  // of measurements with beacons of the same type as the detection
  double total_res = 1.;
  for (auto &pair: beacons_by_type) {
    if (pair.second.dists.empty()) {
      continue;
    }
    // Match up measurements with the most likely mapping to a beacon.
    std::vector<int> assignment;
    solver_.Solve(pair.second.dists, assignment);
    // For each match from measurements to beacon, update the weight
    // based on how likely this measurement would be if the particle
    // were in the correct location
    // i is the i-th measurement with the current type, assigments[i] is the corresponding
    // camera-relative beacon position
    for (size_t i = 0; i < assignment.size(); i++) {
      if (assignment[i] >= 0) {
        const double w = pair.second.meas[i]->weight(pair.second.rel[assignment[i]], sigmas);
        total_res *= w;
      }
    }
  }

  // TODO: perhaps a lower limit on total_res
  // TODO: is it possible for no weights to be assigned and the result be 1?
  return total_res;
}

