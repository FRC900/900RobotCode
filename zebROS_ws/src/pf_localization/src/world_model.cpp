#define _USE_MATH_DEFINES

#include "world_model.hpp"
#include <cmath>
#include <utility>
#include <algorithm>
#include <map>
#include <iostream>
#include <string>


WorldModel::WorldModel(const std::vector<Beacon>& beacons,
                       double x_min, double x_max, double y_min, double y_max) :
  beacons_(beacons), boundaries_ {x_min, x_max, y_min, y_max} {}

std::vector<double> WorldModel::get_boundaries() { // TODO - make boundaries a std::array?
  std::vector<double> res;
  for (size_t i = 0; i < 4; i++) {
    res.push_back(boundaries_[i]);
  }
  return res;
}

//checks if a given particle is within the defined boundaries
bool WorldModel::is_in_world(const Particle& p) const {
  if(p.x_ < boundaries_[0] || p.x_ > boundaries_[1]){
    return false;
  }
  if(p.y_ < boundaries_[2] || p.y_ > boundaries_[3]){
    return false;
  }
  return true;
}

//moves a given particle to the nearest position that is within the defined boundaries
void WorldModel::constrain_to_world(Particle& p) const {
  p.x_ = std::min(boundaries_[1], std::max(boundaries_[0], p.x_));
  p.y_ = std::min(boundaries_[3], std::max(boundaries_[2], p.y_));
}

//computes distances from a position a set of beacon positions
std::vector<double> WorldModel::distances(const Beacon& m,
                                          const std::vector<Beacon>& rel) const {
  std::vector<double> res;
  for (const Beacon& b : rel) {
    res.push_back(hypot(m.x_ - b.x_, m.y_ - b.y_));
  }
  return res;
}

std::vector<double> WorldModel::angle_distances(const BearingBeacon& m,
                                                const std::vector<Beacon>& rel) const {
  std::vector<double> res;
  for (const Beacon& b : rel) {
    BearingBeacon ba {atan2(b.y_, b.x_), b.type_};
    double diff = abs(fmod(m.angle_, (2 * M_PI)) - fmod(ba.angle_, (2 * M_PI)));
    if (diff > M_PI) diff -= 2 * M_PI;
    res.push_back(diff);
  }
  return res;
}

//gets the coordinates of all the beacons relative to a given particle
std::vector<Beacon> WorldModel::single_particle_relative(const Particle& p, const std::vector<Beacon> bcns) const {
  std::vector<Beacon> res;
  for (const Beacon& b : bcns) {
    double x = b.x_ - p.x_;
    double y = b.y_ - p.y_;
    double r = hypot(x, y);
    double theta = atan2(y, x);
    theta -= p.rot_;
    x = r * cos(theta);
    y = r * sin(theta);
    res.emplace_back(Beacon{x, y, b.type_});
  }
  return res;
}

std::vector<Beacon> WorldModel::particle_relative(const Particle& p, const Particle& offset) const {
  return single_particle_relative(offset, single_particle_relative(p, beacons_));
}

std::vector<Beacon> WorldModel::of_type(const std::vector<Beacon>& bcns, const std::string &type) {
  std::vector<Beacon> res;
  for (const Beacon& b : bcns) {
    if (b.type_ == type) {
      res.push_back(b);
    }
  }
  return res;
}

//Uses hungarian algorithm to pair particle relative beacons and robot relative beacons and returns the total error (sum of distance errors from particle to robot beacons)
// TODO - might be simpler to just sort beacons by type, then loop through the array in order
//        grabbing all of one type. Once we see the next type, calc the dists for the current
//        type then repeat until all entries are processed
double WorldModel::total_distance(const Particle& p, const std::vector<Beacon>& m, const Particle& offset) {
  double total_res = 0;
  std::map<std::string, std::vector<Beacon> > by_type;
  for (const Beacon& b : m) {
    if (by_type.count(b.type_) == 0) {
      by_type[b.type_] = std::vector<Beacon>();
    }
    by_type[b.type_].push_back(b);
  }
  for (const std::pair<std::string, std::vector<Beacon> >& m_of_type : by_type) {
    std::vector<int> assignment;
    std::vector<std::vector<double> > dists;
    const std::vector<Beacon> rel{of_type(particle_relative(p, offset), m_of_type.first)};
    for (const Beacon& b : m_of_type.second) {
      dists.push_back(distances(b, rel));
    }
	if (dists.size() == 0)
		continue;
    solver_.Solve(dists, assignment);
    double res = 0;
    for (size_t i = 0; i < assignment.size(); i++) {
      if (assignment[i] < 0) continue;
      res += dists[i][assignment[i]];
    }
    total_res += res;
  }

  return total_res;
}

// like total_distance, but bearing only
double WorldModel::total_angle(const Particle& p, const std::vector<BearingBeacon>& m, const Particle& offset) {
  double total_res = 0;
  std::map<std::string, std::vector<BearingBeacon> > by_type;
  for (const BearingBeacon& b : m) {
    if (by_type.count(b.type_) == 0) {
      by_type[b.type_] = std::vector<BearingBeacon>();
    }
    by_type[b.type_].push_back(b);
  }
  for (const std::pair<std::string, std::vector<BearingBeacon> >& m_of_type : by_type) {
    std::vector<int> assignment;
    std::vector<std::vector<double> > dists;
    const std::vector<Beacon> rel = of_type(particle_relative(p, offset), m_of_type.first);
    for (const BearingBeacon& b : m_of_type.second) {
      dists.push_back(angle_distances(b, rel));
    }
	if (dists.size() == 0)
		continue;
    solver_.Solve(dists, assignment);
    double res = 0;
    for (size_t i = 0; i < assignment.size(); i++) {
      if (assignment[i] < 0) continue;
      res += dists[i][assignment[i]];
    }
    total_res += res;
  }

  return total_res;
}
