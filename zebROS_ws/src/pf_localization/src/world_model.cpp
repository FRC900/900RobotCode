#include "world_model.hpp"
#include <cmath>
#include <random>
#include <utility>
#include <algorithm>

#include <iostream>
#include <string>

#include <ros/ros.h>


WorldModel::WorldModel(const std::vector<std::pair<double, double> >& beacons,
                       double x_min, double x_max, double y_min, double y_max) :
  beacons_(beacons), boundaries_ {x_min, x_max, y_min, y_max} {}

std::vector<double> WorldModel::get_boundaries() {
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
std::vector<double> WorldModel::distances(const std::pair<double, double>& m,
                                          const std::vector<std::pair<double, double> >& rel) const {
  std::vector<double> res;
  res.reserve(beacons_.size());
  for (const std::pair<double, double>& b : rel) {
    res.push_back(hypot(m.first - b.first, m.second - b.second));
  }
  return res;
}

//gets the coordinates of all the beacons relative to a give particle
std::vector<std::pair<double, double> > WorldModel::particle_relative(const Particle& p) const {
  std::vector<std::pair<double, double> > res;
  for (const std::pair<double, double>& b : beacons_) {
    double x = b.first - p.x_;
    double y = b.second - p.y_;
    double r = hypot(x, y);
    double theta = atan2(y, x);
    theta -= p.rot_;
    x = r * cos(theta);
    y = r * sin(theta);

    res.push_back(std::make_pair(x, y));
  }
  return res;
}

//Uses hungarian algorithm to pair particle relative beacons and robot relative beacons and returns the total error (sum of distance errors from particle to robot beacons)
double WorldModel::total_distance(const Particle& p, const std::vector<std::pair<double, double> >& m) {
  std::vector<int> assignment;
  assignment.reserve(m.size());
  std::vector<std::vector<double> > dists;
  dists.reserve(m.size());
  std::vector<std::pair<double, double> > rel = particle_relative(p);
  for (const std::pair<double, double>& b : m) {
    dists.push_back(distances(b, rel));
  }
  solver_.Solve(dists, assignment);
  double res = 0;
  for (size_t i = 0; i < assignment.size(); i++) {
    if (assignment[i] < 0) continue;
    res += dists[i][assignment[i]];
  }
  return res;
}
