#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

#include <memory>

#include "beacon.hpp"
#include "hungarian.hpp"
#include "particle.hpp"

class WorldModel {
private:
  std::vector<PositionBeacon> beacons_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  AssignmentProblemSolver solver_;
  std::vector<PositionBeacon> of_type(const std::vector<PositionBeacon>& bcns, const std::string &type);
  std::vector<PositionBeacon> single_particle_relative(const Particle& p, const std::vector<PositionBeacon> &bcns, const std::string &type) const;
public:
  std::vector<PositionBeacon> particle_relative(const Particle& p) const;
  WorldModel(const std::vector<PositionBeacon>& beacons,
             double x_min, double x_max, double y_min, double y_max);
  void get_boundaries(double &x_min, double &x_max, double &y_min, double &y_max) const;
  //bool is_in_world(const Particle& p) const;
  void constrain_to_world(Particle& p) const;
  void init_particle(Particle& p, double x_min, double x_max, double y_min, double y_max);
  double total_distance(const Particle& p, const std::vector<std::shared_ptr<BeaconBase>>& m);
};

#endif
