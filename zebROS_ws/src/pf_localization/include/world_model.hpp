#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

#include "particle.hpp"
#include "hungarian.hpp"
#include "beacon.hpp"

class WorldModel {
private:
  std::vector<Beacon> beacons_;
  double boundaries_[4]; // xmin, xmax, ymin, ymax
  AssignmentProblemSolver solver_;
  std::vector<double> distances(const Beacon& m,
                                const std::vector<Beacon>& rel) const;
  std::vector<double> angle_distances(const BearingBeacon& m,
                                      const std::vector<Beacon>& rel) const;
  std::vector<Beacon> of_type(const std::vector<Beacon>& bcns, std::string type);
  std::vector<Beacon> single_particle_relative(const Particle& p, const std::vector<Beacon> bcns) const;
public:
  std::vector<Beacon> particle_relative(const Particle& p, const Particle& offset) const;
  WorldModel(const std::vector<Beacon>& beacons,
             double x_min, double x_max, double y_min, double y_max);
  std::vector<double> get_boundaries();
  bool is_in_world(const Particle& p) const;
  void constrain_to_world(Particle& p) const;
  void init_particle(Particle& p, double x_min, double x_max, double y_min, double y_max);
  double total_distance(const Particle& p, const std::vector<Beacon>& m, const Particle& offset);
  double total_angle(const Particle& p, const std::vector<BearingBeacon>& m, const Particle& offset);
};

#endif
