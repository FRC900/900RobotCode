#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

#include "particle.hpp"
#include "hungarian.hpp"

class WorldModel {
private:
  std::vector<std::pair<double, double> > beacons_;
  double boundaries_[4]; // xmin, xmax, ymin, ymax
  AssignmentProblemSolver solver_;
  std::vector<double> distances(const std::pair<double, double>& m,
                                const std::vector<std::pair<double, double> >& rel) const;

public:
  std::vector<std::pair<double, double> > particle_relative(const Particle& p) const;

public:
  WorldModel(const std::vector<std::pair<double, double> >& beacons,
             double x_min, double x_max, double y_min, double y_max);
  std::vector<double> get_boundaries();
  bool is_in_world(const Particle& p) const;
  void constrain_to_world(Particle& p) const;
  void init_particle(Particle& p, double x_min, double x_max, double y_min, double y_max);
  double total_distance(const Particle& p, const std::vector<std::pair<double, double> >& m);
};

#endif
