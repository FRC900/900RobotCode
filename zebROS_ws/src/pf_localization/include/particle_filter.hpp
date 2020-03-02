#ifndef PARTICLE_FILTER_HEADER
#define PARTICLE_FILTER_HEADER

#include <utility>
#include <random>
#include "world_model.hpp"
#include "particle.hpp"

class ParticleFilter {
private:
  size_t num_particles_;
  double noise_stdev_;
  double rot_noise_stdev_;
  std::mt19937 rng_;
  std::vector<Particle> particles_;
  WorldModel world_;
  void normalize();
  void init(const double x_min, const double x_max, const double y_min, const double y_max);
  void constrain_particles();

public:
  ParticleFilter(const WorldModel& w,
                 double x_min, double x_max, double y_min, double y_max,
                 double ns, double rs, size_t n);
  Particle predict();
  void noise_rot();
  void noise_pos();
  void motion_update(double delta_x, double delta_y, double delta_rot);
  void set_rotation(double rot);
  void assign_weights(std::vector<std::pair<double, double> > mBeacons);
  void resample();
  std::vector<Particle> get_particles() const;
};


#endif
