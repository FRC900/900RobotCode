#ifndef PARTICLE_FILTER_HEADER
#define PARTICLE_FILTER_HEADER

#include <memory>
#include <random>
#include "particle.hpp"
#include "world_model.hpp"
#include <geometry_msgs/PoseWithCovariance.h>

// Run checks that each particle hasn't ended up with NaN or Inf
// values - useful for debugging but a lot of overhead for normal use
// Currently there shouldn't be any checks in the code, but this macro
// can be used to add them if needed
#define CHECK_PARTICLES(o) o->check_particles(__FILE__, __LINE__);
//#define CHECK_PARTICLES(o)
class ParticleFilter {
private:
  const size_t num_particles_;
  std::mt19937 rng_;
  std::normal_distribution<double> pos_dist_;
  std::normal_distribution<double> rot_dist_;
  std::vector<Particle> particles_;
  WorldModel world_;
  void normalize();
  void init(const double x_min, const double x_max, const double y_min, const double y_max);
  void constrain_particles();

public:
  ParticleFilter(const WorldModel& w,
                 double x_min, double x_max, double y_min, double y_max,
                 double ns, double rs, size_t n);
  geometry_msgs::PoseWithCovariance predict();
  void noise_rot();
  void noise_pos();
  bool motion_update(double delta_x, double delta_y, double delta_rot);
  bool set_rotation(double rot);
  bool assign_weights(const std::vector<std::shared_ptr<BeaconBase>> &measurements, const std::vector<double> &sigmas);
  void resample();
  std::vector<Particle> get_particles() const;
  void check_particles(const char *file, int line) const;
  // need access to world functions so this allows that
  bool allianceColorCheck(bool amIBlueAlliance) {
    return world_.allianceColorCheck(amIBlueAlliance);
  }
  void reinit();
};


#endif
