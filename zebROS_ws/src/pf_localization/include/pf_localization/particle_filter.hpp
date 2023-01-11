#ifndef PARTICLE_FILTER_HEADER
#define PARTICLE_FILTER_HEADER

#include <set>
#include <memory>
#include <optional>
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
  size_t resetCounter_{0};
  double rotation_threshold_;
  std::vector<Particle> particles_;
  WorldModel world_;
  std::normal_distribution<double> rot_thresh_dist_;
  void normalize();
  void init(const WorldModelBoundaries &boundaries);
  void constrain_particles();

public:
  ParticleFilter(const WorldModel& w,
                 const WorldModelBoundaries &boundaries,
                 double ns, double rs, double rt, size_t n);
  std::optional<geometry_msgs::PoseWithCovariance> predict();
  void noise_rot();
  void noise_pos();
  bool motion_update(double delta_x, double delta_y, double delta_rot);
  bool set_rotation(double rot);
  bool assign_weights(const std::vector<std::shared_ptr<BeaconBase>> &measurements, const std::vector<double> &sigmas);
  void resample();
  const std::vector<Particle> &get_particles() const;
  const std::set<Particle> &get_beacons_seen() const;
  void clear_beacons_seen();
  void check_particles(const char *file, int line) const;
  // need access to world functions so this allows that
  bool allianceColorCheck(bool amIBlueAlliance) {
    return world_.allianceColorCheck(amIBlueAlliance);
  }
  void reinit();
  bool is_valid_beacon(const std::string &beacon_name) const;
};

#endif