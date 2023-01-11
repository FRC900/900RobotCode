#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

#include <memory>
#include <set>

#include "beacon.hpp"
#include "hungarian.hpp"
#include "particle.hpp"

struct WorldModelBoundaries
{
  WorldModelBoundaries(const double x_min, const double x_max, const double y_min, const double y_max)
    : x_min_(x_min)
    , x_max_(x_max)
    , y_min_(y_min)
    , y_max_(y_max)
  {
  
}
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
};

class WorldModel {
private:
  std::vector<PositionBeacon> beacons_;
  const std::vector<PositionBeacon> blue_beacons_;
  const std::vector<PositionBeacon> red_beacons_;
  bool amIBlueAlliance_{true};
  WorldModelBoundaries boundaries_;
  AssignmentProblemSolver solver_;
  std::vector<PositionBeacon> of_type(const std::vector<PositionBeacon>& bcns, const std::string &type);
  mutable std::set<Particle> beacons_seen_; // Coords of beacons seen since last call to clear_beacons(), used for debug visualization
  std::set<std::string> beacon_names_; // list of valid beacon names
  double camera_fov_;

public:
  //std::vector<PositionBeacon> particle_relative(const Particle& p) const;
  WorldModel(std::vector<PositionBeacon>& beacons, const WorldModelBoundaries &boundaries, const double camera_fov);
  const WorldModelBoundaries &get_boundaries() const;
  //bool is_in_world(const Particle& p) const;
  void constrain_to_world(Particle& p) const;
  void init_particle(Particle& p, const WorldModelBoundaries &boundaries);
  bool allianceColorCheck(bool amIBlueAlliance);
  std::vector<PositionBeacon> getRedBeacons(const std::vector<PositionBeacon> &blueBeacons);
  std::optional<PositionBeacon> particle_relative_beacon(const Particle &p, const PositionBeacon &b) const;
  double total_distance(const Particle &p, const std::vector<std::shared_ptr<BeaconBase>> &measurements, const std::vector<double> &sigmas) const;
  const std::set<Particle> &get_beacons_seen() const;
  void clear_beacons_seen();
  bool is_valid_beacon(const std::string &beacon_name) const;
  void set_camera_fov(const double camera_fov);
  };
#endif