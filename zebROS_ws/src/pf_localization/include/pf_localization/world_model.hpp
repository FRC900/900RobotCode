#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

#include <memory>

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
  bool amIBlueAlliance_ = true;
  WorldModelBoundaries boundaries_;
  AssignmentProblemSolver solver_;
  std::vector<PositionBeacon> of_type(const std::vector<PositionBeacon>& bcns, const std::string &type);

public:
  std::vector<PositionBeacon> particle_relative(const Particle& p) const;
  WorldModel(std::vector<PositionBeacon>& beacons, const WorldModelBoundaries &boundaries);
  const WorldModelBoundaries &get_boundaries() const;
  //bool is_in_world(const Particle& p) const;
  void constrain_to_world(Particle& p) const;
  void init_particle(Particle& p, const WorldModelBoundaries &boundaries);
  bool allianceColorCheck(bool amIBlueAlliance);
  std::vector<PositionBeacon> getRedBeacons(const std::vector<PositionBeacon> &blueBeacons);
  double total_distance(const Particle& p, const std::vector<std::shared_ptr<BeaconBase>>& measurements, const std::vector<double> &sigmas) const;
};

#endif
