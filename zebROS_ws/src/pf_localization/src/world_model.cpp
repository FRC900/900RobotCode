#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <string>
#include <utility>
#include "pf_localization/world_model.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "pf_localization/particle_filter.hpp"
// Given a list of beacons relative to the blue corner of the field,
// generate the same list of beacons but translated so the origin is
// the red corner of the field.
std::vector<PositionBeacon> WorldModel::getRedBeacons(const std::vector<PositionBeacon> &blueBeacons)
{
	constexpr double field_width = 16.458;
	constexpr double field_height = 8.228; 
	ros::Time now = ros::Time::now();
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.frame_id = "blue0";
	transformStamped.header.stamp = now;
	transformStamped.child_frame_id = "red0";
	transformStamped.transform.translation.x = field_width;
	transformStamped.transform.translation.y = field_height;
	tf2::Quaternion q;
	q.setRPY(0, 0, M_PI);
	transformStamped.transform.rotation = tf2::toMsg(q);

	std::vector<PositionBeacon> redBeacons;
	for (const auto & blueBeacon : blueBeacons)
	{
		geometry_msgs::PoseStamped b_red;
		b_red.header.frame_id = "blue0";
		b_red.header.stamp = now;
		b_red.pose.position.x = blueBeacon.x_;
		b_red.pose.position.y = blueBeacon.y_;
		tf2::doTransform(b_red, b_red, transformStamped);
		PositionBeacon b_r{b_red.pose.position.x, b_red.pose.position.y, blueBeacon.type_};
		redBeacons.push_back(b_r);
	}
	return redBeacons;
}

WorldModel::WorldModel(std::vector<PositionBeacon>& beacons, const WorldModelBoundaries &boundaries, const double camera_fov) :
  beacons_(beacons), blue_beacons_(beacons), red_beacons_(getRedBeacons(beacons)), boundaries_(boundaries), camera_fov_(camera_fov) {
    for (const auto &b: beacons) {
      beacon_names_.insert(b.type_);
    }
  }

const WorldModelBoundaries& WorldModel::get_boundaries() const {
  return boundaries_;
}

#if 0
//checks if a given particle is within the defined boundaries
bool WorldModel::is_in_world(const Particle& p) const {
  if(p.x_ < boundaries_.x_min_ || p.x_ > boundaries_.x_max_){
    return false;
  }
  if(p.y_ < boundaries_.y_min_ || p.y_ > boundaries_.y_max_){
    return false;
  }
  return true;
}
#endif


// Returns true if it switched the beacon color
bool WorldModel::allianceColorCheck(bool amIBlueAlliance) {
  // match data says we have changed colors
  if (amIBlueAlliance != amIBlueAlliance_) {
    amIBlueAlliance_ = amIBlueAlliance;
    if (amIBlueAlliance_) {
      beacons_.clear();
      // itterate over the blue beacons and add them to the beacons_ vector
      // probably a better way to do this, but beacons_ = blue_beacons_ doesn't work
      // shouldn't be running this often so it probably won't slow anything down
      for (auto it = blue_beacons_.begin(); it != blue_beacons_.end(); ++it) {
        beacons_.push_back(*it);
      }
    } 
    else {
      beacons_.clear();
      // copy red beacons in to beacons
      for (auto it = red_beacons_.begin(); it != red_beacons_.end(); ++it) {
        beacons_.push_back(*it);
      }
    }
    return true;
  }
  return false;
}

//moves a given particle to the nearest position that is within the defined boundaries
void WorldModel::constrain_to_world(Particle& p) const {
  p.x_ = std::min(boundaries_.x_max_, std::max(boundaries_.x_min_, p.x_));
  p.y_ = std::min(boundaries_.y_max_, std::max(boundaries_.y_min_, p.y_));
}

std::optional<PositionBeacon> WorldModel::particle_relative_beacon(const Particle &p, const PositionBeacon &b) const {
  double x = b.x_ - p.x_;
  double y = b.y_ - p.y_;
  double r = hypot(x, y);
  double theta = atan2(y, x);
  theta = angles::normalize_angle(theta - p.rot_);
  // Filter out beacons which aren't visible given the particle's orientation
  // and the camera's FoV
  // TODO - what if the center of the fov isn't 0?
  if (std::abs(theta) > (camera_fov_ / 2.))
  {
    return std::nullopt;
  }
  x = r * cos(theta);
  y = r * sin(theta);
  return PositionBeacon{x, y, b.type_};
}

double WorldModel::total_distance(const Particle& p,
                                  const std::vector<std::shared_ptr<BeaconBase>>& measurements,
                                  const std::vector<double> &sigmas) const {
  // List of positions of each beacon of a given type relative to this particle
  std::vector<std::optional<PositionBeacon>> rel;
  for (const auto &b : beacons_) {
    rel.push_back(particle_relative_beacon(p, b));
  }
  // 2-d array of distances between measurements and beacons
  // each column is the distance between a given measurement and each beacon
  // there's a row added for each measurement
  std::vector<std::vector<double>> dists;
  for (const auto &m : measurements) {
    dists.push_back(m->distances(rel));
  }

  // Match up measurements with the most likely mapping to a beacon.
  std::vector<int> assignment;
  solver_.Solve(dists, assignment, AssignmentProblemSolver::many_forbidden_assignments);

  double total_res = 1.;
  size_t num_matches = 0;
  // For each match from measurements to beacon, update the weight
  // based on how likely this measurement would be if the particle
  // were in the correct location
  // i is the i-th measurement, assigments[i] is the corresponding beacon index
  for (size_t i = 0; i < assignment.size(); i++) {
    if (assignment[i] >= 0) {
      const double w = measurements[i]->weight(*rel[assignment[i]], sigmas);
      total_res *= w;
      num_matches += 1;
      const auto &b = beacons_[assignment[i]];
      beacons_seen_.insert({b.x_, b.y_, 0});
    }
  }
  // If any measurements are filtered out, return a low weight since this
  // particle isn't in a position which can even see the targets
  // given the particle's orientation
  if (num_matches < measurements.size()) {
    return 0.01;
  }

  // TODO: perhaps a lower limit on total_res
  return pow(total_res, 1.0 / num_matches);
}

const std::set<Particle> &WorldModel::get_beacons_seen(void) const {
  return beacons_seen_;
}

void WorldModel::clear_beacons_seen(void) {
  beacons_seen_.clear();
}

bool WorldModel::is_valid_beacon(const std::string &beacon_name) const {
  return beacon_names_.count(beacon_name) > 0;
}

void WorldModel::set_camera_fov(const double camera_fov) {
  camera_fov_ = camera_fov;
}