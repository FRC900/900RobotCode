#define _USE_MATH_DEFINES

#include "pf_localization/particle_filter.hpp"
#include "pf_localization/world_model.hpp"
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <angles/angles.h>
#include <ros/console.h>

std::ostream& operator<<(std::ostream& os, const Particle &p)
{
    os << "Particle(" << p.x_ << ", " << p.y_ << ", " << p.rot_ << ", " << p.weight_ << ")";
    return os;
}
std::ostream& operator<<(std::ostream &os, const PositionBeacon& b)
{
    os << "Beacon(" << b.x_ << ", " << b.y_ << ", " << b.type_ << ")";
    return os;
}
std::ostream& operator<<(std::ostream &os, const BearingBeacon& b)
{
    os << "Beacon(" << b.angle_ << ", " << b.type_ << ")";
    return os;
}

ParticleFilter::ParticleFilter(const WorldModel& w,
                               double x_min, double x_max, double y_min, double y_max,
                               double ns, double rs, size_t n) :
                               num_particles_(n),
                               rng_(std::mt19937(0)),
                               pos_dist_(0, ns),
                               rot_dist_(0, rs),
                               world_(w) {
  init(x_min, x_max, y_min, y_max);
}

void ParticleFilter::constrain_particles() {
  for (Particle& p : particles_) {
    world_.constrain_to_world(p);
  }
}

void ParticleFilter::init(const double x_min, const double x_max, const double y_min, const double y_max) {
  double world_x_min;
  double world_x_max;
  double world_y_min;
  double world_y_max;
  world_.get_boundaries(world_x_min, world_x_max, world_y_min, world_y_max);
  const double x_l = std::max(x_min, world_x_min);
  const double x_u = std::min(x_max, world_x_max);
  const double y_l = std::max(y_min, world_y_min);
  const double y_u = std::min(y_max, world_y_max);
  std::uniform_real_distribution<double> x_distribution(x_l, x_u);
  std::uniform_real_distribution<double> y_distribution(y_l, y_u);
  std::uniform_real_distribution<double> rot_distribution(0, 2 * M_PI);
  for (size_t i = 0; i < num_particles_; i++) {
    const double x = x_distribution(rng_);
    const double y = y_distribution(rng_);
    const double rot = rot_distribution(rng_);
    particles_.emplace_back(Particle{x, y, rot});
  }
  normalize();
}

//Normalize weights to sum to 1
void ParticleFilter::normalize() {
  double sum = 0;
  for (const Particle& p : particles_) {
    sum += p.weight_;
  }
  for (Particle& p : particles_) {
    if (sum != 0) p.weight_ /= sum;
    else p.weight_ = 1.0 / particles_.size();
  }
}

void ParticleFilter::noise_rot() {
  for (Particle& p : particles_) {
    p.rot_ += rot_dist_(rng_);
  }
}

void ParticleFilter::noise_pos() {
  for (Particle& p : particles_) {
    p.x_ += pos_dist_(rng_);
    p.y_ += pos_dist_(rng_);
  }
}

//Particles with higher weights are more likely to be resampled
void ParticleFilter::resample() {
  std::vector<Particle> new_particles;
  new_particles.reserve(num_particles_);
  for (size_t i = 0; i < num_particles_; i++) {
    const double r = ((double) rng_() - rng_.min()) / (rng_.max() - rng_.min());
    //std::cout << r << '\n';
    double a = 0;
    for (const Particle& p : particles_) {
      a += p.weight_;
      if (a > r) {
        new_particles.push_back(p);
        break;
      }
    }
  }
  particles_ = new_particles;
  normalize();
}

geometry_msgs::PoseWithCovariance ParticleFilter::predict() {
  double weight = 0;
  double x = 0;
  double y = 0;
  double s = 0;
  double c = 0;

  for (const Particle& p : particles_) {
    // Add weighted values to means
    x += p.x_ * p.weight_;
    y += p.y_ * p.weight_;
    c += cos(p.rot_) * p.weight_;
    s += sin(p.rot_) * p.weight_;
    weight += p.weight_;
  }

  // Divide by weight to get the weighted average
  x /= weight;
  y /= weight;
  c /= weight;
  s /= weight;
  double rot = atan2(s, c);

  double covariance[9] = {0};
  for (const Particle& p : particles_) {
    // Put distances into an array so they can be accessed in a loop
    double differences[3] = {p.x_ - x, p.y_ - y, angles::shortest_angular_distance(p.rot_, rot)};

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        int index = i * 3 + j;
        covariance[index] += differences[i] * differences[j] * p.weight_;
      }
    }
  }

  // Packing everything into PoseWithCovariance
  geometry_msgs::PoseWithCovariance pose;

  std::fill(std::begin(pose.covariance), std::begin(pose.covariance)+36, 1000);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      // Offset to skip to z rotation
      int row = i > 1 ? i + 3 : i;
      int column = j > 1 ? j + 3 : j;
      pose.covariance[row * 6 + column] = covariance[i * 3 + j] / weight;
    }
  }

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, rot);
  pose.pose.orientation.x = q.getX();
  pose.pose.orientation.y = q.getY();
  pose.pose.orientation.z = q.getZ();
  pose.pose.orientation.w = q.getW();

  return pose;
}

bool ParticleFilter::motion_update(double delta_x, double delta_y, double delta_rot) {
  for (Particle& p : particles_) {
    // p.x_ += delta_x;
    // p.y_ += delta_y;
    const double abs_delta_x = delta_x * cos(p.rot_) + delta_y * sin(p.rot_);
    const double abs_delta_y = delta_x * sin(p.rot_) + delta_y * cos(p.rot_);
    // TODO - do we really want to return here, or should it try to
    // motion update the rest of the particles?
    // Maybe split the check - if d_x or d_y are inf/nan, nothing we can do
    // If a single particle rotation is inf/nan, reset it to 0 or a
    // a random number in [-pi,pi]?
    // Then hoist the check for d_x, d_y outside the loop, return false for
    // that case. Remove the return false here to the particles which do
    // pass are constrained properly
#ifdef CHECK_NAN
    if (std::isnan(delta_rot + abs_delta_x + abs_delta_y) || std::isinf(delta_rot + abs_delta_x + abs_delta_y)) {
      return false;
    }
#endif
    p.rot_ += delta_rot;
    p.x_ += abs_delta_x;
    p.y_ += abs_delta_y;
  }
  constrain_particles();
  return true;
}

bool ParticleFilter::set_rotation(double rot) {
#ifdef CHECK_NAN
  if (std::isnan(rot) || std::isinf(rot)) {
    return false;
  }
#endif
  for (Particle& p : particles_) {
    p.rot_ = rot;
  }
  noise_rot();
  return true;
}

//assigns the reciprocal of the computed error of each particles assignment vector to the respective particle
bool ParticleFilter::assign_weights(const std::vector<std::shared_ptr<BeaconBase>> &mBeacons) {
  if (mBeacons.size() == 0)
	return false;

#ifdef CHECK_NAN
  double test_sum = offset.x_ + offset.y_ + offset.rot_;
  for (const auto& b : mBeacons) {
    test_sum += b->debugSum();
  }
  if (std::isnan(test_sum) || std::isinf(test_sum)) {
    return false;
  }
#endif

  std::vector<double> total_distances;
  for (Particle& p : particles_) {
    total_distances.push_back(world_.total_distance(p, mBeacons));
    if (total_distances.back() == 0)
      return false;
  }
  for (size_t i = 0; i < particles_.size(); i++) {
    particles_[i].weight_ = 1.0 / total_distances[i];
  }
  normalize();
  return true;
}

std::vector<Particle> ParticleFilter::get_particles() const {
  return particles_;
}

void ParticleFilter::check_particles(const char *file, int line) const {
  for (const auto &p : particles_)
  {
    if (!p.isValid())
    {
      ROS_INFO_STREAM("check_particles failed " << file << " : " << line);
      return;
    }
  }
}
