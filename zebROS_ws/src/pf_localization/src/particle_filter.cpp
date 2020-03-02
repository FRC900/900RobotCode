#include "particle_filter.hpp"
#include "world_model.hpp"
#include <random>
#include <vector>
#include <utility>

#include <iostream>
#include <ros/ros.h>


ParticleFilter::ParticleFilter(const WorldModel& w,
                               double x_min, double x_max, double y_min, double y_max,
                               double ns, double rs, size_t n) :
                               world_(w), num_particles_(n),
                               noise_stdev_(ns), rot_noise_stdev_(rs) {
  rng_ = std::mt19937(0);
  init(x_min, x_max, y_min, y_max);
}

void ParticleFilter::constrain_particles() {
  for (Particle& p : particles_) {
    world_.constrain_to_world(p);
  }
}

void ParticleFilter::init(const double x_min, const double x_max, const double y_min, const double y_max) {
  std::vector<double> bounds = world_.get_boundaries();
  double x_l = std::max(x_min, bounds[0]);
  double x_u = std::min(x_max, bounds[1]);
  double y_l = std::max(y_min, bounds[2]);
  double y_u = std::min(y_max, bounds[3]);
  for (size_t i = 0; i < num_particles_; i++) {
    std::uniform_real_distribution<double> x_distribution(x_l, x_u);
  	std::uniform_real_distribution<double> y_distribution(y_l, y_u);
    std::uniform_real_distribution<double> rot_distribution(0, 6.283);
    double x = x_distribution(rng_);
    double y = y_distribution(rng_);
    double rot = rot_distribution(rng_);
    Particle p = {x, y, rot};
    particles_.push_back(p);
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
  std::normal_distribution<double> rot_dist(0, rot_noise_stdev_);
  for (Particle& p : particles_) {
    p.rot_ += rot_dist(rng_);
  }
}

void ParticleFilter::noise_pos() {
  std::normal_distribution<double> pos_dist(0, noise_stdev_);
  for (Particle& p : particles_) {
    p.x_ += pos_dist(rng_);
    p.y_ += pos_dist(rng_);
  }
}

//Particles with higher weights are more likely to be resampled
void ParticleFilter::resample() {
  std::vector<Particle> new_particles;
  new_particles.reserve(num_particles_);
  for (int i = 0; i < num_particles_; i++) {
    const double r = ((double) rng_() - rng_.min()) / (rng_.max() - rng_.min());
    // std::cout << r << '\n';
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

Particle ParticleFilter::predict() {
  double weight = 0;
  Particle res {0, 0, 0};
  double s = 0;
  double c = 0;
  for (const Particle& p : particles_) {
    res.x_ += p.x_ * p.weight_;
    res.y_ += p.y_ * p.weight_;
    c += cos(p.rot_) * p.weight_;
    s += sin(p.rot_) * p.weight_;
    weight += p.weight_;
  }
  res.x_ /= weight;
  res.y_ /= weight;
  c /= weight;
  s /= weight;
  res.rot_ = atan2(s, c);
  return res;
}

void ParticleFilter::motion_update(double delta_x, double delta_y, double delta_rot) {
  for (Particle& p : particles_) {
    // p.x_ += delta_x;
    // p.y_ += delta_y;
    p.rot_ += delta_rot;
    p.x_ += delta_x * cos(p.rot_) + delta_y * sin(p.rot_);
    p.y_ += delta_x * sin(p.rot_) + delta_y * cos(p.rot_);
  }
  constrain_particles();
}

void ParticleFilter::set_rotation(double rot) {
  for (Particle& p : particles_) {
    p.rot_ = rot;
  }
  noise_rot();
}

//assigns the reciprocal of the computed error of each particles assignment vector to the respective particle
void ParticleFilter::assign_weights(std::vector<std::pair<double, double> > mBeacons) {
  for (Particle& p : particles_) {
    p.weight_ = 1 / world_.total_distance(p, mBeacons);
  }
  normalize();
}

std::vector<Particle> ParticleFilter::get_particles() const {
  return particles_;
}
