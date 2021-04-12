#ifndef BEACON_HEADER
#define BEACON_HEADER
#include <iostream>
#include <string>

struct Beacon {
  const double x_;
  const double y_;
  const std::string type_;
  Beacon(double x, double y, const std::string& type) : x_(x), y_(y), type_(type) {}
};

std::ostream& operator<<(std::ostream &os, const Beacon& b);

struct BearingBeacon {
  const double angle_;
  const std::string type_;
  BearingBeacon(double angle, const std::string& type) : angle_(angle), type_(type) {}
};
std::ostream& operator<<(std::ostream &os, const BearingBeacon& b);

#endif
