
class DistanceAndBearing
{
	public:
		DistanceAndBearing(const double distance, const double bearing)
			: distance_(distance)
			, bearing_(bearing)
		{
		}

		distance(void) const
		{
			return distance_;
		}
		bearing(void) const
		{
			return bearing_;
		}
	private:
		double distance_;
		double bearing_;
};

class BeaconCoord
{
	public:
		BeaconCoord(const double x, const double y, const double angle)
			: x_(x)
			, y_(y)
			, angle_(angle)
		{
		}

		double x(void)     const { return x_; }
		double y(void)     const { return y_; }
		double angle(void) const { return angle_; }

	private:
		double x_;
		double y_;
		double angle_;
};

// Beacon - holds field-centric x,y,angle coordinates
//         of targets on the field
class Beacon
{
	public:
		Beacon(const BeaconCoord &state)
			: coord_(state)
		{
		}

		const BeaconCoord &Get(void) const
		{
			return coord_;
		}

		DistanceAndBearing distance(const Coord &coord, bool reverse_angle = false) const
		{
			double dx = coord_.x() - coord.x();
			double dy = coord_.y() - coord.y();
			if (reverse_angle)
			{
				dx = -dx;
				dy = -dy;
			}
			const double distance = hypot(dx, dy);
			const double bearing  = angles::normalize_angle(atan2(dy, dx));
			return DistanceAndBearing(distance, bearing);
		}

		double robotBearing(const double angle) const
		{
			const double abs_bearing = M_PI - angle;
			const double rel_bearing = abs_bearing - coord_.angle();
			return angles::normalize_angle(rel_bearing);
		}
	private:
		BeaconCoord coord_;
};

class Beacons
{
	public:
		Beacons(const std::vector<Beacons> &beacons)
			: beacons_(beacons)
		{
		}

		void append(const Beacon &beacon)
		{
			beacons_.push_back(beacon);
		}

		bool at(size_t i, Beacon &beacon) const
		{
			if (i < beacons_.size())
			{
				beacon = beacons_[i];
				return true;
			}
			return false;
		}
		constexpr double invalidCost = 1e6;
		constexpr double maxDetectionDistance = 7.0; //meters

		std::vector<double> getCosts(const Coord &robotLoc, const Detection &detection)
		{
			std::vector<double> costs;
			costs.resize(beacons_.size());

			auto beaconDistanceAndAngle = detection.distanceAndAngle();
			for (size_t i = 0; i < beacons_.size(); i++)
			{
				auto robotDistanceAndAngle = beacons_[i].distance(robotLoc);
				if (robotDistanceAndAngle > maxDetectionDistance)
				{
					costs[i] = invalidCost;
					continue;
				}
				if (fabs(angles::normalize_angle(robotDistanceAndAngle.angle() - robotLoc.angle())) > 0.85)
				{
					costs[i] = invalidCost;
					continue;
				}
				const double deltaRange = fabs(beaconDistanceAndAngle.distance() - robotDistanceAndAngle.distance());
				const double deltaAngle = fabs(beaconDistanceAndAngle.angle() - robotDistanceAndAngle.angle());


				// Check that angle from target to robot is
				// between +/- pi/2. This is a quick way to rule
				// out cases where it would be looking through
				// a wall to see the target
				reverseAngle = fabs(beacons_[i].robotBearing(beaconDistanceAndAngle.angle()));
				if (reverseAngle >= (M_PI / 2.0))
				{
					costs[i] = invalidCost;
					continue;
				}
			}

		}

	private:
		std::vector<Beacon> beacons_;
}
