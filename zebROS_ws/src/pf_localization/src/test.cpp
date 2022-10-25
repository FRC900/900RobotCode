// TODO - make me use gtests to actually test
// the output is correct
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "pf_localization/particle_filter.hpp"

// Given a list of beacons relative to the blue corner of the field,
// generate the same list of beacons but translated so the origin is
// the red corner of the field.
std::vector<PositionBeacon> getRedBeacons(const std::vector<PositionBeacon> &blueBeacons)
{
	constexpr double field_width = 16.458;
	constexpr double field_height = 8.228; 
	ros::Time now;
	now.fromSec(0); // dummy time for testing
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

// Test 1 iteration with 1 particle at 0,0
// Useful for basic debugging
void test1(void)
{
	// Field dimensions
	constexpr double f_x_min = -6.0;
	constexpr double f_x_max =  6.0;
	constexpr double f_y_min = -6.0;
	constexpr double f_y_max =  6.0;
	// Bounding box for intial particle locations
	constexpr double i_x_min =  0.0;
	constexpr double i_x_max =  0.0;
	constexpr double i_y_min =  0.0;
	constexpr double i_y_max =  0.0;
	constexpr double p_stdev = 0.01;
	constexpr double r_stdev = 0.01;
	constexpr size_t num_particles = 1;
	const std::vector<double> sigmas = {0.025, 0.025};
	// Hard-code map beacon (tag) positions
	std::vector<PositionBeacon> beacons;
	beacons.emplace_back(PositionBeacon{10.0, 10.0, "2"});
	beacons.emplace_back(PositionBeacon{2.0, 2.0, "1"});
	beacons.emplace_back(PositionBeacon{0.0, 0.0, "1"});
	beacons.emplace_back(PositionBeacon{1.0, 1.0, "1"});

	WorldModel world(beacons, getRedBeacons(beacons), WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max));
	auto pf = std::make_unique<ParticleFilter>(world,
			WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
			p_stdev, r_stdev,
			num_particles);

	// Fake measurements from a simulated camera frame
	std::vector<std::shared_ptr<BeaconBase>> measurement;
	measurement.push_back(std::make_shared<PositionBeacon>(10.0, 10.0, "2"));
	measurement.push_back(std::make_shared<PositionBeacon>(0.05, 0.05, "1"));
	measurement.push_back(std::make_shared<PositionBeacon>(1.05, 1.05, "1"));

	pf->set_rotation(0);
	if (pf->assign_weights(measurement, sigmas))
	{
		pf->resample();
		const auto p = pf->predict();
		if (p)
		{ 
			std::cout << p->pose.position;
		}
		else
		{
			std::cout << "No valid prediction";
		}
	}
	std::cout << "************************* DONE *****************************" << std::endl;
}

// Multiple iterations with constant detections
// Should converge to near
//    x: -2.8
//    y: 0.90
//    z: 0
// In a few (< ~10) iterations
void test2()
{
	// Field dimensions
	constexpr double f_x_min = -6.0;
	constexpr double f_x_max =  6.0;
	constexpr double f_y_min = -6.0;
	constexpr double f_y_max =  6.0;
	// Bounding box for intial particle locations
	constexpr double i_x_min = -6.0;
	constexpr double i_x_max =  6.0;
	constexpr double i_y_min = -6.0;
	constexpr double i_y_max =  6.0;
	constexpr double p_stdev = 0.01;
	constexpr double r_stdev = 0.01;
	constexpr size_t num_particles = 200;
	const std::vector<double> sigmas = {0.025, 0.025};
	// Hard-code map beacon (tag) positions
	std::vector<PositionBeacon> beacons;
	beacons.emplace_back(PositionBeacon{0.0, -0.24, "51"});
	beacons.emplace_back(PositionBeacon{0.0, -0.24, "17"});
	beacons.emplace_back(PositionBeacon{-0.68, 1.55, "3"});

	WorldModel world(beacons, getRedBeacons(beacons), WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max));
	auto pf = std::make_unique<ParticleFilter>(world,
			WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
			p_stdev, r_stdev,
			num_particles);

	// Fake measurements from a simulated camera frame
	std::vector<std::shared_ptr<BeaconBase>> measurement;
	measurement.push_back(std::make_shared<PositionBeacon>(2.06598, 0.745478, "3"));
	measurement.push_back(std::make_shared<PositionBeacon>(2.88372, -1.12832, "17"));
	measurement.push_back(std::make_shared<PositionBeacon>(2.8136, -1.0654, "51"));

	for (size_t i = 0; i < 25; i++)
	{
		// Assume IMU is 200 hz, cmd_vel is 100hz and camera is 30 hz
		// Which means for each camera frames, there are 3-ish motion updates
		// and 6 IMU updates.
		pf->set_rotation(0);
		pf->set_rotation(0);
		pf->motion_update(0, 0, 0);
		pf->noise_pos();
		pf->noise_rot();
		pf->set_rotation(0);
		pf->set_rotation(0);
		pf->motion_update(0, 0, 0);
		pf->noise_pos();
		pf->noise_rot();
		pf->set_rotation(0);
		pf->set_rotation(0);
		pf->motion_update(0, 0, 0);
		pf->noise_pos();
		pf->noise_rot();
		if (pf->assign_weights(measurement, sigmas))
		{
			pf->resample();
			std::cout << "i=" << i << " ====================" << std::endl;
			const auto p = pf->predict();
			if (p)
			{
				std::cout << p->pose.position;
			}
			else
			{
				std::cout << "No valid prediction";
			}
		}
	}
	std::cout << "************************* DONE *****************************" << std::endl;
}

int main (void)
{
	test1();
	test2();
}
