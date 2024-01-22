// TODO - make me use gtests to actually test
// the output is correct
#include "pf_localization/particle_filter.hpp"

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
	constexpr double rotation_threshold = 0.25;
	constexpr size_t num_particles = 1;
	const std::vector<double> sigmas = {0.025, 0.025};
	// Hard-code map beacon (tag) positions
	std::vector<PositionBeacon> beacons;
	beacons.emplace_back(10.0, 10.0, "2");
	beacons.emplace_back(2.0, 2.0, "1");
	beacons.emplace_back(0.0, 0.0, "1");
	beacons.emplace_back(1.0, 1.0, "1");

	WorldModel world(beacons, WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max), 2 * M_PI);
	auto pf = std::make_unique<ParticleFilter>(world,
			WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
			p_stdev, r_stdev, rotation_threshold,
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
	constexpr double rotation_threshold = 0.25;
	constexpr size_t num_particles = 200;
	const std::vector<double> sigmas = {0.025, 0.025};
	// Hard-code map beacon (tag) positions
	std::vector<PositionBeacon> beacons;
	beacons.emplace_back(0.0, -0.24, "51");
	beacons.emplace_back(0.0, -0.24, "17");
	beacons.emplace_back(-0.68, 1.55, "3");

	WorldModel world(beacons, WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max), 2 * M_PI);
	auto pf = std::make_unique<ParticleFilter>(world,
			WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
			p_stdev, r_stdev, rotation_threshold,
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
		pf->noise_pos(1.0);
		pf->noise_rot(1.0);
		pf->set_rotation(0);
		pf->set_rotation(0);
		pf->motion_update(0, 0, 0);
		pf->noise_pos(1.0);
		pf->noise_rot(1.0);
		pf->set_rotation(0);
		pf->set_rotation(0);
		pf->motion_update(0, 0, 0);
		pf->noise_pos(1.0);
		pf->noise_rot(1.0);
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

void test3()
{
	// Field dimensions
	constexpr double f_x_min = 0.0;
	constexpr double f_x_max = 16.458;
	constexpr double f_y_min = 0.0;
	constexpr double f_y_max = 8.225;
	// Bounding box for intial particle locations
				// 7.4, 4.4
	constexpr double i_x_min = 6.85;
	constexpr double i_x_max = 6.85;
	constexpr double i_y_min = 4.68;
	constexpr double i_y_max = 4.68;
	constexpr double p_stdev = 0.025;
	constexpr double r_stdev = 0.005;
	constexpr double rotation_threshold = 0.25;
	constexpr size_t num_particles = 200;
	const std::vector<double> sigmas = {0.100, 0.100};
	// Hard-code map beacon (tag) positions
	std::vector<PositionBeacon> beacons;
	beacons.emplace_back(13.2516346, 0.269104, "red_launchpad");
	beacons.emplace_back(13.2516346, 2.720204, "red_launchpad");

	beacons.emplace_back(16.468, 2.509, "ds_numbers");
	beacons.emplace_back(16.468, 3.7425, "ds_numbers");
	beacons.emplace_back(16.468, 5.613, "ds_numbers");
	WorldModel world(beacons, WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max), 2 * M_PI);
	auto pf = std::make_unique<ParticleFilter>(world,
			WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
			p_stdev, r_stdev, rotation_threshold,
			num_particles);

	// Fake measurements from a simulated camera frame
	std::vector<std::shared_ptr<BeaconBase>> measurement;
	measurement.push_back(std::make_shared<BearingBeacon>(1, 0.05635746568, "red_launchpad"));
	measurement.push_back(std::make_shared<BearingBeacon>(1, -0.5004112124, "red_launchpad"));
	measurement.push_back(std::make_shared<BearingBeacon>(1, 0.369244, "ds_numbers"));
	measurement.push_back(std::make_shared<BearingBeacon>(1, 0.2321, "ds_numbers"));

	constexpr double robot_rotation = 2.9;
	//constexpr double robot_rotation = 2.79277278;

	pf->allianceColorCheck(false);
	for (size_t i = 0; i < 50; i++)
	{
		// Assume IMU is 200 hz, cmd_vel is 100hz and camera is 30 hz
		// Which means for each camera frames, there are 3-ish motion updates
		// and 6 IMU updates.
		pf->set_rotation(robot_rotation);
		#if 1
		pf->set_rotation(robot_rotation);
		pf->motion_update(0, 0, 0);
		pf->noise_pos(1.0);
		pf->noise_rot(1.0);
		pf->set_rotation(robot_rotation);
		pf->set_rotation(robot_rotation);
		pf->motion_update(0, 0, 0);
		pf->noise_pos(1.0);
		pf->noise_rot(1.0);
		pf->set_rotation(robot_rotation);
		pf->set_rotation(robot_rotation);
		pf->motion_update(0, 0, 0);
		pf->noise_pos(1.0);
		pf->noise_rot(1.0);
		#endif
		if (pf->assign_weights(measurement, sigmas))
		{
			pf->resample();
			std::cout << "i=" << i << " ====================" << std::endl;
			const auto p = pf->predict();
			if (p)
			{
				// 7.4, 4.4
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

void test4()
{
	// Field dimensions
	constexpr double f_x_min = 0.0;
	constexpr double f_x_max = 16.458;
	constexpr double f_y_min = 0.0;
	constexpr double f_y_max = 8.225;
	// Bounding box for intial particle locations
	constexpr double i_x_min = 0;
	constexpr double i_x_max = 0;
	constexpr double i_y_min = 0;
	constexpr double i_y_max = 0;
	constexpr double p_stdev = 0.025;
	constexpr double r_stdev = 0.005;
	constexpr double rotation_threshold = 0.25;
	constexpr size_t num_particles = 1;
	const std::vector<double> sigmas = {1.000, 1.000};
	// Hard-code map beacon (tag) positions
	std::vector<PositionBeacon> beacons;
	beacons.emplace_back(0, 1, "a01");
	beacons.emplace_back(1, 0, "b10");
	beacons.emplace_back(-1, 0, "c-10");
	beacons.emplace_back(0,-1, "d0-1");
	WorldModel world(beacons, WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max), M_PI / 2.);
	auto pf = std::make_unique<ParticleFilter>(world,
			WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
			p_stdev, r_stdev, rotation_threshold,
			num_particles);

	// Fake measurements from a simulated camera frame
	std::vector<std::shared_ptr<BeaconBase>> measurement;
	measurement.push_back(std::make_shared<BearingBeacon>(0, 1, "a01"));

	//pf->allianceColorCheck(false);
	for (size_t i = 0; i < 1; i++)
	{
		// Assume IMU is 200 hz, cmd_vel is 100hz and camera is 30 hz
		// Which means for each camera frames, there are 3-ish motion updates
		// and 6 IMU updates.
		pf->set_rotation(-M_PI/2.);
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
	ros::Time::init();
	//test1();
	//test2();
	test3();
	//test4();
}