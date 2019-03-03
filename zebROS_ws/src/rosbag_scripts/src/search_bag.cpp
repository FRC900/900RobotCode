#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <fstream>
#include <frc_msgs/MatchSpecificData.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		std::cout << "need a file thanks \n";
		return 1;
	}
	if(argc > 2)
	{
		std::cout << "why so many files ahhh \n";
		return 1;
	}

	std::string bag_name = argv[1];

	rosbag::Bag bag;
	bag.open(bag_name, rosbag::bagmode::Read);

	std::ofstream temp_file;
	temp_file.open ("temp_file.txt");

        temp_file << "Writing to a file";

	std::vector<std::string> topics;
	topics.push_back(std::string("/frcrobot_rio/match_data"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view)
	{
		frc_msgs::MatchSpecificData::ConstPtr s = m.instantiate<frc_msgs::MatchSpecificData>();
		if (s != NULL){
			if(static_cast<bool>(s->Enabled))
			{
				temp_file << s->matchTimeRemaining << std::endl;
				temp_file << static_cast<int>(s->allianceColor) << std::endl;
				temp_file << static_cast<int>(s->matchType) << std::endl;
				temp_file << static_cast<int>(s->driverStationLocation) << std::endl;
				temp_file << "matchNumber: " << static_cast<int>(s->matchNumber) << std::endl;
				temp_file << static_cast<bool>(s->Enabled) << std::endl;
				temp_file << static_cast<bool>(s->Disabled) << std::endl;
				temp_file << static_cast<bool>(s->Autonomous) << std::endl;
			}
		}

	}
	bag.close();

	return 0;
}

