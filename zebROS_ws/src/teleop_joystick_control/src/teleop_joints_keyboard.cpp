#include <teleop_joystick_control/teleop_joints_keyboard.h>
#include <termios.h>

#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6a
#define KEYCODE_k 0x6b
#define KEYCODE_l 0x6c
#define KEYCODE_m 0x6d
#define KEYCODE_n 0x6e
#define KEYCODE_o 0x6f
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x7
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7a
#define KEYCODE_A 0x41
#define KEYCODE_B 0x42
#define KEYCODE_C 0x43
#define KEYCODE_D 0x44
#define KEYCODE_MINUS 0x2D
#define KEYCODE_EQUALS 0x3D
#define KEYCODE_ONE 0x31
#define KEYCODE_TWO 0x32
#define KEYCODE_THREE 0x33
#define KEYCODE_FOUR 0x34
#define KEYCODE_FIVE 0x35
#define KEYCODE_SIX 0x36
#define KEYCODE_SEVEN 0x37
#define KEYCODE_EIGHT 0x38
#define KEYCODE_LEFT_BRACKET 0x5B
#define KEYCODE_ESCAPE  0x1B
#define KEYCODE_CARROT 0x5E
#define KEYCODE_SPACE 0x20
#define KEYCODE_COMMA 0x2C

TeleopJointsKeyboard::TeleopJointsKeyboard(ros::NodeHandle &nh)
{
	// Hard-code this to frcrobot_rio namespace so that it matches
	// the real robot hardware, where joystick data comes from the
	// driver station via the Rio
	joints_pub_ = nh.advertise<sensor_msgs::Joy>("/frcrobot_rio/joystick_states_raw", 1);
}

TeleopJointsKeyboard::~TeleopJointsKeyboard()
{
}

// Code which waits for a set period of time for a keypress.  If
// the keyboard is pressed in that time, read the key press and set
// it equal to c, then return 1 character read.  If nothing is seen,
// return 0.  Return <0 on error.
int TeleopJointsKeyboard::pollKeyboard(int kfd, char &c) const
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 20*1000; // 20 mSec keyboard polling rate

	// read_fds is an array of file descriptors to wait for
	// In this case, we only want to wait for 1 - the keyboard
	// fd passed in as kfd
	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(kfd, &read_fds);

	// Select returns when
	//   1. there is data present on one of the requested fds (returns > 0)
	//   2. the timeout is exceeded (returns 0)
	int rc = select(kfd + 1, &read_fds, NULL, NULL, &tv);
	if (rc < 0)
	{
		perror("select():");
	}
	else if (rc > 0)  // if select didn't timeout
	{
		rc = read(kfd, &c, 1);
		if (rc < 0)
		{
			perror("read():");
		}
	}
	return rc;
}

void TeleopJointsKeyboard::keyboardLoop()
{
	int kfd = 0; // stdin
	char c;
	// get the console in raw mode
	struct termios cooked;
	tcgetattr(kfd, &cooked);
	struct termios raw;
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	// Set read() to return immediately so we
	// can break out of the loop immediately rather than waiting
	// on a timeout from reading stdin
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;

	tcsetattr(kfd, TCSANOW, &raw);

	cmd_.axes.resize(8);
	cmd_.buttons.resize(11);

	cmd_.axes[0] = 0.0;
	cmd_.axes[1] = 0.0;
	cmd_.axes[2] = 0.0;
	cmd_.axes[3] = 0.0;
	cmd_.axes[4] = 0.0;
	cmd_.axes[5] = 0.0;
	cmd_.axes[6] = 0.0;
	cmd_.axes[7] = 0.0;

	cmd_.buttons[0] = false;
	cmd_.buttons[1] = false;
	cmd_.buttons[2] = false;
	cmd_.buttons[3] = false;
	cmd_.buttons[4] = false;
	cmd_.buttons[5] = false;
	cmd_.buttons[6] = false;
	cmd_.buttons[7] = false;
	cmd_.buttons[8] = false;
	cmd_.buttons[9] = false;
	cmd_.buttons[10] = false;

	bool processing_bracket = false;
	bool dirty = false;

	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		// Publish command
		if (!dirty)
		{
			cmd_.header.stamp = ros::Time::now();
			joints_pub_.publish(cmd_);
		}

		ros::spinOnce();
		loop_rate.sleep();

		int rc = pollKeyboard(kfd, c);
		if (rc < 0)
			break;
		else if (rc == 0)
			continue;

		// Assume keypress will be a valid command / cause
		// to update the published joystick data. Set this to false
		// for cases where it isn't true.
		dirty = false;

		if (!processing_bracket)
		{
			ROS_INFO_STREAM("Processing axes and buttons");
			switch (c)
			{
				case KEYCODE_a:
					if(cmd_.axes[0] > -1.0)
					{
						cmd_.axes[0] -= 0.5;
					}
					break;
				case KEYCODE_d:
					if(cmd_.axes[0] < 1.0)
					{
						cmd_.axes[0] += 0.5;
					}
					break;
				case KEYCODE_w:
					if(cmd_.axes[1] < 1.0)
					{
						cmd_.axes[1] += 0.5;
					}
					break;
				case KEYCODE_s:
					if(cmd_.axes[1] > -1.0)
					{
						cmd_.axes[1] -= 0.5;
					}
					break;
				case KEYCODE_q:
					if(cmd_.axes[2] < 1.0)
					{
						cmd_.axes[2] += 0.5;
					}
					break;
				case KEYCODE_z:
					if(cmd_.axes[2] > 0.0)
					{
						cmd_.axes[2] -= 0.5;
					}
					break;
				case KEYCODE_o:
					if(cmd_.axes[3] < 1.0)
					{
						cmd_.axes[3] += 0.5;
					}
					break;
				case KEYCODE_COMMA:
					if(cmd_.axes[3] > 0.0)
					{
						cmd_.axes[3] -= 0.5;
					}
					break;
				case KEYCODE_j:
					if(cmd_.axes[4] > -1.0)
					{
						cmd_.axes[4] -= 0.5;
					}
					break;
				case KEYCODE_l:
					if(cmd_.axes[4] < 1.0)
					{
						cmd_.axes[4] += 0.5;
					}
					break;
				case KEYCODE_i:
					if(cmd_.axes[5] < 1.0)
					{
						cmd_.axes[5] += 0.5;
					}
					break;
				case KEYCODE_k:
					if(cmd_.axes[5] > -1.0)
					{
						cmd_.axes[5] -= 0.5;
					}
					break;

				case KEYCODE_ONE:
					cmd_.buttons[0] = true;
					break;
				case KEYCODE_TWO:
					cmd_.buttons[1] = true;
					break;
				case KEYCODE_THREE:
					cmd_.buttons[2] = true;
					break;
				case KEYCODE_FOUR:
					cmd_.buttons[3] = true;
					break;
				case KEYCODE_e:
					cmd_.buttons[4] = true;
					break;
				case KEYCODE_u:
					cmd_.buttons[5] = true;
					break;
				case KEYCODE_FIVE:
					cmd_.buttons[6] = true;
					break;
				case KEYCODE_SIX:
					cmd_.buttons[7] = true;
					break;
				case KEYCODE_x:
					cmd_.buttons[8] = true;
					break;
				case KEYCODE_m:
					cmd_.buttons[9] = true;
					break;
				case KEYCODE_SEVEN:
					cmd_.buttons[10] = true;
					break;

				case KEYCODE_r:
					ROS_INFO_STREAM("Resetting buttons to false!");
					for(size_t i = 0; i <= cmd_.buttons.size(); i++)
					{
						cmd_.buttons[i] = false;
					}
					break;

				case KEYCODE_LEFT_BRACKET:
					processing_bracket = true;
					dirty = true;
					break;
				case  KEYCODE_ESCAPE:
					//std::cout << std::endl;
					//std::cout << "Exiting " << std::endl;
					//quit(0);
					break;
				case KEYCODE_CARROT:
					ROS_WARN("It's a carrot");
					dirty = true;
					break;
				default:
					dirty = true;
					break;
			}
		}
		else // Processing bracket
		{
			ROS_INFO_STREAM("Running Dpad");
			switch (c)
			{
				case KEYCODE_B:
					if(cmd_.axes[7] > -1.0)
					{
						cmd_.axes[7] -= 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_A:
					if(cmd_.axes[7] < 1.0)
					{
						cmd_.axes[7] += 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_D:
					if(cmd_.axes[6] < 1.0)
					{
						cmd_.axes[6] += 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_C:
					if(cmd_.axes[6] > -1.0)
					{
						cmd_.axes[6] -= 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_LEFT_BRACKET:
					processing_bracket = true;
					break;
				default:
					dirty = true;
					break;
			}
		}
	}
	// Restore sanity to keyboard input
	tcsetattr(kfd, TCSANOW, &cooked);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_joints_keyboard");
	ros::NodeHandle nh;
	TeleopJointsKeyboard keyboard(nh);
	keyboard.keyboardLoop();
	return 0;
}
