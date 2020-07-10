#include "falcon/util/FalconCLIBase.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"

class FalconROS : public libnifalcon::FalconCLIBase
{
public:
	FalconROS();
	bool parseOptions(int argc, char** argv);
	void run();
	enum
	{
		LED_OPTIONS = 0x8
	};

protected:
	void force_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	ros::NodeHandle n;
	ros::Subscriber force_sub;
	ros::Publisher pos_pub;
	ros::Publisher joy_pub;
	std::array<double, 3> next_force;
};