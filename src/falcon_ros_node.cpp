#include <iostream>
#include <csignal>
#include "falcon/core/FalconDevice.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/util/FalconCLIBase.h"

#include "FalconTestBase.h"
#include "falcon/gmtl/Vec.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

bool stop = false;

void sigproc(int i)
{
	if(!stop)
	{
		stop = true;
		std::cout << "Quitting" << std::endl;
	}
	else exit(0);
}

class FalconROS : public libnifalcon::FalconCLIBase
{
public:
	FalconROS()
	{
		m_falconDevice->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
		m_falconDevice->setFalconGrip<libnifalcon::FalconGripFourButton>();
		force_sub = n.subscribe("falcon/force_desired", 1000, &FalconROS::force_cb, this);
		pos_pub = n.advertise<geometry_msgs::Vector3>("falcon/position_current", 1000);
		next_force = {0,0,0};
	}

	enum
	{
		LED_OPTIONS = 0x8
	};

	bool parseOptions(int argc, char** argv)
	{
		libnifalcon::FalconCLIBase::addOptions(FalconROS::LED_OPTIONS | FalconROS::DEVICE_OPTIONS | FalconROS::COMM_OPTIONS | FalconROS::FIRMWARE_OPTIONS);
		if(!FalconCLIBase::parseOptions(argc, argv)) return false;
		optparse::Values options = m_parser.parse_args(argc, argv);
  		return true;
	}
	void run()
	{
		while(!calibrateDevice() && !stop);
		ros::Rate loop_rate(1000);
		while (ros::ok())
		{
			if(!m_falconDevice->runIOLoop())
			{
				loop_rate.sleep();
				continue;
			}
			std::array<double, 3> pos = m_falconDevice->getPosition();
			geometry_msgs::Vector3 msg;
			msg.x = pos[0];
			msg.y = pos[1];
			msg.z = pos[2];
			pos_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			// std::cout << "test" << std::endl;
			m_falconDevice->setForce(next_force);
		}
	}
protected:
	void force_cb(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		next_force[0] = msg->x;
		next_force[1] = msg->y;
		next_force[2] = msg->z;
	}
	ros::NodeHandle n;
	ros::Subscriber force_sub;
	ros::Publisher pos_pub;
	std::array<double, 3> next_force;
};

int main(int argc, char** argv)
{
	signal(SIGINT, sigproc);
#ifndef WIN32
	signal(SIGQUIT, sigproc);
#endif
	ros::init(argc, argv, "falcon_ros_node");
	FalconROS f;
	if(!f.parseOptions(argc, argv))
		return 0;
	f.run();
	return 0;
}