#include <iostream>
#include <csignal>
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "falcon_ros_node.h"

// TODO:
// Figure out how argc / argv work

bool stop = false;

void sigproc(int i)
{
	// Function to still obey interrupts when calling hanging Falcon functions such as calibration
	if(!stop)
	{
		stop = true;
		std::cout << "Quitting" << std::endl;
	}
	else exit(0);
}

FalconROS::FalconROS()
{
	// Set up falcon-specific parameters
	m_falconDevice->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
	m_falconDevice->setFalconGrip<libnifalcon::FalconGripFourButton>();
	// Set up ros node
	force_sub = n.subscribe("falcon/force_desired", 1, &FalconROS::force_cb, this);
	joy_pub = n.advertise<sensor_msgs::Joy>("falcon/joy", 1000);
	pos_pub = n.advertise<geometry_msgs::PoseStamped>("falcon/position_current", 1000);
	// Initialize empty force
	next_force = {0,0,0};
}

bool FalconROS::parseOptions(int argc, char** argv)
{
	// Parse falcon command line options. See 
	libnifalcon::FalconCLIBase::addOptions(FalconROS::LED_OPTIONS | FalconROS::DEVICE_OPTIONS | FalconROS::COMM_OPTIONS | FalconROS::FIRMWARE_OPTIONS);
	if(!FalconCLIBase::parseOptions(argc, argv)) return false;
	optparse::Values options = m_parser.parse_args(argc, argv);
	return true;
}

void FalconROS::run()
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
		ros::Time t = ros::Time::now();
		// Publish pose message
		std::array<double, 3> pos = m_falconDevice->getPosition();
		geometry_msgs::PoseStamped pos_msg;
		pos_msg.header.stamp = t;
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = pos[1];
		pos_msg.pose.position.z = pos[2];
		pos_msg.pose.orientation.x = 0;
		pos_msg.pose.orientation.y = 0;
		pos_msg.pose.orientation.z = 0;
		pos_msg.pose.orientation.w = 1;
		pos_pub.publish(pos_msg);
		// Publish joystick message
		uint8_t buttons = m_falconDevice->getFalconGrip()->getDigitalInputs();
		sensor_msgs::Joy joy_msg;
		joy_msg.header.stamp = t;
		joy_msg.axes = {(float)pos[0], (float)pos[1], (float)pos[2]};
		joy_msg.buttons = { (buttons & libnifalcon::FalconGripFourButton::CENTER_BUTTON)  > 0 ,
							(buttons & libnifalcon::FalconGripFourButton::FORWARD_BUTTON) > 0,
							(buttons & libnifalcon::FalconGripFourButton::PLUS_BUTTON)    > 0,
							(buttons & libnifalcon::FalconGripFourButton::MINUS_BUTTON)   > 0 };
		joy_pub.publish(joy_msg);

		ros::spinOnce();
		loop_rate.sleep();
		m_falconDevice->setForce(next_force);
	}
}

void FalconROS::force_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	next_force[0] = msg->vector.x;
	next_force[1] = msg->vector.y;
	next_force[2] = msg->vector.z;
}

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